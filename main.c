#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "hd44780_driver.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f10x_usart.h"
//-------------------------------------------------------------
#define SYSCLOCK 72000000U
#define impKM 14850U
#define hour 3600U
#define sec 1000000U
#define Kspd 242424U //sec*hour/impKM
#define BAUDRATE 256000U
//----------------------------------------------------------
volatile uint32_t SysTick_CNT = 0;
volatile uint32_t lastFlash = 0;
uint32_t lastTime = 0;
uint8_t shownum = 0;
volatile float KMPH = 0.0;
uint16_t adc_res;
char strbuf[13];
uint16_t intbuf = 0;
uint32_t itertime = 0;
//**********************К алгоритму************************
#define m0 14000U
#define p0 827U// = avP + d * dpP
#define delta_t 10000U
#define b 300U
#define J 1000U
#define L 5U
//------------------------------вход--------------------------------
volatile uint8_t f = 0;
//------------------------------калибровка--------------------------
float k = 0.0;
float avKF = 0.0;
//------------------------------общие-------------------------------
float avPfree = 0.0, avP = 0.0;
float avAfree = 0.0, avA = 0.0;
uint16_t g = 0;
uint16_t s = 0;
uint16_t i = 0;
uint16_t j = 0;
uint16_t p[J];
float v[J];
uint32_t t[J];
float a[J];
float KL[L];
float KF[L];
uint8_t lenKF = L;
float v0 = 1.0; // (2)
float kbuf = 0.0;
//------------------------------выход-------------------------------
float mk = 0.0;
float avMKF = 0.0;
float M = 0.0;
//----------------------------prototypes----------------------------
void SetSysClockTo72(void);
void SysTick_Init(void);
void initialization(void);
void postformat(char* str, int len);
void show(void);
void write_log(void);
float fsumPlusSqr(float *X, uint8_t sqr, int begin, int end);
float SKO(float *X, int sizeX);
float SKOi(float *X, int sizeX, int pos);
float RErr(float *X, int sizeX);
int lenNoZero(float *X, int sizeX);
void filterKF(void);
void axleLoad(void);
char *gcvt(double number, int ndigit, char *buf);
//==========================================================================

void SetSysClockTo72(void) {
	SET_BIT(RCC->CR, RCC_CR_HSEON);
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY == RESET)) {
	}
	//Enable the Prefetch strBuffer
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
	MODIFY_REG(RCC->CFGR,
			RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL,
			RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != (RCC_CR_PLLRDY)) {
	}
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
	}
}
//----------------------------------------------------------
void SysTick_Init(void) {
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, SYSCLOCK / 1000000 - 1);
	CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	SET_BIT(SysTick->CTRL,
			SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

void initialization(void) {
	SetSysClockTo72();
	SysTick_Init();
	GPIO_InitTypeDef PORT;
	//Затактируем порты
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
    //Прерывания - это альтернативная функция порта,
    //поэтому надо установить бит Alternate function I/O clock enable
    //в регистре RCC_APB2ENR
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);
    // Настроим ноги со светодиодами на выход
	PORT.GPIO_Pin = GPIO_Pin_11;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &PORT);

    //настраиваем вывод на вход
	PORT.GPIO_Pin = (GPIO_Pin_0);
	PORT.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &PORT);

	PORT.GPIO_Pin = (GPIO_Pin_2 | GPIO_Pin_3);
	PORT.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &PORT);

    //Выключаем JTAG (он занимает ноги нужные нам)
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    // Настраиваем ногу PA10 как вход UARTа (RxD)
	PORT.GPIO_Pin = GPIO_Pin_10;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &PORT);

    // Настраиваем ногу PA9 как выход UARTа (TxD)
    // Причем не просто выход, а выход с альтернативной функцией
	PORT.GPIO_Pin = GPIO_Pin_9;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &PORT);

    //прерывания
	EXTI_InitTypeDef ei;
	ei.EXTI_Line = (EXTI_Line0 | EXTI_Line2 | EXTI_Line3);
	ei.EXTI_LineCmd = ENABLE;
	ei.EXTI_Mode = EXTI_Mode_Interrupt;
	ei.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&ei);

	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE); //Включаем тактирование АЦП
	ADC1->CR2 |= ADC_CR2_CAL; //Запуск калибровки АЦП
	while (!(ADC1->CR2 & ADC_CR2_CAL))
		; //Ожидаем окончания калибровки
	ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0); //Задаем длительность выборки
	ADC1->CR2 |= ADC_CR2_JEXTSEL; //Преобразование инжектированной группы
    //запустится установкой бита JSWSTART
	ADC1->CR2 |= ADC_CR2_JEXTTRIG; //Разрешаем внешний запуск инжектированной группы
	ADC1->CR2 |= ADC_CR2_CONT; //Преобразования запускаются одно за другим
	ADC1->CR1 |= ADC_CR1_JAUTO; //Разрешить преобразование инжектированной группы
    //после регулярной. Не понятно зачем, но без этого не работает
	ADC1->JSQR |= (1 << 15); //Задаем номер канала (выбран ADC1)
	ADC1->CR2 |= ADC_CR2_ADON; //Теперь включаем АЦП
	ADC1->CR2 |= ADC_CR2_JSWSTART; //Запуск преобразований
	while (!(ADC1->SR & ADC_SR_JEOC))
		//ждем пока первое преобразование завершится
		;
	 //Теперь можно читать результат из JDR1

	lcd_init(); //Инициализируем дисплей
	lcd_set_state(LCD_ENABLE, CURSOR_DISABLE, NO_BLINK); //Выключаем курсор и мигалку

	//Заполняем структуру настройками UARTa
	USART_InitTypeDef uart_struct;
	uart_struct.USART_BaudRate = BAUDRATE;
	uart_struct.USART_WordLength = USART_WordLength_8b;
	uart_struct.USART_StopBits = USART_StopBits_1;
	uart_struct.USART_Parity = USART_Parity_No;
	uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Инициализируем UART
	USART_Init(USART1, &uart_struct);
	//Включаем UART
	USART_Cmd(USART1, ENABLE);
}

void send_to_uart(uint8_t data) {
	while (!(USART1->SR & USART_SR_TC))
		;
	USART1->DR = data;
}

void send_str(char *string, uint8_t flag) {
	uint8_t i = 0;
	while (string[i]) {
		send_to_uart(string[i]);
		i++;
	}
	if (flag) {
		send_to_uart('\r');
		send_to_uart('\n');
	}
}

void postformat(char *str, int len) {
	int pos = len;
	for (int i = 0; i < len; i++) {
		if (str[i] == 0) {
			pos = i;
			break;
		}
	}
	for (int i = pos; i < len; i++) {
		lcd_out(" ");
	}
}

void SysTick_Handler(void) {
	SysTick_CNT++;
}

void EXTI0_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line0); //Очищаем флаг
	GPIOB->ODR ^= GPIO_Pin_11; //Инвертируем состояние светодиода
	KMPH = (float) Kspd / (SysTick_CNT - lastFlash);
	lastFlash = SysTick_CNT;
}

void EXTI2_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line2);
	f = 1;
}

void EXTI3_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line3);
	f = 0;
}

void show(void) {
	switch (shownum) {
	case 0:
		//P
		adc_res = ADC1->JDR1;
		itoa(adc_res, strbuf, 10);
		lcd_set_xy(2, 0);
		lcd_out(strbuf);
		postformat(strbuf, 4);
		shownum++;
		break;
		//V
	case 1:
		intbuf = KMPH;
		itoa(intbuf, strbuf, 10);
		lcd_set_xy(9, 0);
		lcd_out(strbuf);
		postformat(strbuf, 3);
		shownum++;
		break;
	case 2:
		//f
		lcd_set_xy(19, 3);
		if (!f) {
			lcd_out("C");
		} else {
			lcd_out("W");
		}
		//M ----------- (2)
		intbuf = M;
		itoa(intbuf, strbuf, 10);
		lcd_set_xy(15, 0);
		lcd_out(strbuf);
		postformat(strbuf, 4);
		shownum++;
		break;
	case 3:
		//Mk
		// intbuf = round(avMKF);
		// itoa(intbuf, strbuf, 10);
		gcvt(avMKF, 12, strbuf);
		lcd_set_xy(3, 1);
		lcd_out(strbuf);
		postformat(strbuf, 5);
		shownum++;
		break;
	case 4:
		//K
		// intbuf = round(avKF);
		// itoa(intbuf, strbuf, 10);
		gcvt(avKF, 12, strbuf);
		lcd_set_xy(2, 2);
		lcd_out(strbuf);
		postformat(strbuf, 5);
		shownum++;
		break;
	case 5:
		//s
		lcd_set_xy(2, 3);
		switch (s) {
		case 0:
			lcd_out("0");
			break;
		case 1:
			lcd_out("1");
			break;
		case 2:
			lcd_out("2");
			break;
		default:
			lcd_out(" ");
			break;
		}
		shownum++;
		break;
	case 6:
		//g
		itoa(g, strbuf, 10);
		lcd_set_xy(6, 3);
		lcd_out(strbuf);
		postformat(strbuf, 4);
		shownum = 0;
		break;
	default:
		shownum = 0;
		break;
	}
}

void write_log(void) {
	//T
	send_str("T=", 0);
	itoa(SysTick_CNT, strbuf, 10);
	send_str(strbuf, 0);
	send_to_uart(';');
	//P
	send_str("P=", 0);
	adc_res = ADC1->JDR1;
	itoa(adc_res, strbuf, 10);
	send_str(strbuf, 0);
	send_to_uart(';');
	//V
	send_str("V=", 0);
	intbuf = round(KMPH);
	itoa(intbuf, strbuf, 10);
	send_str(strbuf, 0);
	send_to_uart(';');
	//M ----------- (2)
	send_str("M=", 0);
	intbuf = round(M);
	itoa(intbuf, strbuf, 10);
	send_str(strbuf, 0);
	send_to_uart(';');
	//f
	send_str("F=", 0);
	if (!f) {
		send_to_uart('0');
	} else {
		send_to_uart('1');
	}
	send_to_uart(';');
	//K
	send_str("K=", 0);
	// intbuf = round(k);
	// itoa(intbuf, strbuf, 10);
	gcvt(k, 12, strbuf);
	send_str(strbuf, 0);
	send_to_uart(';');
	//Mk
	send_str("Mk=", 0);
	// intbuf = round(mk);
	// itoa(intbuf, strbuf, 10);
	gcvt(mk, 12, strbuf);
	send_str(strbuf, 0);
	send_to_uart(';');
	//avKF
	send_str("avKF=", 0);
	// intbuf = round(avKF);
	// itoa(intbuf, strbuf, 10);
	gcvt(avKF, 12, strbuf);
	send_str(strbuf, 0);
	send_to_uart(';');
	//avMKF
	send_str("avMKF=", 0);
	// intbuf = round(avMKF);
	// itoa(intbuf, strbuf, 10);
	gcvt(avMKF, 12, strbuf);
	send_str(strbuf, 0);
	send_to_uart(';');
	//s
	send_str("S=", 0);
	switch (s) {
	case 0:
		send_to_uart('0');
		break;
	case 1:
		send_to_uart('1');
		break;
	case 2:
		send_to_uart('2');
		break;
	}
	send_to_uart(';');
	//g
	send_str("G=", 0);
	itoa(g, strbuf, 10);
	send_str(strbuf, 1);
}

float fsumPlusSqr(float *X, uint8_t sqr, int begin, int end) {
	float sum = 0.0;
	for (i=begin; i<end; i++){
		if (sqr) {
			sum += X[i]*X[i];
			}
		else {
			sum += X[i];
		}
	}
	return sum;
}

float SKO(float *X, int sizeX) {
	float result;
	result = sqrt((fsumPlusSqr(X,1,0,sizeX)/sizeX) - pow((fsumPlusSqr(X,0,0,sizeX)/sizeX),2));
	return result;
}

float SKOi(float *X, int sizeX, int pos) {
	float result;
	if (i==0){
		result = sqrt((fsumPlusSqr(X,1,1,sizeX)/(sizeX-1)) - pow(fsumPlusSqr(X,0,1,sizeX),2)/((sizeX-1)*(sizeX-1)));
	}
	else if (i==sizeX-1){
		result = sqrt((fsumPlusSqr(X,1,0,sizeX-1)/(sizeX-1)) - pow(fsumPlusSqr(X,0,0,sizeX-1),2)/((sizeX-1)*(sizeX-1)));
	}
	else {
		result = sqrt((fsumPlusSqr(X,1,0,pos)+fsumPlusSqr(X,1,pos+1,sizeX))/(sizeX-1) - pow((fsumPlusSqr(X,0,0,pos)+fsumPlusSqr(X,0,pos+1,sizeX)),2)/((sizeX-1)*(sizeX-1)));
	}
	return result;
}

float RErr(float *X, int sizeX) {
	float result;
	result = SKO(X, sizeX)/fsumPlusSqr(X,0,0,sizeX)*sizeX;
	return result;
}

int lenNoZero(float *X, int sizeX) {
	int i=0;
	while (X[i]) {
		if (i==sizeX) {
			break;
		}
		i++;
	}
	return i;
}

void filterKF(void){
	float err = 1.0;
	float K[L];
	uint8_t i, lenK=0;
	lenKF=L;
	for (i=0;i<L;i++) {
		KF[i]=KL[i];
	}
	while (err>0.05) {
		for (i=0;i<L;i++){
			K[i]=0.0;
		}
		lenK=0;
		for (i=0;i<lenKF;i++){
			if (SKOi(KF,lenKF,i)>SKO(KF,lenKF)) {
				K[lenK] = KF[i];
				lenK++;
			}
		}
		for (i=0;i<L;i++) {
			KF[i]=K[i];
			lenKF=lenK;
		}
		err = RErr(K,lenK);
		if (lenK == 0) {
			break;
		}
	}
}

void axleLoad(void) {
	uint32_t iterstart = 0;
	if ((f == 1) && (avKF == 0.0)) {
		lcd_set_xy(15, 3);
		lcd_out("ERR");
		if (SysTick_CNT - lastTime > 100000U) {
			lastTime = SysTick_CNT;
			show();
			write_log();
		}
		//*****************Ошибка: нужна калибровка
	} else {
		lcd_set_xy(15, 3);
		lcd_out("Wrk");
		for (i = 0; i < L; i++){
			KL[i]=0.0;
		}
		i = 0;
		while (i < L) { // (2)
			iterstart = SysTick_CNT;
			//init---------------------------------
			avPfree = 0.0;
			avP = 0.0;
			avAfree = 0.0;
			avA = 0.0;
			g = 0;
			s = 0;
			for (j = 0; j < J; j++) {
				p[j] = 0;
				v[j] = 0.0;
				t[j] = 0;
				a[j] = 0.0;
			}
			//-------------------------------------
			j = 0;
			while (j < J) {//сбор данных
				if ((j==0) || ((j!=0) && ((SysTick_CNT - t[j - 1]) >= delta_t))) {
					t[j] = SysTick_CNT; //SysTick_CNT
					p[j] = ADC1->JDR1; //ADC1->JDR1
					if ((j == 0) && (p[j] > p0)) {
						s = 1;
						break;
					}
					v[j] = KMPH;
					if (j != 0)
						a[j] = (float)(v[j] - v[j - 1])/(t[j] - t[j - 1]); //
					if ((p[j] <= p0) && (g == 0)) {
						// avPfree = (float)(avPfree * j + p[j]) / (j + 1);
						avAfree = (float)(avAfree * j + a[j]) / (j + 1);
					}
					if (p[j] > p0) {
						avP = (float)(avP * g + p[j]) / (g + 1);
						avA = (float)(avA * g + a[j]) / (g + 1);
						g++;
						if (g == (J - 1)) {//зависает при g=999 !!! (2)
							break;
						}
					}
					if (((p[j] <= p0) || (v[j] <= v0)) && (g > 0)) {//оконч. торм. при остановке (2)
						if (g < b) {
							s = 2;
							break;
						} else {
							break;
						}
					}
					j++;
					if (SysTick_CNT - lastTime > 100000) {
						lastTime = SysTick_CNT;
						show();
					}
					write_log();
				}
			}
			if ((s == 0) && (g > 0)) {
				kbuf = (float)((avA - avAfree) / avP); // (2)
				if ((kbuf > -1.0e-8) && (kbuf < -0.1e-9)) { // (2)
					if (f) {
						mk = kbuf;
						KL[i] = kbuf;
					}
					else {
						k = kbuf;
						KL[i] = kbuf;
					}
					i++; // (2)
				}
			}
			itertime = SysTick_CNT - iterstart;
		}
	}
	//-----------filter...
	filterKF();
	if (f) {
		avMKF = fsumPlusSqr(KF,0,0,lenKF)/lenKF;
		if (avKF != 0) {
			M = (float)(avMKF/avKF)*m0;//(2)
		}
	}
	else {
		avKF = fsumPlusSqr(KF,0,0,lenKF)/lenKF;
		//f=1;
	}
}

int main(void) {
	initialization();
	lcd_set_xy(0, 0);
	lcd_out("P=     V=    M=");//(2)
	lcd_set_xy(0, 1);
	lcd_out("Mk=");
	lcd_set_xy(0, 2);
	lcd_out("K=");
	lcd_set_xy(0, 3);
	lcd_out("s=  g=");
	// send_str("T;P;V;f;Mk;K;s;g", 1);
	while (1) {
		if ((SysTick_CNT - lastFlash) > sec) { //если сигнала нет больше секунды
			KMPH = 0;
		}
		axleLoad();
	}
}
