/******************************************************************************
;  *   	@MCU   	   	   	   	 : MC32P7031
;  *   	@Create Date         : 2019.12.29
;  *   	@Author/Corporation  : Martin/SinoMCU
;  *   	@����֧��QQȺ   	   	  : 814641858      
;  *    @����΢�ٷ���̳   	   	  : http://bbs.sinomcu.com/	 
;  *   	@Copyright           : 2019 SINOMCU Corporation,All Rights Reserved.
;  *----------------------Abstract Description---------------------------------
;  *   	   	   	P54 1ms��ת  P53���PWM  P00���� P40.P41.VDDAD�ɼ�

   	   	   	   	ע�� 7031����ֱ�Ӷ�IOĳһλ
   	   	   	   	   	�� if(P54D==x)
   	   	   	   	   	   	P54D=!P54D;
   	   	   	   	   	��Ϊ ������IO������һ������
   	   	   	   	   	   	buff=IOP5;
   	   	   	   	   	   	���ж�buffĳһλ

"T0��ʱ��1mS�ж�P54��ת  
T1���PWM��P53��
T0��ʱ5S�������ߣ�P00�½��ػ���
P40.P41.VDD AD�ɼ�"

******************************************************************************/

#include "MC32P7031.h"
#include "user.h"

/************************************************
;  *    @Function Name       : CLR_RAM
;  *    @Description         : �ϵ���RAM
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void CLR_RAM(void)
{
   	__asm 
   	movai 0x7F 
   	movra FSR0
   	clrr FSR1
   	clrr INDF
   	DJZR FSR0 
   	goto $ -2 
   	clrr INDF
   	__endasm;
}

/************************************************
;  *    @Function Name       : IO_Init
;  *    @Description         : IO��ʼ��
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void IO_Init(void)
{
   	// P0 - p00:key, p01:led3, p02:led1, p03:-
   	// P00 	key
   	// P01 	��
   	// P02 	led1
   	// P03 	��
   	IOP0 = 0x00; //io������λ
   	OEP0 = 0xfD; //io�ڷ��� 1:out  0:in
   	PUP0 = 0x02; //io����������   1:enable  0:disable

   	// P4
   	// P40 led6
   	// P41 led5
   	// P42 led4
   	// P43 adc ���
   	// P44 adc ��ص�ѹ
   	IOP4 = 0x00; //io������λ
   	OEP4 = 0xe7;  //io�ڷ���
   	PUP4 = 0x00;  //io����������   1:enable  0:disable
   	ANSEL = 0x10; //io����ѡ��  1:ģ������  0:ͨ��io

   	// P5
   	// P52 ��
   	// P53 led3
   	// P54 PWM
   	IOP5 = 0x00; //io������λ
   	OEP5 = 0xff; //io�ڷ��� 1:out  0:in
   	PUP5 = 0x00; //io����������   1:enable  0:disable
}

/************************************************
;  *    @Function Name       : TIMER0_INT_Init
;  *    @Description         : 
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void TIMER2_INT_Init(void)
{
   	TXCR = 0x08; //T0ʱ�� T0 T2 Fcpu  T1 Fhosc ��ֹT0����

   	T2CR = 0xD4; //�ڲ� 8��Ƶ  �����Զ�����
   	T2C = 256 - 125;
   	T2D = 256 - 125; //1ms
   	T2IF = 0;
   	T2IE = 1;
}

/************************************************
;  *    @Function Name       : TIMER1_PWM_Init
;  *    @Description         : 
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void TIMER0_PWM_Init(void)
{
   	// ����TIMER2, ʹ��PWM2
   	T0CR = 0xF1;
   	T0C = 0;
   	T0D = 200;

   	PWM0OE=0;
   	TC0EN=0;
}

/************************************************
;  *    @Function Name       : ADC_Init
;  *    @Description         : ADC��ʼ��
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void ADC_Init(void)
{
   	// ANSEL |= 0x03; //P40 1  ģ������  1 ģ������  0  IO��
   	ADCR &= 0x00;
   	ADCR |= 0x90;
   	VREF = 0x02; //�����ڲ��ο���ѹ Ϊ4.0V
   	ADRL &= 0x80;
   	ADRL |= 0x20; //adcʱ��  Fcpu/16
}

/************************************************
;  *    @Function Name       : Sys_Init
;  *    @Description         : ϵͳ��ʼ��
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void Sys_Init(void)
{
   	GIE = 0;
   	CLR_RAM();
   	IO_Init();
   	TIMER2_INT_Init();
   	TIMER0_PWM_Init();
   	ADC_Init();
   	GIE = 1;
}

/************************************************
;  *    @Function Name       : ADC_Get_Value
;  *    @Description         : ADC����ת��
;  *    @IN_Parameter      	 : CHX  ADCͨ��
;  *    @Return parameter    : ��ͨ��ADC��ֵ
;  ***********************************************/
//uint ADC_Get_Value(uchar CHX)
//{
//     	u16 ADC_temp_value;
//
//     	ADCR = (ADCR & 0xf8) | CHX; // ADC ʹ��  ADת��ͨ������  ͨ��  CHX
//
//     	if (changl_Num != CHX) //ͨ���л���  ��ȥ
//     	{
//     	   	changl_Num = CHX;
//     	   	ADEOC = 0;
//     	   	ADST = 1; // ��ʼת��
//     	   	while (ADEOC == 0)
//     	   	{ // ���EOC�ı�־λ   �ȴ�ת�����
//     	   	}
//     	}
//     	ADEOC = 0;
//     	ADST = 1; // ��ʼת��
//     	while (ADEOC == 0)
//     	{ // ���EOC�ı�־λ   �ȴ�ת�����
//     	}
//     	ADC_temp_value = ADRH;
//     	ADC_temp_value = ADC_temp_value << 4 | (ADRL & 0x0f);
//
//     	return ADC_temp_value; // ���ADCת��������
//}

u16 adctmp;
/************************************************
;  *    @Function Name       : ADC_Get_Value_Average
;  *    @Description         : ����ת��
;  *    @IN_Parameter      	 : ͨ��
;  *    @Return parameter    : ����ADC��ֵ
;  ***********************************************/
uint ADC_Get_Value_Average(uchar CHX)
{
   	u8 channel = 0;
   	unsigned long int tmpBuff = 0;
   	u16 ADCMAX = 0;
   	u16 ADCMIN = 0xffff;

   	ADCR = (ADCR & 0xf8) | CHX; // ADC ʹ��  ADת��ͨ������  ͨ��  CHX
   	for (channel = 0; channel < 20; channel++)
   	{
   	   	ADEOC = 0; 
   	   	ADST = 1; // ��ʼת��
   	   	while (!ADEOC)
   	   	   	; //�ȴ�ת�����
   	   	adctmp = ADRH;
   	   	adctmp = adctmp << 4 | (ADRL & 0x0f);
   	   	if (channel < 2)
   	   	   	continue; //����ǰ���β�����
   	   	if (adctmp > ADCMAX)
   	   	   	ADCMAX = adctmp; //���
   	   	if (adctmp < ADCMIN)
   	   	   	ADCMIN = adctmp; //��С
   	   	tmpBuff += adctmp;
   	}

   	tmpBuff -= ADCMAX; 	   	   	   	 //ȥ��һ�����
   	tmpBuff -= ADCMIN; 	   	   	   	 //ȥ��һ����С
   	adctmp = (tmpBuff >> 4); //����16��ȡƽ��ֵ
   	return adctmp;
}

// adc = v * 210
// 0 0% 		6v 		1260
// 1 15%		6.2v	1302
// 2 30%		6.6v	1386
// 3 45%		7v		1470
// 4 60%		7.4v	1554
// 5 75%		8v		1680
// 6 100% 		8.2v	1722			 

u8 measure_bat_level(void)
{
   	ADC_Get_Value_Average(4);

	if (adctmp > 1722) {
		return 6;
	} else if (adctmp > 1680) {
		return 5;
	} else if (adctmp > 1554) {
		return 4;
	} else if (adctmp > 1470) {
		return 3;
	} else if (adctmp > 1386) {
		return 2;
	} else if (adctmp > 1320) {
		return 1;
	}

	FLAG_LOW_BAT = 1;
   	return 0;
}

void delay_ms(u16 ms)
{
   	uint32_t start = timer0_count;
   	while(timer0_count - start < ms);
}

// ����pwm0���
void pwm_on_by_gear(u8 gear)
{
   	PWM0OE=0;
   	TC0EN=0;

   	if (gear > 0) {
   		T0D = 190 + gear * 10;
   		PWM0OE=1;
   		TC0EN=1;
   	}
}

// ����ָ��LED�ƿ��أ�1 - 6
void led_on(u8 n, u8 onoff)
{
    if (n == 1) {
   	   	IO_buff.byte = IOP0;
   	   	IO_BIT2 = onoff;
   	   	IOP0 = IO_buff.byte;
    } else if (n == 2) {
   	   	IO_buff.byte = IOP5;
   	   	IO_BIT3 = onoff;
   	   	IOP5 = IO_buff.byte;
    } else if (n == 3) {
   	   	IO_buff.byte = IOP0;
   	   	IO_BIT0 = onoff;
   	   	IOP0 = IO_buff.byte;
    } else if (n == 4) {
   	   	IO_buff.byte = IOP4;
   	   	IO_BIT2 = onoff;
   	   	IOP4 = IO_buff.byte;
    } else if (n == 5) {
   	   	IO_buff.byte = IOP4;
   	   	IO_BIT1 = onoff;
   	   	IOP4 = IO_buff.byte;
    } else if (n == 6) {
   	   	IO_buff.byte = IOP4;
   	   	IO_BIT0 = onoff;
   	   	IOP4 = IO_buff.byte;
    }
}

// ����ָ������LED���� 0 - 6
void led_on_by_gear(u8 n) 
{
   	for (int i=1; i<7; i++) {
   	   	led_on(i, 0);
   	}

   	for (int i=1; i<=n; i++) {
   	   	led_on(i, 1);
   	}
}

// ���ݵ����ٷֱ���ʾ LED
// 0 < 15%
// 1 < 30%
// 2 < 45%
// 3 < 60%
// 4 < 75%
// 5 < 100
// 6 = 100%
void led_on_by_bat(u8 n)
{
   	for (int i=(n+1); i<7; i++) {
   	   	led_on(i, 0);
   	}

   	for (int i=1; i<=n; i++) {
   	   	led_on(i, 1);
   	}

   	delay_ms(1000);

   	for (int i=(n+1); i<7; i++) {
   	   	led_on(i, 1);
   	   	delay_ms(1000);
   	}
}

// �Ƿ����ڳ��
u8 fascia_gun_is_charging(void)
{
   	IO_buff.byte = IOP4;
   	return IO_BIT3;
}

// ����
void fascia_gun_on(void)
{
   	// ��ʱ����0
   	timer0_count = 0;
	last_switch_gear_tick = 0;
	last_adc_tick = 0;
}

// �ػ�
void fascia_gun_off(void)
{
	FLAG_LOW_BAT = 0;
	// ��ʱ������
   	timer0_count = 0;
   	// ��λ����
   	gear = 0;
   	// ledȫ��
   	led_on_by_gear(0);
   	// ����ر�
	pwm_on_by_gear(0);
}

void main(void)
{
   	Sys_Init();
   	while (1)
   	{  	   	   
   	   	// ���״̬, ����ص���, ���ݵ�����ʾ�����
   	   	if(fascia_gun_is_charging()) {
   	   	   	FLAG_CHARGING = 1;

   	   	   	// ������ʱ�ѿ���, ����Ĥǹ�ػ�
   	   	   	if (gear > 0) {
   	   	   	   	fascia_gun_off();
   	   	   	}

			// �������ȼ�
			bat_level = measure_bat_level();

   	   	   	// ���ݵ�ص�����ʾLED
   	   	   	led_on_by_bat(bat_level);

   	   	// �ǳ��״̬, ��ⰴ���� ���ػ�����ڵ�λ
   	   	} else {
   	   	   	// ������ε�ʱ, ִ��һ�ιػ�����
   	   	   	if (FLAG_CHARGING) {
   	   	   	   	fascia_gun_off();
   	   	   	   	FLAG_CHARGING = 0;
   	   	   	}

   	   	   	// ���p01����
   	   	   	{
   	   	   	   	IO_buff.byte = IOP0;
   	   	   	   	if (IO_BIT1 == 0) {
   	   	   	   	   	if (key_down_count == 0) {
   	   	   	   	   	   	key_down_count = timer0_count;
   	   	   	   	   	} else {
   	   	   	   	   	   	key_press_count = timer0_count - key_down_count;

   	   	   	   	   	   	if (key_press_count > 2000) {
   	   	   	   	   	   	   	FLAG_KEY_LONG = 1;
   	   	   	   	   	   	}
   	   	   	   	   	}
   	   	   	   	} else {
   	   	   	   	   	if (key_down_count > 0) {
   	   	   	   	   	   	key_press_count = timer0_count - key_down_count;
   	   	   	   	   	   	key_down_count = 0;	

   	   	   	   	   	   	if (key_press_count > 100 && key_press_count < 2000) {
   	   	   	   	   	   	   	FLAG_KEY_SHORT = 1;
   	   	   	   	   	   	}
   	   	   	   	   	}
   	   	   	   	}
   	   	   	}

   	   	   	// ��������
   	   	   	if (FLAG_KEY_LONG) {
   	   	   	   	FLAG_KEY_LONG = 0;
   	   	   	   	// �ػ�
   	   	   	   	fascia_gun_off();
   	   	   	}

   	   	   	// �����̰�
   	   	   	if (FLAG_KEY_SHORT) {
   	   	   	   	FLAG_KEY_SHORT = 0;

   	   	   	   	// ����
   	   	   	   	if (gear == 0) {
   	   	   	   	   	fascia_gun_on();
					bat_level = measure_bat_level();
   	   	   	   	}

   	   	   	   	gear++;
				last_switch_gear_tick = timer0_count;

				// ���û�е͵��������ڵ�λ
				if (bat_level > 0) {
   	   	   	   		// ����6���ػ�
   	   	   	   		if (gear > 6) {
   	   	   	   		   	fascia_gun_off();
   	   	   	   	
   	   	   	   		// ���ڵ�λ
   	   	   	   		} else {
   	   	   	   		   	// 	���ݵ�λ��ʾLED
   	   	   	   		   	led_on_by_gear(gear);
   	   	   	   		   	// 	���ݵ�λ���ڵ��
   	   	   	   		   	pwm_on_by_gear(gear);
   	   	   	   		}
				} else {
					if (gear > 6) {
						gear = 6;
					}
				}
   	   	   	}

			// ���ڿ���״̬�ĳ����Լ�⹤��	
			if (gear > 0) {
				// ����ʱÿ��10����һ�ε���
				if (timer0_count - last_adc_tick > 10 * 1000ul) {
					bat_level = measure_bat_level();
				}

				// һ����λ��������15���ӹػ�
				if (timer0_count - last_switch_gear_tick > 15 * 60 * 1000ul) {
					fascia_gun_off();
				}
			}

			// �͵���״̬6����һ����, ֱ���ػ���������
			if (FLAG_LOW_BAT) {
				for (int i=1; i<7; i++) {
					led_on(i, 1);
				}
				delay_ms(1000);
				for (int i=1; i<7; i++) {
					led_on(i, 0);
				}
				delay_ms(1000);
			}
   	   	}
   	}
}

void int_isr(void) __interrupt
{
   	__asm 
   	push
   	__endasm;
   	//=======T2========================
   	if (T2IF && T2IE)
   	{
   	   	T2IF = 0;
   	   	timer0_count++;
   	}

   	//=======ADC=======================
   	if (ADIF && ADIE)
   	{
   	   	ADIF = 0;
   	}
   	
   	__asm 
   	pop
   	__endasm;
}

