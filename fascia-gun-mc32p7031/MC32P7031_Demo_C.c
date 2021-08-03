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
   	IOP0 = 0x00; //io������λ
   	OEP0 = 0xfc; //io�ڷ��� 1:out  0:in
   	PUP0 = 0x02; //io����������   1:enable  0:disable

   	IOP4 = 0x00; //io������λ
   	OEP4 = 0xef;  //io�ڷ���
   	PUP4 = 0x00;  //io����������   1:enable  0:disable
   	ANSEL = 0x10; //io����ѡ��  1:ģ������  0:ͨ��io

   	IOP5 = 0x00; //io������λ
   	OEP5 = 0xfb; //io�ڷ��� 1:out  0:in
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
   	//T0ʱ�� T0 T2 Fcpu  T1 Fhosc ����T0����
   	TXCR = 0x0a;

   	// ����TIMER0, ʹ��PWM0
   	T0CR = 0xE0;
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

// adc = v * 210
// 0 < 7v    	
// 1 > 7v    	1470
// 2 > 7.5v    	1575
// 3 > 7.8v    	1638
// 4 > 8v  	   	1680
// 5 > 8.2v    	1722   	   	   	 
void measure_bat_level(void)
{
   	ADC_Get_Value_Average(4);

   	FLAG_LOW_BAT = 0;

   	if (adctmp > 1722) {
   	   	bat_level = 5;
   	} else if (adctmp > 1680) {
   	   	bat_level = 4;
   	} else if (adctmp > 1638) {
   	   	bat_level = 3;
   	} else if (adctmp > 1575) {
   	   	bat_level = 2;
   	} else if (adctmp > 1470) {
   	   	bat_level = 1;
   	} else {
   	   	FLAG_LOW_BAT = 1;
   	   	bat_level = 0;
		
		// �ر�pwm���
		// pwm_on_by_gear(0);
   	}
}

void delay_ms(u16 ms)
{
   	delay_count = 0;
   	while(delay_count < ms);
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
   	   	IO_buff.byte = IOP4;
   	   	IO_BIT3 = onoff;
   	   	IOP4 = IO_buff.byte;
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

// �Ƿ����ڳ��
u8 fascia_gun_is_charging(void)
{
   	IO_buff.byte = IOP0;
   	return IO_BIT0;
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

	for (int i=0; i<5; i++) {
   		IO_buff.byte = IOP0;
   		if (IO_BIT0 == 0) {
		  	 return;
		}
   		delay_ms(100);
	}

   	for (int i=(n+1); i<7; i++) {
   	   	led_on(i, 1);

		for (int i=1; i<5; i++) {
   			IO_buff.byte = IOP0;
   			if (IO_BIT0 == 0) {
			   return;
			}

   	   		delay_ms(100);
		}
   	}
}

void system_sleep(void)
{
	GIE = 0;
	ADON = 0;

	Nop();
	Nop();
	OSCM &= 0xE7;
	OSCM |= 0x08;
	Nop();
	Nop();
	Nop(); 	   	   	

	GIE = 1;
	ADON = 1;


   	IO_buff.byte = IOP0;
   	if (IO_BIT1 == 0) {
		FLAG_KEY_SHORT = 1;
		gear_count = 201;
	}
}

// �ػ�
void fascia_gun_off(void)
{
   	gear = 0;
   	FLAG_LOW_BAT = 0;
   	// ledȫ��
   	led_on_by_gear(0);
   	// ����ر�
   	pwm_on_by_gear(0);
}

void main(void)
{
   	Sys_Init();

 	fascia_gun_off();

   	while (1)
   	{  	   	   
   	   	// ���״̬, ����ص���, ���ݵ�����ʾ�����
   	   	if(fascia_gun_is_charging()) {
   	   	   	// ������״̬, ���һ�ε�ѹ
   	   	   	if (FLAG_CHARGING == 0) {
   	   	   	   	FLAG_CHARGING = 1;
				FLAG_KEY_SHORT = 0;
   	   	   	   	measure_bat_level();
   	   	   	   	bat_adc_count = 0;
   	   	   	}

   	   	   	// ������ʱ�ѿ���, �ر�led��pwm
   	   	   	if (gear > 0) {
				fascia_gun_off();
   	   	   	}
			
			// �ж��Ƿ����, û�г���10���Ӽ��һ�ε���
   			IO_buff.byte = IOP5;
   			if (IO_BIT2 == 0) {
   	   	   	   	bat_adc_count = 0;
				bat_level = 6;
			} else {
   	   	   		if (bat_adc_count > 10 * 60 * 1000ul) {
   	   	   	   		measure_bat_level();
   	   	   	   		bat_adc_count = 0;
				}
			}

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
   	   	   	   	   	if (key_press_count == 0) {
   	   	   	   	   	   	key_press_count = 1;
   	   	   	   	   	} else {
   	   	   	   	   	   	if (key_press_count > 2000) {
   	   	   	   	   	   	   	FLAG_KEY_LONG = 1;
   	   	   	   	   	   	   	FLAG_KEY_LONG_DONE = 1;
   	   	   	   	   	   	   	key_press_count = 0;
   	   	   	   	   	   	}
   	   	   	   	   	}
   	   	   	   	} else {
   	   	   	   	   	if (key_press_count > 100 && FLAG_KEY_LONG_DONE == 0) {
   	   	   	   	   	   	FLAG_KEY_SHORT = 1;
   	   	   	   	   	}

   	   	   	   	   	key_press_count = 0;
   	   	   	   	   	FLAG_KEY_LONG_DONE = 0;
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

				if (gear_count > 200) {						
					if (gear == 0) {
						measure_bat_level();
					}

   	   	   	   		gear++;
   	   	   	   		gear_count = 0;

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
			}

   	   	   	// ���ڿ���״̬�ĳ����Լ�⹤��  	
   	   	   	if (gear > 0) {
   	   	   	   	// ����ʱÿ��1���Ӽ��һ�ε���
   	   	   	   	if (bat_adc_count > 10 * 1000ul) {
   	   	   	   	   	measure_bat_level();
   	   	   	   	   	bat_adc_count = 0;
   	   	   	   	}

   	   	   	   	// һ����λ��������15���ӹػ�
   	   	   	   	if (gear_count > 15 * 60 * 1000ul) {
   	   	   	   	   	fascia_gun_off();
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
   	   	   	} else {
				delay_ms(200);
   	   	   	   	IO_buff.byte = IOP0;
				// û�а�������, û�г�������ӣ�����
   	   	   	   	if (IO_BIT1 == 1 && IO_BIT0 == 0) {
					system_sleep();
				}
			}
   	   	}
   	}
}

void int_isr(void) __interrupt
{
   	__asm 
   	push
   	__endasm;
   	//=======T0========================
   	if(T0IF&&T0IE)
   	{
   	   	T0IF=0;
   	}
   	
   	//=======T2========================
   	if (T2IF && T2IE)
   	{
   	   	T2IF = 0;
   	   	gear_count++;
   	   	delay_count++;
		bat_adc_count++;
   	   	if (key_press_count > 0) {
   	   	   	key_press_count++;
   	   	}
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
