/******************************************************************************
;  *   	@MCU   	   	   	   	 : MC32P7031
;  *   	@Create Date         : 2019.12.29
;  *   	@Author/Corporation  : Martin/SinoMCU
;  *   	@技术支持QQ群   	   	  : 814641858      
;  *    @晟矽微官方论坛   	   	  : http://bbs.sinomcu.com/	 
;  *   	@Copyright           : 2019 SINOMCU Corporation,All Rights Reserved.
;  *----------------------Abstract Description---------------------------------
;  *   	   	   	P54 1ms翻转  P53输出PWM  P00唤醒 P40.P41.VDDAD采集

   	   	   	   	注意 7031不能直接读IO某一位
   	   	   	   	   	如 if(P54D==x)
   	   	   	   	   	   	P54D=!P54D;
   	   	   	   	   	改为 将整组IO读出到一个变量
   	   	   	   	   	   	buff=IOP5;
   	   	   	   	   	   	在判断buff某一位

"T0定时器1mS中断P54翻转  
T1输出PWM（P53）
T0定时5S进入休眠，P00下降沿唤醒
P40.P41.VDD AD采集"

******************************************************************************/

#include "MC32P7031.h"
#include "user.h"

/************************************************
;  *    @Function Name       : CLR_RAM
;  *    @Description         : 上电清RAM
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
;  *    @Description         : IO初始化
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void IO_Init(void)
{
   	IOP0 = 0x00; //io口数据位
   	OEP0 = 0xfc; //io口方向 1:out  0:in
   	PUP0 = 0x02; //io口上拉电阻   1:enable  0:disable

   	IOP4 = 0x00; //io口数据位
   	OEP4 = 0xef;  //io口方向
   	PUP4 = 0x00;  //io口上拉电阻   1:enable  0:disable
   	ANSEL = 0x10; //io类型选择  1:模拟输入  0:通用io

   	IOP5 = 0x00; //io口数据位
   	OEP5 = 0xfb; //io口方向 1:out  0:in
   	PUP5 = 0x00; //io口上拉电阻   1:enable  0:disable
}

/************************************************
;  *    @Function Name       : TIMER0_INT_Init
;  *    @Description         : 
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void TIMER2_INT_Init(void)
{
   	T2CR = 0xD4; //内部 8分频  允许自动重载
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
   	//T0时钟 T0 T2 Fcpu  T1 Fhosc 开启T0唤醒
   	TXCR = 0x0a;

   	// 开启TIMER0, 使能PWM0
   	T0CR = 0xE0;
   	T0C = 0;
   	T0D = 200;

   	PWM0OE=0;
   	TC0EN=0;
}

/************************************************
;  *    @Function Name       : ADC_Init
;  *    @Description         : ADC初始化
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void ADC_Init(void)
{
   	// ANSEL |= 0x03; //P40 1  模拟输入  1 模拟输入  0  IO口
   	ADCR &= 0x00;
   	ADCR |= 0x90;
   	VREF = 0x02; //设置内部参考电压 为4.0V
   	ADRL &= 0x80;
   	ADRL |= 0x20; //adc时钟  Fcpu/16
}

/************************************************
;  *    @Function Name       : Sys_Init
;  *    @Description         : 系统初始化
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
;  *    @Description         : 连续转换
;  *    @IN_Parameter      	 : 通道
;  *    @Return parameter    : 返回ADC的值
;  ***********************************************/
uint ADC_Get_Value_Average(uchar CHX)
{
   	u8 channel = 0;
   	unsigned long int tmpBuff = 0;
   	u16 ADCMAX = 0;
   	u16 ADCMIN = 0xffff;

   	ADCR = (ADCR & 0xf8) | CHX; // ADC 使能  AD转换通道开启  通道  CHX
   	for (channel = 0; channel < 20; channel++)
   	{
   	   	ADEOC = 0; 
   	   	ADST = 1; // 开始转换
   	   	while (!ADEOC)
   	   	   	; //等待转换完成
   	   	adctmp = ADRH;
   	   	adctmp = adctmp << 4 | (ADRL & 0x0f);
   	   	if (channel < 2)
   	   	   	continue; //丢弃前两次采样的
   	   	if (adctmp > ADCMAX)
   	   	   	ADCMAX = adctmp; //最大
   	   	if (adctmp < ADCMIN)
   	   	   	ADCMIN = adctmp; //最小
   	   	tmpBuff += adctmp;
   	}

   	tmpBuff -= ADCMAX; 	   	   	   	 //去掉一个最大
   	tmpBuff -= ADCMIN; 	   	   	   	 //去掉一个最小
   	adctmp = (tmpBuff >> 4); //除以16，取平均值
   	return adctmp;
}

// 控制pwm0输出
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
		
		// 关闭pwm输出
		// pwm_on_by_gear(0);
   	}
}

void delay_ms(u16 ms)
{
   	delay_count = 0;
   	while(delay_count < ms);
}

// 控制指定LED灯开关，1 - 6
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

// 是否正在充电
u8 fascia_gun_is_charging(void)
{
   	IO_buff.byte = IOP0;
   	return IO_BIT0;
}

// 控制指定个数LED灯亮 0 - 6
void led_on_by_gear(u8 n) 
{
   	for (int i=1; i<7; i++) {
   	   	led_on(i, 0);
   	}

   	for (int i=1; i<=n; i++) {
   	   	led_on(i, 1);
   	}
}

// 根据电量百分比显示 LED
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

// 关机
void fascia_gun_off(void)
{
   	gear = 0;
   	FLAG_LOW_BAT = 0;
   	// led全关
   	led_on_by_gear(0);
   	// 电机关闭
   	pwm_on_by_gear(0);
}

void main(void)
{
   	Sys_Init();

 	fascia_gun_off();

   	while (1)
   	{  	   	   
   	   	// 充电状态, 检测电池电量, 根据电量显示跑马灯
   	   	if(fascia_gun_is_charging()) {
   	   	   	// 进入充电状态, 检测一次电压
   	   	   	if (FLAG_CHARGING == 0) {
   	   	   	   	FLAG_CHARGING = 1;
				FLAG_KEY_SHORT = 0;
   	   	   	   	measure_bat_level();
   	   	   	   	bat_adc_count = 0;
   	   	   	}

   	   	   	// 如果充电时已开机, 关闭led和pwm
   	   	   	if (gear > 0) {
				fascia_gun_off();
   	   	   	}
			
			// 判断是否充满, 没有充满10分钟检测一次电量
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

   	   	   	// 根据电池电量显示LED
   	   	   	led_on_by_bat(bat_level);

   	   	// 非充电状态, 检测按键， 开关机或调节档位
   	   	} else {
   	   	   	// 充电器拔掉时, 执行一次关机操作
   	   	   	if (FLAG_CHARGING) {
   	   	   	   	fascia_gun_off();
   	   	   	   	FLAG_CHARGING = 0;
   	   	   	}

   	   	   	// 检测p01按键
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

   	   	   	// 按键长按
   	   	   	if (FLAG_KEY_LONG) {
   	   	   	   	FLAG_KEY_LONG = 0;
   	   	   	   	// 关机
   	   	   	   	fascia_gun_off();
   	   	   	}

   	   	   	// 按键短按
   	   	   	if (FLAG_KEY_SHORT) {
   	   	   	   	FLAG_KEY_SHORT = 0;

				if (gear_count > 200) {						
					if (gear == 0) {
						measure_bat_level();
					}

   	   	   	   		gear++;
   	   	   	   		gear_count = 0;

   	   	   	   		// 电池没有低电量，调节档位
   	   	   	   		if (bat_level > 0) {
   	   	   	   		   	// 大于6档关机
   	   	   	   		   	if (gear > 6) {
   	   	   	   		   	   	fascia_gun_off();
   	   	   	   		   	// 调节档位
   	   	   	   		   	} else {
   	   	   	   		   	   	// 	根据档位显示LED
   	   	   	   		   	   	led_on_by_gear(gear);
   	   	   	   		   	   	// 	根据档位调节电机
   	   	   	   		   	   	pwm_on_by_gear(gear);
   	   	   	   		   	}
   	   	   	   		} else {
   	   	   	   		   	if (gear > 6) {
   	   	   	   		   	   	gear = 6;
   	   	   	   		   	}
   	   	   	   		}
   	   	   		}
			}

   	   	   	// 处于开机状态的持续性检测工作  	
   	   	   	if (gear > 0) {
   	   	   	   	// 工作时每隔1分钟检测一次电量
   	   	   	   	if (bat_adc_count > 10 * 1000ul) {
   	   	   	   	   	measure_bat_level();
   	   	   	   	   	bat_adc_count = 0;
   	   	   	   	}

   	   	   	   	// 一个档位持续超过15分钟关机
   	   	   	   	if (gear_count > 15 * 60 * 1000ul) {
   	   	   	   	   	fascia_gun_off();
   	   	   	   	}

   	   	   	   	// 低电量状态6个灯一起闪, 直到关机操作出现
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
				// 没有按键按下, 没有充电器连接，休眠
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
