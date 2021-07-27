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
   	// P0 - p00:key, p01:led3, p02:led1, p03:-
   	// P00 	key
   	// P01 	无
   	// P02 	led1
   	// P03 	无
   	IOP0 = 0x00; //io口数据位
   	OEP0 = 0xfD; //io口方向 1:out  0:in
   	PUP0 = 0x02; //io口上拉电阻   1:enable  0:disable

   	// P4
   	// P40 led6
   	// P41 led5
   	// P42 led4
   	// P43 adc 充电
   	// P44 adc 电池电压
   	IOP4 = 0x00; //io口数据位
   	OEP4 = 0xe7;  //io口方向
   	PUP4 = 0x00;  //io口上拉电阻   1:enable  0:disable
   	ANSEL = 0x00; //io类型选择  1:模拟输入  0:通用io

   	// P5
   	// P52 无
   	// P53 led3
   	// P54 PWM
   	IOP5 = 0x00; //io口数据位
   	OEP5 = 0xff; //io口方向 1:out  0:in
   	PUP5 = 0x00; //io口上拉电阻   1:enable  0:disable
}

/************************************************
;  *    @Function Name       : TIMER0_INT_Init
;  *    @Description         : 
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void TIMER0_INT_Init(void)
{
   	TXCR = 0x08; //T0时钟 T0 T2 Fcpu  T1 Fhosc 禁止T0唤醒

   	T0CR = 0xD4; //内部 8分频  允许自动重载
   	T0C = 256 - 125;
   	T0D = 256 - 125; //1ms
   	T0IF = 0;
   	T0IE = 1;
}

/************************************************
;  *    @Function Name       : TIMER1_PWM_Init
;  *    @Description         : 
;  *    @IN_Parameter        :
;  *    @Return parameter    :
;  ***********************************************/
void TIMER1_PWM_Init(void)
{
   	T1CR = 0xF1;
   	T1C = 0;
   	T1D = 127;
}

/************************************************
;  *    @Function Name       : ADC_Init
;  *    @Description         : ADC初始化
;  *    @IN_Parameter      	 : 
;  *    @Return parameter    :
;  ***********************************************/
void ADC_Init(void)
{
   	ANSEL |= 0x03; //P40 1  模拟输入  1 模拟输入  0  IO口
   	ADCR &= 0x00;
   	ADCR |= 0x90;
   	VREF = 0x00; //设置内部参考电压 为2.0V
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
   	TIMER0_INT_Init();
   	//TIMER1_PWM_Init();
   	//ADC_Init();
   	GIE = 1;
}

/************************************************
;  *    @Function Name       : ADC_Get_Value
;  *    @Description         : ADC单次转换
;  *    @IN_Parameter      	 : CHX  ADC通道
;  *    @Return parameter    : 该通道ADC的值
;  ***********************************************/
uint ADC_Get_Value(uchar CHX)
{
   	u16 ADC_temp_value;

   	ADCR = (ADCR & 0xf8) | CHX; // ADC 使能  AD转换通道开启  通道  CHX

   	if (changl_Num != CHX) //通道切换了  舍去
   	{
   	   	changl_Num = CHX;
   	   	ADEOC = 0;
   	   	ADST = 1; // 开始转换
   	   	while (ADEOC == 0)
   	   	{ // 检查EOC的标志位   等待转换完毕
   	   	}
   	}
   	ADEOC = 0;
   	ADST = 1; // 开始转换
   	while (ADEOC == 0)
   	{ // 检查EOC的标志位   等待转换完毕
   	}
   	ADC_temp_value = ADRH;
   	ADC_temp_value = ADC_temp_value << 4 | (ADRL & 0x0f);

   	return ADC_temp_value; // 获得ADC转换的数据
}

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
   	u16 ADC_temp_value;

   	ADCR = (ADCR & 0xf8) | CHX; // ADC 使能  AD转换通道开启  通道  CHX
   	for (channel = 0; channel < 20; channel++)
   	{
   	   	ADEOC = 0;
   	   	ADST = 1; // 开始转换
   	   	while (!ADEOC)
   	   	   	; //等待转换完成
   	   	ADC_temp_value = ADRH;
   	   	ADC_temp_value = ADC_temp_value << 4 | (ADRL & 0x0f);
   	   	if (channel < 2)
   	   	   	continue; //丢弃前两次采样的
   	   	if (ADC_temp_value > ADCMAX)
   	   	   	ADCMAX = ADC_temp_value; //最大
   	   	if (ADC_temp_value < ADCMIN)
   	   	   	ADCMIN = ADC_temp_value; //最小
   	   	tmpBuff += ADC_temp_value;
   	}

   	tmpBuff -= ADCMAX; 	   	   	   	 //去掉一个最大
   	tmpBuff -= ADCMIN; 	   	   	   	 //去掉一个最小
   	ADC_temp_value = (tmpBuff >> 4); //除以16，取平均值
   	return ADC_temp_value;
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

// 控制指定个数LED等亮 0 - 6
void led_on_by_gear(u8 n) 
{
   	for (int i=1; i<7; i++) {
   	   	led_on(i, 0);
   	}

   	for (int i=1; i<=n; i++) {
   	   	led_on(i, 1);
   	}
}

// 开机
void fscia_gun_on(void)
{
	// 定时器清0
	timer0_count = 0;
}

// 关机
void fascia_gun_off(void)
{
	// 档位清零
	gear = 0;
	// led全关
	led_on_by_gear(0);
	// 电机关闭
}

u8 led_step = 0;

void main(void)
{
   	Sys_Init();
   	while (1)
   	{  	   	   
   	   	// 检测p01按键
   	   	{
   	   	   	IO_buff.byte = IOP0;
   	   	   	if (IO_BIT1 == 0) {
   	   	   	   	if (key_down_count == 0) {
   	   	   	   	   	key_down_count = timer0_count;
   	   	   	   	}
   	   	   	} else {
				if (key_down_count > 0) {
   	   	   	   		key_press_count = timer0_count - key_down_count;
					key_down_count = 0;	

   	   	   	   		if (key_press_count > 1000) {
   	   	   	   		   	FLAG_KEY_LONG = 1;
   	   	   	   		} else if (key_press_count > 100) {
   	   	   	   		   	FLAG_KEY_SHORT = 1;
   	   	   	   		}
				}
   	   	   	}
   	   	}

		// 按键长按
   	   	if (FLAG_KEY_LONG) {
   	   	   	FLAG_KEY_LONG = 0;
			// 长按关机
			fascia_gun_off();
   	   	}

		// 按键短按
		if (FLAG_KEY_SHORT) {
			FLAG_KEY_SHORT = 0;

			if (gear == 0) {
				// 开机
				fscia_gun_on();
			}

			gear++;

			if (gear > 6) {
				// 大于6档关机
				fascia_gun_off();
			} else {
				// 根据档位显示LED
				led_on_by_gear(gear);
				// 根据档位调节电机
				// TODO:
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
   	if (T0IF && T0IE)
   	{
   	   	T0IF = 0;
   	   	timer0_count++;
   	   	//if (timer0_count >= 5000) //5s
   	   	//{
   	   	//     	//IO_buff.byte = IOP0;
   	   	//     	//IO_BIT2 = !IO_BIT2;
   	   	//     	//IOP0 = IO_buff.byte;

   	   	//     	timer0_count = 0;
   	   	//     	FLAG_TIMER0_5000ms = 1;
   	   	//}
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

