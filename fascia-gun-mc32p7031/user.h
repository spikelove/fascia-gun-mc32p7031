/******************************************************************************
;  *   	@MCU   	   	   	   	 : MC32P7031
;  *   	@Create Date         : 2019.12.29
;  *   	@Author/Corporation  : Martin/SinoMCU
;  *   	@技术支持QQ群   	   	  : 814641858      
;  *    @晟矽微官方论坛   	   	  : http://bbs.sinomcu.com/	 
;  *   	@Copyright           : 2019 SINOMCU Corporation,All Rights Reserved.
;  *----------------------Abstract Description---------------------------------
;  *   	   	   	   	   	   	定义
******************************************************************************/

#ifndef USER
#define USER

#include <mc35-common.h>
#include "MC32P7031.h"

#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t  unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int


void CLR_RAM(void);
void IO_Init(void);
void TIMER0_INT_Init(void);
void TIMER1_PWM_Init(void);
void ADC_Init(void);
void Sys_Init(void);
uint ADC_Get_Value(uchar CHX);
uint ADC_Get_Value_Average(uchar CHX);

void led_set(u8 n, u8 onoff);

//u8 changl_Num;   	   	   	//存储上次通道值  判断通道值是否改变   	

// 用于延迟计数
volatile u32 delay_count;
// 按键按下计数
volatile u32 key_press_count;
// 用于电池电压adc检测计数
volatile u32 bat_adc_count;
// 档位持续时间计时 
volatile u32 gear_count;

// 当前档位
u8 gear;
// 记录电池电量 0-6
u8 bat_level;

//============Define  Flag=================
typedef union {
   	unsigned char byte;
   	struct
   	{
   	   	u8 bit0 : 1;
   	   	u8 bit1 : 1;
   	   	u8 bit2 : 1;
   	   	u8 bit3 : 1;
   	   	u8 bit4 : 1;
   	   	u8 bit5 : 1;
   	   	u8 bit6 : 1;
   	   	u8 bit7 : 1;
   	} bits;
}bit_flag;
volatile bit_flag IO_buff;
volatile bit_flag flag1;

#define IO_BIT0	   	   	   	IO_buff.bits.bit0
#define IO_BIT1	   	   	   	IO_buff.bits.bit1
#define IO_BIT2	   	   	   	IO_buff.bits.bit2
#define IO_BIT3	   	   	   	IO_buff.bits.bit3
#define IO_BIT4	   	   	   	IO_buff.bits.bit4
#define IO_BIT5	   	   	   	IO_buff.bits.bit5
#define IO_BIT6	   	   	   	IO_buff.bits.bit6
#define IO_BIT7	   	   	   	IO_buff.bits.bit7

#define    	FLAG_TIMER0_5000ms     	flag1.bits.bit0	   	 
#define    	FLAG_KEY_SHORT     	   	flag1.bits.bit1	   	 
#define    	FLAG_KEY_LONG  	   	   	flag1.bits.bit2	   	 
#define    	FLAG_CHARGING  	   	   	flag1.bits.bit3	   	 
#define    	FLAG_LOW_BAT   	   	   	flag1.bits.bit4
#define    	FLAG_KEY_LONG_DONE   	flag1.bits.bit5

#endif


/**************************** end of file *********************************************/

