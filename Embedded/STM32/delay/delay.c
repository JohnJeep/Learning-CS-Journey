/**
  ******************************************************************************
  * @file    delay.c
  * @author  JohnJeep
  * @version V1.0
  * @date    8-Jaunary-2020 
  * @brief   不搭配操作系统实现us、ms级别的延迟函数。
  * 
  ******************************************************************************
  * @attention
  * 
  * 采用外部的时钟进行精确的延时，必须要有外部的晶振才可以，否则不会实现。
  * 直接使用for循环实现延迟函数，导致延时时间不准确，精确延时采用系统时钟的方法。
  *
  ******************************************************************************
  */

#include "delay.h"
#include "stm32f10x.h" 

static uint8_t  param_us = 0;					    // us延时倍乘数			   
static uint16_t param_ms = 0;					    // ms延时倍乘数
	
// 延时函数初始化
void delay_init(uint16_t SystemClock)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	    // 选择外部时钟  HCLK/8,需要8MHz晶振
	param_us = SystemClock/8;				    // 延时 1us 需要多少个 SysTick 时钟周期。8位整形
	param_ms = (uint16_t)param_us*1000;		            // 延时 1ms 需要多少个 SysTick 时钟周期。16位整形数据
}								    

/**
  * @brief  实现us级延时
  *         
  * @param  us: 延时的微秒数; 保证us <= (2^24)/param_us, 否则超出LOAD寄存器范围，高位被舍去，导致延时时间不准确。
  * @retval None
  * @notice temp & 0x01 判断systick定时器是否还处于开启状态，可以防止systick被意外关闭，从而导致死循环。
  */
void delay_us(uint32_t us)
{		
	uint32_t temp;	    	 
	SysTick->LOAD = us * param_us; 				   // 把延时的数换成SysTick的时钟数，写入到LOAD寄存器器中
	SysTick->VAL = 0x00;        			           // 清空当前寄存器VAL 的内容
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;	           // 开启倒数功能	  
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01) && !(temp&(1<<16)));		           // 等待时间到达，即倒数结束  
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	           // 关闭SysTick计数器
	SysTick->VAL = 0X00;      			           // 清空寄存器VAL的值	 
}

/**
  * @brief  实现ms级延时
  *         
  * @param  ms: 延时的毫秒数; 
  *             ms的最大值为：ms <= 0xFFFFFF*8*1000/SystemClock, 超过这个值否则导致延时时间不准确。
  *             对72M条件下,  nms <= 1864
  * @retval None
  * @notice temp & 0x01 判断systick定时器是否还处于开启状态，可以防止systick被意外关闭，从而导致死循环。
  */
void delay_ms(uint32_t ms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD = (uint32_t)ms*param_ms;			   // 把延时的数换成SysTick的时钟数，写入到LOAD寄存器器中
	SysTick->VAL = 0x00;				           // 清空当前寄存器VAL 的内容
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	           // 开启倒数功能 
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01)&& !(temp&(1<<16)));	                   // 等待时间到达，即倒数结束   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	           // 关闭SysTick计数器
	SysTick->VAL =0X00;       				   // 清空寄存器VAL的值	  	    
} 
