#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f4xx.h"

#define TIM_COUNT	4                                           // 设置几个定时器
#define ENABLE_INT()      __set_PRIMASK(0)                      // 使能全局中断
#define DISABLE_INT()     __set_PRIMASK(1)                      // 禁止全局中断

/*
	定义用于硬件定时器的TIM， 可以使 TIM2 - TIM5
	TIM3 和TIM4 是16位
	TIM2 和TIM5 是32位
*/
#define TIM_HARD		       TIM5
#define TIM_HARD_IRQn	       TIM5_IRQn
#define TIM_HARD_RCC	       RCC_APB1Periph_TIM5
#define TIMx_IRQHandler		   TIM5_IRQHandler							                        



// 定义定时器结构体,成员变量为volatile，防止编译器优化
typedef struct
{
	volatile uint8_t    Mode;
	volatile uint8_t    Flag;                                   // 定时到达标志
	volatile uint32_t   Count;
	volatile uint32_t   PreLoad;
}TIM_TickTypeDef;


// 定义定时器工作模式
typedef enum
{
	TIM_ONCE_MODE = 0,                                          // 一次工作模式
	TIM_AUTO_MODE = 1                                           // 自动定时工作模式
	
}TIM_MODE_TypeDef;





// declare function 
void BSP_TimerInit(void);
void BSP_StartTimer(uint8_t id, uint32_t period);
void BSP_StartAutoTimer(uint8_t id, uint32_t period);
void BSP_TimerDown(TIM_TickTypeDef *pTimer);
void BSP_StopTimer(uint8_t id);
uint8_t BSP_CheckTimer(uint8_t id);
int32_t CPU_GetRunTime(void);
void Delay_ms(uint32_t n);
void Delay_us(uint32_t n);
void SysTick_ISR(void);

void BSP_HardTimerInit(void);
void BSP_HardTimerStart(uint8_t CCx, uint32_t TimeOut, void* CallBack);

#endif
