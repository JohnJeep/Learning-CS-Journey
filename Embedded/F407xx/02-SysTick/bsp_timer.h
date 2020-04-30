#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f4xx.h"

#define TIM_COUNT	4                                           // ���ü�����ʱ��
#define ENABLE_INT()      __set_PRIMASK(0)                      // ʹ��ȫ���ж�
#define DISABLE_INT()     __set_PRIMASK(1)                      // ��ֹȫ���ж�

/*
	��������Ӳ����ʱ����TIM�� ����ʹ TIM2 - TIM5
	TIM3 ��TIM4 ��16λ
	TIM2 ��TIM5 ��32λ
*/
#define TIM_HARD		       TIM5
#define TIM_HARD_IRQn	       TIM5_IRQn
#define TIM_HARD_RCC	       RCC_APB1Periph_TIM5
#define TIMx_IRQHandler		   TIM5_IRQHandler							                        



// ���嶨ʱ���ṹ��,��Ա����Ϊvolatile����ֹ�������Ż�
typedef struct
{
	volatile uint8_t    Mode;
	volatile uint8_t    Flag;                                   // ��ʱ�����־
	volatile uint32_t   Count;
	volatile uint32_t   PreLoad;
}TIM_TickTypeDef;


// ���嶨ʱ������ģʽ
typedef enum
{
	TIM_ONCE_MODE = 0,                                          // һ�ι���ģʽ
	TIM_AUTO_MODE = 1                                           // �Զ���ʱ����ģʽ
	
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
