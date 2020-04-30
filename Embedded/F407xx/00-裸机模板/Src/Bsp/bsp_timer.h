#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "stm32f4xx.h"

void BSP_TimerInit(void);
void Delay_ms(__IO uint32_t time);

#endif
