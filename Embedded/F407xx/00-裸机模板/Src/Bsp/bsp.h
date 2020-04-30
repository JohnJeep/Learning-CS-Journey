#ifndef __BSP_H
#define __BSP_H

#include "bsp_timer.h"
#include "stdio.h"
#include "led.h"
#include "usart.h"


void BSP_Idle(void);
void BSP_RunPer1ms(void);
void BSP_RunPer10ms(void);

#endif
