#ifndef __DELAY_H
#define __DELAY_H 			    
#include "stdint.h"

void delay_init(uint16_t SystemClock);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
#endif
