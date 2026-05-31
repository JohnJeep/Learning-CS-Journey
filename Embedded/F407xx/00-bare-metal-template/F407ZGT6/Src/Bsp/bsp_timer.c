#include "bsp_timer.h"

__IO uint32_t count;

void BSP_TimerInit(void)
{
	if(SysTick_Config(SystemCoreClock / 1000))
	{
		while(1);    
	}
}



void Delay_ms(__IO uint32_t time)
{
	count = time;
	while(count != 0);
}

void SysTick_Handler()
{
	if(count != 0)
	{
		count--;
	}
}
