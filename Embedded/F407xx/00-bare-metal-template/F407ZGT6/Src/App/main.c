#include "bsp.h"

int main(void)
{
	USART_ConfigInit();
	LED_Init();
	BSP_TimerInit();	

	
	while(1)
	{
		LED0_TOGGLE;

		Delay_ms(1000);

		GPIO_ResetBits(GPIOG, GPIO_Pin_14);
		Delay_ms(1000);
		GPIO_SetBits(GPIOG, GPIO_Pin_14);
	
		LED2_ON;
		Delay_ms(1000);		
		LED2_OFF;
	}
	
	
}
