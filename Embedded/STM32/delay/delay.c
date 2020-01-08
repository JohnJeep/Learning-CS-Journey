/**
  ******************************************************************************
  * @file    delay.c
  * @author  JohnJeep
  * @version V1.0
  * @date    8-Jaunary-2020 
  * @brief   ���������ϵͳʵ��us��ms������ӳٺ�����
  * 
  ******************************************************************************
  * @attention
  * 
  * �����ⲿ��ʱ�ӽ��о�ȷ����ʱ������Ҫ���ⲿ�ľ���ſ��ԣ����򲻻�ʵ�֡�
  * ֱ��ʹ��forѭ��ʵ���ӳٺ�����������ʱʱ�䲻׼ȷ����ȷ��ʱ����ϵͳʱ�ӵķ�����
  *
  ******************************************************************************
  */

#include "delay.h"
#include "stm32f10x.h" 

static uint8_t  param_us = 0;							//us��ʱ������			   
static uint16_t param_ms = 0;							//ms��ʱ������
	
	
// ��ʱ������ʼ��
void delay_init(uint16_t SystemClock)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	    // ѡ���ⲿʱ��  HCLK/8,��Ҫ8MHz����
	param_us = SystemClock/8;				            // ��ʱ 1us ��Ҫ���ٸ� SysTick ʱ�����ڡ�8λ����
	param_ms = (uint16_t)param_us*1000;					        // ��ʱ 1ms ��Ҫ���ٸ� SysTick ʱ�����ڡ�16λ��������
}								    

/**
  * @brief  ʵ��us����ʱ
  *         
  * @param  us: ��ʱ��΢����; ��֤us <= (2^24)/param_us, ���򳬳�LOAD�Ĵ�����Χ����λ����ȥ��������ʱʱ�䲻׼ȷ��
  * @retval None
  * @notice temp & 0x01 �ж�systick��ʱ���Ƿ񻹴��ڿ���״̬�����Է�ֹsystick������رգ��Ӷ�������ѭ����
  */
void delay_us(uint32_t us)
{		
	uint32_t temp;	    	 
	SysTick->LOAD = us * param_us; 					// ����ʱ��������SysTick��ʱ������д�뵽LOAD�Ĵ�������
	SysTick->VAL = 0x00;        					// ��յ�ǰ�Ĵ���VAL ������
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;	    // ������������	  
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01) && !(temp&(1<<16)));		    // �ȴ�ʱ�䵽�����������  
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	    // �ر�SysTick������
	SysTick->VAL = 0X00;      					    // ��ռĴ���VAL��ֵ	 
}

/**
  * @brief  ʵ��ms����ʱ
  *         
  * @param  ms: ��ʱ�ĺ�����; 
  *             ms�����ֵΪ��ms <= 0xFFFFFF*8*1000/SystemClock, �������ֵ��������ʱʱ�䲻׼ȷ��
  *             ��72M������,  nms <= 1864
  * @retval None
  * @notice temp & 0x01 �ж�systick��ʱ���Ƿ񻹴��ڿ���״̬�����Է�ֹsystick������رգ��Ӷ�������ѭ����
  */
void delay_ms(uint32_t ms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD = (uint32_t)ms*param_ms;			// ����ʱ��������SysTick��ʱ������д�뵽LOAD�Ĵ�������
	SysTick->VAL = 0x00;							// ��յ�ǰ�Ĵ���VAL ������
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	    // ������������ 
	do
	{
		temp = SysTick->CTRL;
	}while((temp&0x01)&& !(temp&(1<<16)));	       // �ȴ�ʱ�䵽�����������   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	   // �ر�SysTick������
	SysTick->VAL =0X00;       					   // ��ռĴ���VAL��ֵ	  	    
} 
