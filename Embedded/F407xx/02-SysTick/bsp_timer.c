#include "bsp_timer.h"
#include "stdio.h"

// ���嶨ʱ���ṹ�����
static TIM_TickTypeDef s_timer[TIM_COUNT];

// ��2��ȫ�ֱ���ת���� Delay_ms() ���� 
static volatile uint32_t DelayCount = 0;
static volatile uint8_t TimeOutFlag = 0;

// ����Ա�ʾ 24.85�죬�����Ʒ��������ʱ�䳬�������������뿼���������
__IO int32_t g_RunningTime = 0;

/* ���� TIM��ʱ�жϵ���ִ�еĻص�����ָ�� */
void (*s_TIM_CallBack1)(void);
void (*s_TIM_CallBack2)(void);
void (*s_TIM_CallBack3)(void);
void (*s_TIM_CallBack4)(void);



void BSP_TimerInit(void)
{
	uint8_t i;
	
	// �������е������ʱ��
	for(i=0; i<TIM_COUNT; i++)
	{
		s_timer[i].Mode = TIM_ONCE_MODE;
		s_timer[i].Count = 0;
		s_timer[i].Flag = 0;
		s_timer[i].PreLoad = 0;	
	}
	
	// ����systic�ж�����Ϊ1ms��������systick�ж�
	SysTick_Config(SystemCoreClock / 1000);

	// ����Ӳ����ʱ�ж�
	BSP_HardTimerInit();   // ��ѡ
}

/**
  * @brief  ����һ�������ʱ��
  * @note   
  * @param  id: �ĸ���ʱ��
  *         period: ��ʱ������,��λ1ms                    
  * @retval None
  */  
void BSP_StartTimer(uint8_t id, uint32_t period)
{
	// �����ж�
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // �����쳣�������ȴ����Ź���λ
	}
	
	// ���ж�
	DISABLE_INT();
	
	// ���ö�ʱ����Ա������ֵ
	s_timer[id].Count = period;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_ONCE_MODE;
	s_timer[id].PreLoad = period;
	
	ENABLE_INT(); // ���ж�
}

/**
  * @brief   �����Զ�ʱ
  * @note   
  * @param  id: �ĸ���ʱ��
  *         period: ��ʱ������,��λ10ms                    
  * @retval None
  */ 
void BSP_StartAutoTimer(uint8_t id, uint32_t period)
{
	// �����ж�
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // �����쳣�������ȴ����Ź���λ
	}
	
	// ���ж�
	DISABLE_INT();
	
	// ���ö�ʱ����Ա������ֵ
	s_timer[id].Count = period;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_AUTO_MODE;
	s_timer[id].PreLoad = period;
	
	ENABLE_INT(); // ���ж�
}

/**
  * @brief ֹͣһ����ʱ��
  * @note   
  * @param id: �ĸ���ʱ�� 
  *                     
  * @retval None
  */ 
void BSP_StopTimer(uint8_t id)
{
	// �����ж�
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // �����쳣�������ȴ����Ź���λ
	}
	
	// ���ж�
	DISABLE_INT();
	
	// ���ö�ʱ����Ա������ֵ
	s_timer[id].Count = 0;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_ONCE_MODE;
	
	ENABLE_INT(); // ���ж�	
}

/**
  * @brief  ��ⶨʱ���Ƿ�ʱ
  * @note   
  * @param  id: �ĸ���ʱ�� 
  *                     
  * @retval 0����ʾ��ʱδ���� 1����ʾ��ʱ��
  */ 
uint8_t BSP_CheckTimer(uint8_t id)
{
	if(id >=TIM_COUNT)
	{	
		return 0;
	}
	
	if(s_timer[id].Flag == 1)
	{
		s_timer[id].Flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief ÿ��1ms�Զ�ʱ�������� 1����SysTick_ISR�е���
  * @note   
  * @param pTimer: ��ʱ������ָ�� 
  *                     
  * @retval None
  */ 
void BSP_TimerDown(TIM_TickTypeDef *pTimer)
{
	if(pTimer->Count > 0)
	{
		// ��ʱ�������־������������ 1
		if(--pTimer->Count == 0)
		{
			pTimer->Flag = 1;
			// ���Զ�ģʽ�����Զ���װ������
			if(pTimer->Mode == TIM_AUTO_MODE)
			{
				pTimer->Count = pTimer->PreLoad;
			}
		}
	}
}


/**
  * @brief SysTick�жϷ������ÿ��1ms����1��
  * @note   
  * @param 
  *                     
  * @retval None
  */
void SysTick_ISR(void)
{
	static uint8_t count = 0;
	uint8_t i;
	
	// ms���ӳ٣�ÿ��1ms�����һ��
	if(DelayCount > 0)
	{
		if(--DelayCount == 0)
		{
			TimeOutFlag =1;    // ��ʱʱ�䵽ʱ�����ñ�־λΪ 1
		}
	}
	
	// ÿ�� 1ms�����ʱ��������һ
	for(i=0; i<TIM_COUNT; i++)
	{
		BSP_TimerDown(&s_timer[i]);
	}
	
	// ÿ�� 1ms��һ������CPU���е�ʱ��
	g_RunningTime++;
	if(g_RunningTime == 0x7FFFFFFF)         // g_RunningTime�����Ϊ 0x7FFFFFFF
	{
		g_RunningTime = 0;
	}
	
	// ����������Ҫ��ʱɨ��ĺ���
	// BSP_RunPer1ms();		// ÿ��1ms����һ�δ˺������˺����� bsp.c 

	if (++count >= 10)
	{
		count = 0;

		// BSP_RunPer10ms();	// ÿ��10ms����һ�δ˺������˺����� bsp.c
	}	
}

/**
  * @brief  ms������ʱ��������systick��ʱ����������ܵ��ô˺�����
  * @note   n : �ӳٳ��ȣ���λ 1ms
  * @param 
  *                     
  * @retval None
  */
void Delay_ms(uint32_t n)
{
	if (n == 0)
	{
		return;
	}
	else if (n == 1)
	{
		n = 2;
	}	
	
	DISABLE_INT();   // ���ж�
	DelayCount = n;
	TimeOutFlag = 0;
	ENABLE_INT();    // ���ж�
	
	while(1)
	{
		//BSP_Idle();  // CPU����ִ�еĲ���
		
		/*
		�ȴ��ӳ�ʱ�䵽
		ע�⣺��������Ϊ TimeOutFlag = 0�����Կ����Ż��������TimeOutFlag ������������Ϊ volatile
		*/
		if (TimeOutFlag == 1)
		{
			break;
		}
	}
	
}	

/**
  * @brief  us������ʱ, ������systick��ʱ����������ܵ��ô˺�����
  * @note   n : �ӳٳ��ȣ���λ 1us
  * @param 
  *                     
  * @retval None
  */
void Delay_us(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt;
    uint32_t reload;

	reload = SysTick->LOAD;
    ticks = n * (SystemCoreClock / 1000000);	 /* ��Ҫ�Ľ����� */

    tcnt = 0;
    told = SysTick->VAL;             /* �ս���ʱ�ļ�����ֵ */

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            // SYSTICK��һ���ݼ��ļ�����
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;               // ����װ�صݼ�
            }
            told = tnow;

            // ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�
            if (tcnt >= ticks)
            {
            	break;
            }
        }
    }
}

/**
  * @brief  ��ȡCPU����ʱ�䣬��λ1ms
  * @note  
  * @param none
  *                     
  * @retval CPU����ʱ�䣬��λ1ms
  */
int32_t CPU_GetRunTime(void)
{
	int32_t time;
	
	DISABLE_INT();
	time = g_RunningTime;    //���������Systick�ж��б���д�������Ҫ���жϽ��б���
	ENABLE_INT();
	return time;	
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	SysTick_ISR();
}



/**
  * @brief  ���� TIM4������us����Ӳ����ʱ��TIM4���������У�����ֹͣ.
  * @param  None
  * @retval None
  * @note   APB1Periph:  PCLK1 = HCLK / 4         168/4=42M
  *         APB2Periph:  PCLK1 = HCLK / 2         168M/2      
  *         AHBPeriph:   HCLK  = SYSCLK  
  */
void BSP_HardTimerInit()
{
	uint32_t Period;
	uint16_t Prescaler;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	
	Period = 0xFFFF;
	Prescaler = (SystemCoreClock/2) / 1000000;            //  ��Ƶ������ 1us
	RCC_APB1PeriphClockCmd(TIM_HARD_RCC, ENABLE);         // enable TIM clock
	
	//Initializes the TIMx Time Base Unit peripheral
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_Period = Period;
	TIM_TimeBaseInitStruct.TIM_Prescaler = Prescaler;
	TIM_TimeBaseInit(TIM_HARD, &TIM_TimeBaseInitStruct);
	
	// enable counter
	TIM_Cmd(TIM_HARD, ENABLE);

	// NVIC init
	NVIC_InitStruct.NVIC_IRQChannel = TIM_HARD_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;   // ��ռ���ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;          // ��Ӧ���ȼ� 
	NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  ʹ��TIM2-5�����ζ�ʱ��ʹ��, ��ʱʱ�䵽��ִ�лص�����������ͬʱ����4����ʱ�����������š�
  *         ��ʱ��������10us ����Ҫ�ķ��ڵ��ñ�������ִ��ʱ�䣬�����ڲ������˲�����С��
  *			TIM2��TIM5 ��32λ��ʱ������ʱ��Χ�ܴ�
  *			TIM3��TIM4 ��16λ��ʱ����
  * @param  CCx : ����ͨ������1��2��3, 4
  *         TimeOut : ��ʱʱ��, ��λ 1us. ����16λ��ʱ������� 65.5ms; ����32λ��ʱ������� 4294��
  *         CallBack : ��ʱʱ�䵽�󣬱�ִ�еĺ���
  * @retval None
  */
void BSP_HardTimerStart(uint8_t CCx, uint32_t TimeOut, void* CallBack)
{
	uint32_t CurrentCnt;
	uint32_t CaptureCnt;
	
	if(TimeOut < 5)
	{
		;
	}
	else
	{
		TimeOut -= 5;
	}
	
	CurrentCnt = TIM_GetCounter(TIM_HARD);// ��ȡ��ǰ�ļ�����ֵ
	CaptureCnt = CurrentCnt + TimeOut;    // ���㲶��ļ�����ֵ
	if(CCx == 1)
	{
		s_TIM_CallBack1 = (void(*) (void))CallBack;
		TIM_SetCompare1(TIM_HARD, CaptureCnt);        // ���ò���Ƚϼ�����CC1 
		TIM_ITConfig(TIM_HARD, TIM_IT_CC1, ENABLE);   // enable CC1 interrupt
	}
	else if(CCx == 2)
	{
		s_TIM_CallBack2 = (void(*) (void))CallBack;
		TIM_SetCompare2(TIM_HARD, CaptureCnt);        // ���ò���Ƚϼ�����CC2
		TIM_ITConfig(TIM_HARD, TIM_IT_CC2, ENABLE);   // enable CC2 interrupt	
	}
	else if(CCx == 3)
	{
		s_TIM_CallBack3 = (void(*) (void))CallBack;
		TIM_SetCompare3(TIM_HARD, CaptureCnt);        // ���ò���Ƚϼ�����CC3
		TIM_ITConfig(TIM_HARD, TIM_IT_CC3, ENABLE);   // enable CC3 interrupt	
	}
	else if(CCx == 4)
	{
		s_TIM_CallBack4 = (void(*) (void))CallBack;
		TIM_SetCompare4(TIM_HARD, CaptureCnt);        // ���ò���Ƚϼ�����CC4
		TIM_ITConfig(TIM_HARD, TIM_IT_CC4, ENABLE);   // enable CC4 interrupt	
	}	
}



// timer interrupt request handler
void TIMx_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC1))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC1);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC1, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC1
		
		// ִ�ж���Ļص��������ص�����������Ҫ������ʱ������Ҫ�ȹ��жϣ�Ȼ����ִ�С� 
		s_TIM_CallBack1();
	}
	
	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC2
		
		// ִ�ж���Ļص��������ص�����������Ҫ������ʱ������Ҫ�ȹ��жϣ�Ȼ����ִ�С� 
		s_TIM_CallBack2();
	}

	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC3
		
		// ִ�ж���Ļص��������ص�����������Ҫ������ʱ������Ҫ�ȹ��жϣ�Ȼ����ִ�С� 
		s_TIM_CallBack3();
	}

	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC4
		
		// ִ�ж���Ļص��������ص�����������Ҫ������ʱ������Ҫ�ȹ��жϣ�Ȼ����ִ�С� 
		s_TIM_CallBack4();
	}
}





