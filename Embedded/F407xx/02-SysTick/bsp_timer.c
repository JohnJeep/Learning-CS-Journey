#include "bsp_timer.h"
#include "stdio.h"

// 定义定时器结构体变量
static TIM_TickTypeDef s_timer[TIM_COUNT];

// 这2个全局变量转用于 Delay_ms() 函数 
static volatile uint32_t DelayCount = 0;
static volatile uint8_t TimeOutFlag = 0;

// 最长可以表示 24.85天，如果产品连续运行时间超过这个数，则必须考虑溢出问题
__IO int32_t g_RunningTime = 0;

/* 保存 TIM定时中断到后执行的回调函数指针 */
void (*s_TIM_CallBack1)(void);
void (*s_TIM_CallBack2)(void);
void (*s_TIM_CallBack3)(void);
void (*s_TIM_CallBack4)(void);



void BSP_TimerInit(void)
{
	uint8_t i;
	
	// 清零所有的软件定时器
	for(i=0; i<TIM_COUNT; i++)
	{
		s_timer[i].Mode = TIM_ONCE_MODE;
		s_timer[i].Count = 0;
		s_timer[i].Flag = 0;
		s_timer[i].PreLoad = 0;	
	}
	
	// 配置systic中断周期为1ms，并启动systick中断
	SysTick_Config(SystemCoreClock / 1000);

	// 开启硬件定时中断
	BSP_HardTimerInit();   // 可选
}

/**
  * @brief  启动一次软件定时器
  * @note   
  * @param  id: 哪个定时器
  *         period: 定时器周期,单位1ms                    
  * @retval None
  */  
void BSP_StartTimer(uint8_t id, uint32_t period)
{
	// 错误判断
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // 参数异常，死机等待看门狗复位
	}
	
	// 关中断
	DISABLE_INT();
	
	// 设置定时器成员变量的值
	s_timer[id].Count = period;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_ONCE_MODE;
	s_timer[id].PreLoad = period;
	
	ENABLE_INT(); // 开中断
}

/**
  * @brief   周期性定时
  * @note   
  * @param  id: 哪个定时器
  *         period: 定时器周期,单位10ms                    
  * @retval None
  */ 
void BSP_StartAutoTimer(uint8_t id, uint32_t period)
{
	// 错误判断
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // 参数异常，死机等待看门狗复位
	}
	
	// 关中断
	DISABLE_INT();
	
	// 设置定时器成员变量的值
	s_timer[id].Count = period;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_AUTO_MODE;
	s_timer[id].PreLoad = period;
	
	ENABLE_INT(); // 开中断
}

/**
  * @brief 停止一个定时器
  * @note   
  * @param id: 哪个定时器 
  *                     
  * @retval None
  */ 
void BSP_StopTimer(uint8_t id)
{
	// 错误判断
	if(id >= TIM_COUNT)
	{
		printf("Error file: %s; Error func: %s()\r\n", __FILE__, __FUNCTION__);
		while(1);    // 参数异常，死机等待看门狗复位
	}
	
	// 关中断
	DISABLE_INT();
	
	// 设置定时器成员变量的值
	s_timer[id].Count = 0;
	s_timer[id].Flag = 0;
	s_timer[id].Mode = TIM_ONCE_MODE;
	
	ENABLE_INT(); // 开中断	
}

/**
  * @brief  检测定时器是否超时
  * @note   
  * @param  id: 哪个定时器 
  *                     
  * @retval 0：表示定时未到， 1：表示定时到
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
  * @brief 每隔1ms对定时器变量减 1，在SysTick_ISR中调用
  * @note   
  * @param pTimer: 定时器变量指针 
  *                     
  * @retval None
  */ 
void BSP_TimerDown(TIM_TickTypeDef *pTimer)
{
	if(pTimer->Count > 0)
	{
		// 定时器到达标志：计数器减到 1
		if(--pTimer->Count == 0)
		{
			pTimer->Flag = 1;
			// 是自动模式，则自动重装计数器
			if(pTimer->Mode == TIM_AUTO_MODE)
			{
				pTimer->Count = pTimer->PreLoad;
			}
		}
	}
}


/**
  * @brief SysTick中断服务程序，每隔1ms进入1次
  * @note   
  * @param 
  *                     
  * @retval None
  */
void SysTick_ISR(void)
{
	static uint8_t count = 0;
	uint8_t i;
	
	// ms级延迟，每隔1ms会进来一次
	if(DelayCount > 0)
	{
		if(--DelayCount == 0)
		{
			TimeOutFlag =1;    // 延时时间到时，设置标志位为 1
		}
	}
	
	// 每隔 1ms软件定时器数量减一
	for(i=0; i<TIM_COUNT; i++)
	{
		BSP_TimerDown(&s_timer[i]);
	}
	
	// 每隔 1ms加一，计算CPU运行的时间
	g_RunningTime++;
	if(g_RunningTime == 0x7FFFFFFF)         // g_RunningTime最大数为 0x7FFFFFFF
	{
		g_RunningTime = 0;
	}
	
	// 调用其它需要定时扫描的函数
	// BSP_RunPer1ms();		// 每隔1ms调用一次此函数，此函数在 bsp.c 

	if (++count >= 10)
	{
		count = 0;

		// BSP_RunPer10ms();	// 每隔10ms调用一次此函数，此函数在 bsp.c
	}	
}

/**
  * @brief  ms级别延时，必须在systick定时器启动后才能调用此函数。
  * @note   n : 延迟长度，单位 1ms
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
	
	DISABLE_INT();   // 关中断
	DelayCount = n;
	TimeOutFlag = 0;
	ENABLE_INT();    // 开中断
	
	while(1)
	{
		//BSP_Idle();  // CPU空闲执行的操作
		
		/*
		等待延迟时间到
		注意：编译器认为 TimeOutFlag = 0，所以可能优化错误，因此TimeOutFlag 变量必须申明为 volatile
		*/
		if (TimeOutFlag == 1)
		{
			break;
		}
	}
	
}	

/**
  * @brief  us级别延时, 必须在systick定时器启动后才能调用此函数。
  * @note   n : 延迟长度，单位 1us
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
    ticks = n * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */

    tcnt = 0;
    told = SysTick->VAL;             /* 刚进入时的计数器值 */

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            // SYSTICK是一个递减的计数器
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;               // 重新装载递减
            }
            told = tnow;

            // 时间超过/等于要延迟的时间,则退出
            if (tcnt >= ticks)
            {
            	break;
            }
        }
    }
}

/**
  * @brief  获取CPU运行时间，单位1ms
  * @note  
  * @param none
  *                     
  * @retval CPU运行时间，单位1ms
  */
int32_t CPU_GetRunTime(void)
{
	int32_t time;
	
	DISABLE_INT();
	time = g_RunningTime;    //这个变量在Systick中断中被改写，因此需要关中断进行保护
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
  * @brief  配置 TIM4，用于us级别硬件定时。TIM4将自由运行，永不停止.
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
	Prescaler = (SystemCoreClock/2) / 1000000;            //  分频到周期 1us
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
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;   // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;          // 响应优先级 
	NVIC_Init(&NVIC_InitStruct);
}

/**
  * @brief  使用TIM2-5做单次定时器使用, 定时时间到后执行回调函数。可以同时启动4个定时器，互不干扰。
  *         定时精度正负10us （主要耗费在调用本函数的执行时间，函数内部进行了补偿减小误差）
  *			TIM2和TIM5 是32位定时器。定时范围很大
  *			TIM3和TIM4 是16位定时器。
  * @param  CCx : 捕获通道几，1，2，3, 4
  *         TimeOut : 超时时间, 单位 1us. 对于16位定时器，最大 65.5ms; 对于32位定时器，最大 4294秒
  *         CallBack : 定时时间到后，被执行的函数
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
	
	CurrentCnt = TIM_GetCounter(TIM_HARD);// 读取当前的计数器值
	CaptureCnt = CurrentCnt + TimeOut;    // 计算捕获的计数器值
	if(CCx == 1)
	{
		s_TIM_CallBack1 = (void(*) (void))CallBack;
		TIM_SetCompare1(TIM_HARD, CaptureCnt);        // 设置捕获比较计数器CC1 
		TIM_ITConfig(TIM_HARD, TIM_IT_CC1, ENABLE);   // enable CC1 interrupt
	}
	else if(CCx == 2)
	{
		s_TIM_CallBack2 = (void(*) (void))CallBack;
		TIM_SetCompare2(TIM_HARD, CaptureCnt);        // 设置捕获比较计数器CC2
		TIM_ITConfig(TIM_HARD, TIM_IT_CC2, ENABLE);   // enable CC2 interrupt	
	}
	else if(CCx == 3)
	{
		s_TIM_CallBack3 = (void(*) (void))CallBack;
		TIM_SetCompare3(TIM_HARD, CaptureCnt);        // 设置捕获比较计数器CC3
		TIM_ITConfig(TIM_HARD, TIM_IT_CC3, ENABLE);   // enable CC3 interrupt	
	}
	else if(CCx == 4)
	{
		s_TIM_CallBack4 = (void(*) (void))CallBack;
		TIM_SetCompare4(TIM_HARD, CaptureCnt);        // 设置捕获比较计数器CC4
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
		
		// 执行定义的回调函数。回调函数可能需要重启定时器，需要先关中断，然后再执行。 
		s_TIM_CallBack1();
	}
	
	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC2);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC2, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC2
		
		// 执行定义的回调函数。回调函数可能需要重启定时器，需要先关中断，然后再执行。 
		s_TIM_CallBack2();
	}

	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC3);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC3, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC3
		
		// 执行定义的回调函数。回调函数可能需要重启定时器，需要先关中断，然后再执行。 
		s_TIM_CallBack3();
	}

	if(TIM_GetITStatus(TIM_HARD, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM_HARD, TIM_IT_CC4);
		TIM_ITConfig(TIM_HARD, TIM_IT_CC4, DISABLE);     // Disable TIM interrupts:  TIM_IT_CC4
		
		// 执行定义的回调函数。回调函数可能需要重启定时器，需要先关中断，然后再执行。 
		s_TIM_CallBack4();
	}
}





