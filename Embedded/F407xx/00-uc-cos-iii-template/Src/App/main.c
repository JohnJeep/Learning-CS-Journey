/**
  ******************************************************************************
  * @file    main.c
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   创建4个任务，一个启动任务,3个LED控制任务,三个控制任务的优先级不一样
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
#include <includes.h>


// ①任务栈
static CPU_STK APP_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED0_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED1_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED2_TaskStartStk[APP_TASK_START_STK_SIZE];  

     
/*
******************************************************************************
*    
*		AppTaskStartTCB是一个任务句柄，任务句柄是一个指针，用于指向一个任务 
*       当任务创建好之后，它就具有了一个任务句柄以后我们要想操作这个任务都需要
*       通过这个任务句柄，如果是自身的任务操作自己，那么这个句柄可以为 NULL。
* 
******************************************************************************
*/
// ②任务控制块
static OS_TCB AppTaskStartTCB;                                         
static OS_TCB LED0_TaskStartTCB;
static OS_TCB LED1_TaskStartTCB;
static OS_TCB LED2_TaskStartTCB;

// ③任务主体函数
static void LED0_TaskStart(void *arg)
{
	OS_ERR err;
	(void) arg;
	
	while(DEF_TRUE)
    {
        LED0_TOGGLE;
        OSTimeDly(500, OS_OPT_TIME_DLY, &err);
    }
}

static void LED1_TaskStart(void *arg)
{
	OS_ERR err;
	(void) arg;
	
	while(DEF_TRUE)
    {
        LED1_TOGGLE;
        OSTimeDly(1000, OS_OPT_TIME_DLY, &err);
    }
}

static void LED2_TaskStart(void *arg)
{
	OS_ERR err;
	(void) arg;
	
	while(DEF_TRUE)
    {
        LED2_TOGGLE;
        OSTimeDly(200, OS_OPT_TIME_DLY, &err);
    }
}

static void AppTaskStart(void *arg)
{
    CPU_INT32U cpu_clk_freq;
    CPU_INT32U         cnts;
    OS_ERR              err;
    (void)              arg;
			            
    CPU_Init();  
    BSP_Init();
    cpu_clk_freq = BSP_CPU_ClkFreq();                       // 决定SysTick reference freq
    cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;    // 决定nbr SysTick increments
    OS_CPU_SysTickInit(cnts);                               // 初始化时钟节拍中断，初始化中断的优先级，ysTick中断的使能
    Mem_Init();                                             // 内存初始化
    

#if OS_CFG_STAT_TASK_EN > 0u
    
    OSStatTaskCPUUsageInit(&err);                           // 计算无任务运行时CPU的能力
#endif
    
#ifdef CPU_CFG_INT_DIS_MEAS_EN    
    CPU_IntDisMeasMaxCurReset();
#endif

	// LED0
    OSTaskCreate ((OS_TCB        *)&LED0_TaskStartTCB,
                  (CPU_CHAR      *)"LED0 Task Start",                               
                  (OS_TASK_PTR    )LED0_TaskStart,                                   
                  (void          *)0,                                              
                  (OS_PRIO        )LED0_TASK_START_PRIO,                            
                  (CPU_STK       *)&LED0_TaskStartStk[0],                            
                  (CPU_STK_SIZE   )LED0_TASK_START_STK_SIZE / 10,                   
                  (CPU_STK_SIZE   )LED0_TASK_START_STK_SIZE,                        
                  (OS_MSG_QTY     )5u,                                             
                  (OS_TICK        )0u,                                             
                  (void          *)0,                                              
                  (OS_OPT         )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),    
                  (OS_ERR        *)&err); 
	// LED1
    OSTaskCreate ((OS_TCB        *)&LED1_TaskStartTCB,
                  (CPU_CHAR      *)"LED1 Task Start",                               
                  (OS_TASK_PTR    )LED1_TaskStart,                                   
                  (void          *)0,                                              
                  (OS_PRIO        )LED1_TASK_START_PRIO,                            
                  (CPU_STK       *)&LED1_TaskStartStk[0],                            
                  (CPU_STK_SIZE   )LED1_TASK_START_STK_SIZE / 10,                   
                  (CPU_STK_SIZE   )LED1_TASK_START_STK_SIZE,                        
                  (OS_MSG_QTY     )5u,                                             
                  (OS_TICK        )0u,                                             
                  (void          *)0,                                              
                  (OS_OPT         )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),    
                  (OS_ERR        *)&err); 
	// LED2			  
    OSTaskCreate ((OS_TCB        *)&LED2_TaskStartTCB,
                  (CPU_CHAR      *)"LED2 Task Start",                               
                  (OS_TASK_PTR    )LED2_TaskStart,                                   
                  (void          *)0,                                              
                  (OS_PRIO        )LED2_TASK_START_PRIO,                            
                  (CPU_STK       *)&LED2_TaskStartStk[0],                            
                  (CPU_STK_SIZE   )LED2_TASK_START_STK_SIZE / 10,                   
                  (CPU_STK_SIZE   )LED2_TASK_START_STK_SIZE,                        
                  (OS_MSG_QTY     )5u,                                             
                  (OS_TICK        )0u,                                             
                  (void          *)0,                                              
                  (OS_OPT         )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),    
                  (OS_ERR        *)&err);
}


int main(void)
{	
    OS_ERR err;
    
    BSP_Init();
    OSInit(&err);
    
    // ④任务创建函数
    OSTaskCreate ((OS_TCB        *)&AppTaskStartTCB,                               // 任务控制块
                  (CPU_CHAR      *)"App Task Start",                               // 任务名字
                  (OS_TASK_PTR    )AppTaskStart,                                   // 任务函数的名称
                  (void          *)0,                                              // 任务入口函数形参，不用的时候配置为 0
                  (OS_PRIO        )APP_TASK_START_PRIO,                            // 任务优先级                        
                  (CPU_STK       *)&APP_TaskStartStk[0],                           // 堆栈的起始地址
                  (CPU_STK_SIZE   )APP_TASK_START_STK_SIZE / 10,                   // 堆栈深度的限制位置
                  (CPU_STK_SIZE   )APP_TASK_START_STK_SIZE,                        // 任务堆栈大小, 是32位的，单位为 字
                  (OS_MSG_QTY     )5u,                                             // 发送到任务的最大消息数
                  (OS_TICK        )0u,                                             // 在任务之间循环时的时间片的时间量（以滴答为单位）
                  (void          *)0,                                              // 指向用户提供的内存位置的指针
                  (OS_OPT         )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),    // 用户可选的任务特定选项
                  (OS_ERR        *)&err);                                          // 保存返回的错误代码
    
	// ⑤启动任务调度器,调度任务
     OSStart(&err);
}
