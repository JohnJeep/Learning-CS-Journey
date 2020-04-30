/**
  ******************************************************************************
  * @file    main.c
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   ����4������һ����������,3��LED��������,����������������ȼ���һ��
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
#include <includes.h>


// ������ջ
static CPU_STK APP_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED0_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED1_TaskStartStk[APP_TASK_START_STK_SIZE];  
static CPU_STK LED2_TaskStartStk[APP_TASK_START_STK_SIZE];  

     
/*
******************************************************************************
*    
*		AppTaskStartTCB��һ������������������һ��ָ�룬����ָ��һ������ 
*       �����񴴽���֮�����;�����һ���������Ժ�����Ҫ��������������Ҫ
*       ͨ�������������������������������Լ�����ô����������Ϊ NULL��
* 
******************************************************************************
*/
// ��������ƿ�
static OS_TCB AppTaskStartTCB;                                         
static OS_TCB LED0_TaskStartTCB;
static OS_TCB LED1_TaskStartTCB;
static OS_TCB LED2_TaskStartTCB;

// ���������庯��
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
    cpu_clk_freq = BSP_CPU_ClkFreq();                       // ����SysTick reference freq
    cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;    // ����nbr SysTick increments
    OS_CPU_SysTickInit(cnts);                               // ��ʼ��ʱ�ӽ����жϣ���ʼ���жϵ����ȼ���ysTick�жϵ�ʹ��
    Mem_Init();                                             // �ڴ��ʼ��
    

#if OS_CFG_STAT_TASK_EN > 0u
    
    OSStatTaskCPUUsageInit(&err);                           // ��������������ʱCPU������
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
    
    // �����񴴽�����
    OSTaskCreate ((OS_TCB        *)&AppTaskStartTCB,                               // ������ƿ�
                  (CPU_CHAR      *)"App Task Start",                               // ��������
                  (OS_TASK_PTR    )AppTaskStart,                                   // ������������
                  (void          *)0,                                              // ������ں����βΣ����õ�ʱ������Ϊ 0
                  (OS_PRIO        )APP_TASK_START_PRIO,                            // �������ȼ�                        
                  (CPU_STK       *)&APP_TaskStartStk[0],                           // ��ջ����ʼ��ַ
                  (CPU_STK_SIZE   )APP_TASK_START_STK_SIZE / 10,                   // ��ջ��ȵ�����λ��
                  (CPU_STK_SIZE   )APP_TASK_START_STK_SIZE,                        // �����ջ��С, ��32λ�ģ���λΪ ��
                  (OS_MSG_QTY     )5u,                                             // ���͵�����������Ϣ��
                  (OS_TICK        )0u,                                             // ������֮��ѭ��ʱ��ʱ��Ƭ��ʱ�������Եδ�Ϊ��λ��
                  (void          *)0,                                              // ָ���û��ṩ���ڴ�λ�õ�ָ��
                  (OS_OPT         )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),    // �û���ѡ�������ض�ѡ��
                  (OS_ERR        *)&err);                                          // ���淵�صĴ������
    
	// ���������������,��������
     OSStart(&err);
}
