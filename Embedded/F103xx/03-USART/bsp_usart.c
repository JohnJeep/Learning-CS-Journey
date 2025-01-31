/**
  ******************************************************************************
  * @file    usart.c
  * @author  MCD Application Team
  * @version V0.0.1
  * @date    7-April-2020
  * @brief   常用几种串口收发数据的方法
  ******************************************************************************
  * @attention
  *
  *  1. 简单使用printf函数打印
  *  2. 查询方式: 数据发送和接收
  *  3. 中断方式: 数据发送和接收
  *  4. DMA查询模式: 数据发送和接收
  *  5. DMA中断模式: 数据发送和接收
  *
  ******************************************************************************
  */

#include "bsp_usart.h"	

void USART_ConfigInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	GPIO_RCC_APBxClockCmd(GPIO_RCC_Periph,   ENABLE);                             
	USART_RCC_APBxClockCmd(USART_RCC_Periph, ENABLE);                                    //使能USART时钟

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			                         // 速度50MHz	
	
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN; 		
	GPIO_Init(USART_RX_GPIO, &GPIO_InitStructure); 						             
	
	GPIO_InitStructure.GPIO_Pin = USART_TX_PIN; 		
	GPIO_Init(USART_TX_GPIO, &GPIO_InitStructure); 						             
	
	USART_InitStructure.USART_BaudRate = USART_BAUDRATE;					            // 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                         // 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		                         // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;			                         // 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;      // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                     // 收发模式
	USART_Init(USART_ID, &USART_InitStructure); 					                     // 初始化串口

#if EN_USART_NVIC	
	// 串口中断的方式
	USART_NVIC_Config();
#else
	// 采用轮询的方式
#endif

    USART_Cmd(USART_ID, ENABLE);                                                        // 使能串口
	USART_ClearFlag(USART_ID, USART_FLAG_TC);                                           // 清除发送完成标志	
}

void USART_NVIC_Config(void)
{
	//Ref_USART NVIC 配置
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		                             // 串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	                         // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		                             // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                             // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							                             // 初始化NVIC寄存器
	
	USART_ITConfig(USART_ID, USART_IT_RXNE, ENABLE);                                     // 开启相关中断
	USART_ITConfig(USART_ID, USART_IT_IDLE, ENABLE );                                    // 使能串口总线空闲中断 	
}


// interrupt deal with
void USARTx_IRQHandler(void)
{
	uint8_t ucTemp;
	if (USART_GetITStatus(USART_ID, USART_IT_RXNE)!= RESET)
	{
		ucTemp = USART_ReceiveData(USART_ID);
		USART_SendData(USART_ID, ucTemp);
	}
}


// 发送一个字符
void USART_SendByte(USART_TypeDef* pUSARTx, char ch)
{
	USART_SendData(pUSARTx, ch);                                    // 发送一个字节数据到USART
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	// 等待发送数据寄存器为空， USART_FLAG_TXE发送单个字符
}

// 发送字符串
void USART_SendString(USART_TypeDef* pUSARTx, char *str)
{
	unsigned int k=0;
	do 
	{
	  USART_SendByte(pUSARTx, *(str + k));
	  k++;
	} while(*(str + k)!='\0');
  
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)        //  等待发送完成,USART_FLAG_TC发送多个字符
	{}
}

// 发送一个16位数 
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	temp_h = (ch&0XFF00)>>8;          // 取出高八位
	temp_l = ch&0XFF;                 // 取出低八位
	
	USART_SendData(pUSARTx,temp_h);	  // 发送高八位
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	USART_SendData(pUSARTx,temp_l);	  // 发送低八位
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

#if 1
//重定向c库函数putc到串口，重定向后可使用printf()函数 
int fputc(int ch, FILE *f)
{ 	
	USART_SendData(USART_ID, (uint8_t)ch);                           // 发送一个字节数据到串口
	while (USART_GetFlagStatus(USART_ID, USART_FLAG_TC) == RESET);    // 等待发送完毕	   
	return ch;
}

// 重定向c库函数getc()，重定向后可使用scanf()函数从串口输入数据
int fgetc(FILE* f)
{
	while(USART_GetFlagStatus(USART_ID, USART_FLAG_RXNE) == RESET);   //等待串口输入数据 
	return (int)USART_ReceiveData(USART_ID);
}
#endif


