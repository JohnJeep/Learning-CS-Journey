#include "485_usart.h"	


// 为了给 MAX485芯片预留响应时间
static void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
} 

void USART_ConfigInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RS485_GPIO_RCC_xxxClockCmd(RS485_GPIO_RCC_Periph | RS485_RCC_TXEN, ENABLE);                // 使能USART GPIO时钟
	RS485_USART_RCC_APBxClockCmd(RS485_USART_RCC_Periph, ENABLE);                              // 使能USART时钟
 
	//串口x对应引脚复用映射
	GPIO_PinAFConfig(RS485_USART_TX_GPIO, RS485_USART_TX_SOURCE, RS485_USART_TX_AF);           // 复用为USART
	GPIO_PinAFConfig(RS485_USART_RX_GPIO, RS485_USART_RX_SOURCE, RS485_USART_RX_AF); 

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			                               // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				                               // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				                               // 上拉
																						          
	//Rx                                                                                          
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN; 	                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                               // 复用功能
	GPIO_Init(RS485_USART_RX_GPIO, &GPIO_InitStructure); 			                               	 		
																						          
	//Tx                                                                                          
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN; 	                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                               // 复用功能
	GPIO_Init(RS485_USART_TX_GPIO, &GPIO_InitStructure); 			                         	 		

	// 485控制口
	GPIO_InitStructure.GPIO_Pin   = RS485_PIN_TXEN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(RS485_PORT_TXEN, &GPIO_InitStructure);

   //RS485_USART_ID Init                                                                   
	USART_InitStructure.USART_BaudRate = RS485_BAUDRATE;	                                   // 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                               // 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		                               // 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;			                               // 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;            // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                           // 收发模式
	USART_Init(RS485_USART_ID, &USART_InitStructure); 					

	// NVIC Init
	NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;		                               // 串口中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;	                                   // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		                                   // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                                   // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							                                
																						    
	USART_ITConfig(RS485_USART_ID, USART_IT_RXNE, ENABLE);                                     // 开启相关中断
	USART_Cmd(RS485_USART_ID, ENABLE);  
	USART_ClearFlag(RS485_USART_ID, USART_FLAG_TC);                                            // 清发送完成标志,解决第一个字节不能发送问题
	
	GPIO_ResetBits(RS485_PORT_TXEN, RS485_PIN_TXEN);                                           // 默认开始RS485串口接收(Receive)
}

// 发送一个字符  
void RS485_SendByte(uint8_t ch )
{
	// 使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚	
	USART_SendData(RS485_USART_ID,ch);
	while (USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_TXE) == RESET);	
}

//  发送字符串
void RS485_SendString(uint8_t *str)
{
	unsigned int k = 0;
	
	RS485_TX_EN()	;//	使能发送数据
    do 
    {
        RS485_SendByte(*(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*加短暂延时，保证485发送数据完毕*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	使能接收数据
}

#if 1
//重定向c库函数putc到串口，重定向后可使用printf()函数 
int fputc(int ch, FILE *f)
{ 	
	RS485_TX_EN();
	USART_SendData(RS485_USART_ID, (uint8_t)ch);                           // 发送一个字节数据到串口
	while (USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_TC) == RESET);   // 等待发送完毕	
	RS485_RX_EN();
	return ch;
}

// 重定向c库函数getc()，重定向后可使用scanf()函数从串口输入数据
int fgetc(FILE* f)
{
	RS485_TX_EN();
	while(USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_RXNE) == RESET);   //等待串口输入数据 
	RS485_RX_EN();
	return (int)USART_ReceiveData(RS485_USART_ID);
}
#endif



//中断缓存串口数据
#define UART_BUFF_SIZE      1024

volatile    uint16_t count = 0;
uint8_t     USART_Buffer[UART_BUFF_SIZE];

void RS485_USART_IRQHandler(void)
{
    if(count < UART_BUFF_SIZE)
    {
        if(USART_GetITStatus(RS485_USART_ID, USART_IT_RXNE) != RESET)
        {
            USART_Buffer[count] = USART_ReceiveData(RS485_USART_ID);
            count++;
						
			USART_ClearITPendingBit(RS485_USART_ID, USART_IT_RXNE);
        }
    }
	else
	{
		USART_ClearITPendingBit(RS485_USART_ID, USART_IT_RXNE);
//			clean_rebuff();       
	}
}

//获取接收到的数据和长度，查看缓冲区大数据
char *get_rebuff(uint16_t *len) 
{
    *len = count;
    return (char *)&USART_Buffer;
}

//清空缓冲区里面所有的数据
void clean_rebuff(void) 
{
    uint16_t i=UART_BUFF_SIZE+1;
    count = 0;
	while(i)
	USART_Buffer[--i]=0;
}


