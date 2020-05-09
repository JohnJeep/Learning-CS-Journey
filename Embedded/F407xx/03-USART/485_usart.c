#include "485_usart.h"	


// Ϊ�˸� MAX485оƬԤ����Ӧʱ��
static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
} 

void USART_ConfigInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RS485_GPIO_RCC_xxxClockCmd(RS485_GPIO_RCC_Periph | RS485_RCC_TXEN, ENABLE);                // ʹ��USART GPIOʱ��
	RS485_USART_RCC_APBxClockCmd(RS485_USART_RCC_Periph, ENABLE);                              // ʹ��USARTʱ��
 
	//����x��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(RS485_USART_TX_GPIO, RS485_USART_TX_SOURCE, RS485_USART_TX_AF);           // ����ΪUSART
	GPIO_PinAFConfig(RS485_USART_RX_GPIO, RS485_USART_RX_SOURCE, RS485_USART_RX_AF); 

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			                               // �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				                               // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				                               // ����
																						          
	//Rx                                                                                          
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN; 	                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                               // ���ù���
	GPIO_Init(RS485_USART_RX_GPIO, &GPIO_InitStructure); 			                               	 		
																						          
	//Tx                                                                                          
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN; 	                                          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                               // ���ù���
	GPIO_Init(RS485_USART_TX_GPIO, &GPIO_InitStructure); 			                         	 		

	// 485���ƿ�
	GPIO_InitStructure.GPIO_Pin   = RS485_PIN_TXEN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(RS485_PORT_TXEN, &GPIO_InitStructure);

   //RS485_USART_ID Init                                                                   
	USART_InitStructure.USART_BaudRate = RS485_BAUDRATE;	                                   // ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                               // �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		                               // һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;			                               // ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;            // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                           // �շ�ģʽ
	USART_Init(RS485_USART_ID, &USART_InitStructure); 					

	// NVIC Init
	NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;		                               // �����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;	                                   // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		                                   // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                                   // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							                                
																						    
	USART_ITConfig(RS485_USART_ID, USART_IT_RXNE, ENABLE);                                     // ��������ж�
	USART_Cmd(RS485_USART_ID, ENABLE);  
	USART_ClearFlag(RS485_USART_ID, USART_FLAG_TC);                                            // �巢����ɱ�־,�����һ���ֽڲ��ܷ�������
	
	GPIO_ResetBits(RS485_PORT_TXEN, RS485_PIN_TXEN);                                           // Ĭ�Ͽ�ʼRS485���ڽ���(Receive)
}

// ����һ���ַ�  
void RS485_SendByte(uint8_t ch )
{
	// ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ�������	
	USART_SendData(RS485_USART_ID,ch);
	while (USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_TXE) == RESET);	
}

//  �����ַ���
void RS485_SendString(uint8_t *str)
{
	unsigned int k = 0;
	
	RS485_TX_EN()	;//	ʹ�ܷ�������
    do 
    {
        RS485_SendByte(*(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
	/*�Ӷ�����ʱ����֤485�����������*/
	Delay(0xFFF);
		
	RS485_RX_EN()	;//	ʹ�ܽ�������
}

#if 1
//�ض���c�⺯��putc�����ڣ��ض�����ʹ��printf()���� 
int fputc(int ch, FILE *f)
{ 	
	RS485_TX_EN();
	USART_SendData(RS485_USART_ID, (uint8_t)ch);                           // ����һ���ֽ����ݵ�����
	while (USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_TC) == RESET);   // �ȴ��������	
	RS485_RX_EN();
	return ch;
}

// �ض���c�⺯��getc()���ض�����ʹ��scanf()�����Ӵ�����������
int fgetc(FILE* f)
{
	RS485_TX_EN();
	while(USART_GetFlagStatus(RS485_USART_ID, USART_FLAG_RXNE) == RESET);   //�ȴ������������� 
	RS485_RX_EN();
	return (int)USART_ReceiveData(RS485_USART_ID);
}
#endif



//�жϻ��洮������
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

//��ȡ���յ������ݺͳ��ȣ��鿴������������
char *get_rebuff(uint16_t *len) 
{
    *len = count;
    return (char *)&USART_Buffer;
}

//��ջ������������е�����
void clean_rebuff(void) 
{
    uint16_t i=UART_BUFF_SIZE+1;
    count = 0;
	while(i)
	USART_Buffer[--i]=0;
}


