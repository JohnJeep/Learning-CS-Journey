#include "usart.h"	

void USART_InitConfig(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_RCC_APBxClockCmd(GPIO_RCC_Periph, ENABLE);                                      // ʹ��USART GPIOʱ��
	USART_RCC_APBxClockCmd(USART_RCC_Periph, ENABLE);                                    // ʹ��USARTʱ��
 
	//����x��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(USART_TX_GPIO, USART_TX_SOURCE, USART_TX_AF);                       // ����ΪUSART
	GPIO_PinAFConfig(USART_RX_GPIO, USART_RX_SOURCE, USART_RX_AF); 

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			                         // �ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 				                         // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 				                         // ����
																						    
	//Rx                                                                                    
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN; 	                                    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                         // ���ù���
	GPIO_Init(USART_RX_GPIO,&GPIO_InitStructure); 			                         	 		
																						    
	//Tx                                                                                    
	GPIO_InitStructure.GPIO_Pin = USART_TX_PIN; 	                                    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				                         // ���ù���
	GPIO_Init(USART_TX_GPIO, &GPIO_InitStructure); 			                         	 		
																						    
   //USART_ID ��ʼ������                                                                   
	USART_InitStructure.USART_BaudRate = USART_BAUDRATE;	                             // ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                         // �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		                         // һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;			                         // ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;      // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                     // �շ�ģʽ
	USART_Init(USART_ID, &USART_InitStructure); 					
	
	USART_ClearFlag(USART_ID, USART_FLAG_TC);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART_IRQ;		                                 // �����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;	                             // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		                             // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                             // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							                                
																						    
	USART_ITConfig(USART_ID, USART_IT_RXNE, ENABLE);                                     // ��������ж�

	USART_Cmd(USART_ID, ENABLE);  
}


void USART_IRQHandler(void)
{
	uint8_t ucTemp;
	if (USART_GetITStatus(USART_ID, USART_IT_RXNE)!=RESET)
	{
		ucTemp = USART_ReceiveData(USART_ID);
		USART_SendData(USART_ID, ucTemp);
	}
}


// ����һ���ַ�
void USART_SendByte(USART_TypeDef* pUSARTx, char ch)
{
	USART_SendData(pUSARTx, ch);                                    // ����һ���ֽ����ݵ�USART
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	// �ȴ��������ݼĴ���Ϊ�գ� USART_FLAG_TXE���͵����ַ�
}

// �����ַ���
void USART_SendString(USART_TypeDef* pUSARTx, char *str)
{
	unsigned int k=0;
	do 
	{
	  USART_SendByte(pUSARTx, *(str + k));
	  k++;
	} while(*(str + k)!='\0');
  
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)        //  �ȴ��������,USART_FLAG_TC���Ͷ���ַ�
	{}
}

#if 1
//�ض���c�⺯��putc�����ڣ��ض�����ʹ��printf()���� 
int fputc(int ch, FILE *f)
{ 	
	USART_SendData(USART_ID, (uint8_t)ch);                           // ����һ���ֽ����ݵ�����
	while (USART_GetFlagStatus(USART_ID, USART_FLAG_TC) == RESET);   // �ȴ��������	   
	return ch;
}

// �ض���c�⺯��getc()���ض�����ʹ��scanf()�����Ӵ�����������
int fgetc(FILE* f)
{
	while(USART_GetFlagStatus(USART_ID, USART_FLAG_RXNE) == RESET);   //�ȴ������������� 
	return (int)USART_ReceiveData(USART_ID);
}
#endif
