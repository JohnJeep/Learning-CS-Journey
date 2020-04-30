/**
  ******************************************************************************
  * @file    usart.c
  * @author  MCD Application Team
  * @version V0.0.1
  * @date    7-April-2020
  * @brief   ���ü��ִ����շ����ݵķ���
  ******************************************************************************
  * @attention
  *
  *  1. ��ʹ��printf������ӡ
  *  2. ��ѯ��ʽ: ���ݷ��ͺͽ���
  *  3. �жϷ�ʽ: ���ݷ��ͺͽ���
  *  4. DMA��ѯģʽ: ���ݷ��ͺͽ���
  *  5. DMA�ж�ģʽ: ���ݷ��ͺͽ���
  *
  ******************************************************************************
  */

#include "bsp_usart.h"	

void USART_ConfigInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	
	GPIO_RCC_APBxClockCmd(GPIO_RCC_Periph,   ENABLE);                             
	USART_RCC_APBxClockCmd(USART_RCC_Periph, ENABLE);                                    //ʹ��USARTʱ��

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			                         // �ٶ�50MHz	
	
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN; 		
	GPIO_Init(USART_RX_GPIO, &GPIO_InitStructure); 						             
	
	GPIO_InitStructure.GPIO_Pin = USART_TX_PIN; 		
	GPIO_Init(USART_TX_GPIO, &GPIO_InitStructure); 						             
	
	USART_InitStructure.USART_BaudRate = USART_BAUDRATE;					            // ����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	                         // �ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;		                         // һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;			                         // ����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;      // ��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                     // �շ�ģʽ
	USART_Init(USART_ID, &USART_InitStructure); 					                     // ��ʼ������

#if EN_USART_NVIC	
	// �����жϵķ�ʽ
	USART_NVIC_Config();
#else
	// ������ѯ�ķ�ʽ
#endif

    USART_Cmd(USART_ID, ENABLE);                                                        // ʹ�ܴ���
	USART_ClearFlag(USART_ID, USART_FLAG_TC);                                           // ���������ɱ�־	
}

void USART_NVIC_Config(void)
{
	//Ref_USART NVIC ����
	NVIC_InitTypeDef NVIC_InitStructure;	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		                             // �����ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	                         // ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		                             // �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			                             // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							                             // ��ʼ��NVIC�Ĵ���
	
	USART_ITConfig(USART_ID, USART_IT_RXNE, ENABLE);                                     // ��������ж�
	USART_ITConfig(USART_ID, USART_IT_IDLE, ENABLE );                                    // ʹ�ܴ������߿����ж� 	
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

// ����һ��16λ�� 
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	temp_h = (ch&0XFF00)>>8;          // ȡ���߰�λ
	temp_l = ch&0XFF;                 // ȡ���Ͱ�λ
	
	USART_SendData(pUSARTx,temp_h);	  // ���͸߰�λ
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	USART_SendData(pUSARTx,temp_l);	  // ���͵Ͱ�λ
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

#if 1
//�ض���c�⺯��putc�����ڣ��ض�����ʹ��printf()���� 
int fputc(int ch, FILE *f)
{ 	
	USART_SendData(USART_ID, (uint8_t)ch);                           // ����һ���ֽ����ݵ�����
	while (USART_GetFlagStatus(USART_ID, USART_FLAG_TC) == RESET);    // �ȴ��������	   
	return ch;
}

// �ض���c�⺯��getc()���ض�����ʹ��scanf()�����Ӵ�����������
int fgetc(FILE* f)
{
	while(USART_GetFlagStatus(USART_ID, USART_FLAG_RXNE) == RESET);   //�ȴ������������� 
	return (int)USART_ReceiveData(USART_ID);
}
#endif


