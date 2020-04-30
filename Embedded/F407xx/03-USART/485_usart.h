#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "stdio.h"	

// USART1: PB6-Tx   PB7-Rx
#define RS485_GPIO_RCC_APBxClockCmd                 RCC_AHB1PeriphClockCmd
#define RS485_GPIO_RCC_Periph                       RCC_AHB1Periph_GPIOB

#define RS485_USART_RCC_APBxClockCmd                RCC_APB2PeriphClockCmd
#define RS485_USART_RCC_Periph                      RCC_APB2Periph_USART1

#define RS485_USART_ID                              USART1
#define RS485_BAUDRATE                              115200  
										      
#define RS485_USART_RX_GPIO                         GPIOB
#define RS485_USART_RX_SOURCE                       GPIO_PinSource7
#define RS485_USART_RX_PIN                          GPIO_Pin_7
#define RS485_USART_RX_AF                           GPIO_AF_USART1
										      
#define RS485_USART_TX_GPIO                         GPIOB
#define RS485_USART_TX_SOURCE                       GPIO_PinSource6
#define RS485_USART_TX_PIN                          GPIO_Pin_6
#define RS485_USART_TX_AF                           GPIO_AF_USART1
										      
#define RS485_USART_IRQHandler                      USART1_IRQHandler
#define RS485_USART_IRQ                 	        USART1_IRQn

void USART_ConfigInit(void);
void USART_SendString(USART_TypeDef* pUSARTx, char *str);
void USART_SendByte(USART_TypeDef* pUSARTx, char ch); 

#endif

