#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "stdio.h"	

// USART1: PA9---Tx   PA10---Rx
#define GPIO_RCC_APBxClockCmd                 RCC_AHB1PeriphClockCmd
#define GPIO_RCC_Periph                       RCC_AHB1Periph_GPIOA

#define USART_RCC_APBxClockCmd                RCC_APB2PeriphClockCmd
#define USART_RCC_Periph                      RCC_APB2Periph_USART1

#define USART_ID                              USART1
#define USART_BAUDRATE                        115200  
										      
#define USART_RX_GPIO                         GPIOA
#define USART_RX_SOURCE                       GPIO_PinSource10
#define USART_RX_PIN                          GPIO_Pin_10
#define USART_RX_AF                           GPIO_AF_USART1
										      
#define USART_TX_GPIO                         GPIOA
#define USART_TX_SOURCE                       GPIO_PinSource9
#define USART_TX_PIN                          GPIO_Pin_9
#define USART_TX_AF                           GPIO_AF_USART1
										      
#define USART_IRQHandler                      USART1_IRQHandler
#define USART_IRQ                 	          USART1_IRQn

void USART_InitConfig(void);
void USART_SendString(USART_TypeDef* pUSARTx, char *str);
void USART_SendByte(USART_TypeDef* pUSARTx, char ch); 

#endif

