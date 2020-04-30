#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x_gpio.h"
#include "stdio.h"	

#define EN_USART_NVIC                     1



/* 实现代码的可扩展性，引脚使用宏定义 */
#define GPIO_RCC_APBxClockCmd                 RCC_APB2PeriphClockCmd
#define GPIO_RCC_Periph                       RCC_APB2Periph_GPIOA

#define USART_RCC_APBxClockCmd                RCC_APB2PeriphClockCmd
#define USART_RCC_Periph                      RCC_APB2Periph_USART1

#define USART_ID                              USART1
#define USART_BAUDRATE                        115200  
										      
#define USART_RX_GPIO                         GPIOA
#define USART_RX_SOURCE                       GPIO_PinSource10
#define USART_RX_PIN                          GPIO_Pin_10

#define USART_TX_GPIO                         GPIOA
#define USART_TX_SOURCE                       GPIO_PinSource9
#define USART_TX_PIN                          GPIO_Pin_9


#define USARTx_IRQ                 	           USART1_IRQn
#define USARTx_IRQHandler                      USART1_IRQHandler

void USART_ConfigInit(void);
void USART_NVIC_Config(void);

void USART_SendByte(USART_TypeDef* pUSARTx, char ch);
void USART_SendString(USART_TypeDef* pUSART, char* str);

#endif


