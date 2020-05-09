#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "stdio.h"	

// USART6: PC6---Tx   PC7---Rx    en2---PD15
// USART4: PA0---Tx   PA1---Rx    en1---PA2

#define RS485_GPIO_RCC_xxxClockCmd                  RCC_AHB1PeriphClockCmd
#define RS485_GPIO_RCC_Periph                       RCC_AHB1Periph_GPIOA

#define RS485_USART_RCC_APBxClockCmd                RCC_APB1PeriphClockCmd
#define RS485_USART_RCC_Periph                      RCC_APB1Periph_UART4

#define RS485_USART_ID                              UART4 
#define RS485_BAUDRATE                              115200  
										      
#define RS485_USART_RX_GPIO                         GPIOA
#define RS485_USART_RX_SOURCE                       GPIO_PinSource0
#define RS485_USART_RX_PIN                          GPIO_Pin_0
#define RS485_USART_RX_AF                           GPIO_AF_UART4 
										      
#define RS485_USART_TX_GPIO                         GPIOA
#define RS485_USART_TX_SOURCE                       GPIO_PinSource1
#define RS485_USART_TX_PIN                          GPIO_Pin_1
#define RS485_USART_TX_AF                           GPIO_AF_UART4 
										      
#define RS485_USART_IRQHandler                      UART4_IRQHandler 
#define RS485_USART_IRQ                 	        UART4_IRQn

// RS485控制数据Transmit 和 Receive引脚
#define RS485_PORT_TXEN								GPIOA
#define RS485_RCC_TXEN								RCC_AHB1Periph_GPIOA
#define RS485_PIN_TXEN								GPIO_Pin_2


// RS485芯片控制收发引脚
#define RS485_RX_EN()             RS485_PORT_TXEN->BSRRH = RS485_PIN_TXEN 
#define RS485_TX_EN()             RS485_PORT_TXEN->BSRRL = RS485_PIN_TXEN 


void USART_ConfigInit(void);
void RS485_SendString(uint8_t *str);
void RS485_SendByte(uint8_t ch);
#endif

