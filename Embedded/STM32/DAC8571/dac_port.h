#ifndef __DAC_PORT_H
#define __DAC_PORT_H
#include "stdint.h"

void I2C_GPIO_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaitAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_SendByte(uint8_t TxData);
uint8_t I2C_ReadByte(uint8_t ack);

void DAC8571_Write(uint16_t data);
void DAC8571_Read(void);
void I2C_delay(uint8_t i);

#endif
