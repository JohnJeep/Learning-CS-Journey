#ifndef __ADC_PORT_H
#define __ADC_PORT_H
#include "stdint.h"

// define commands 
#define ADS1256_CMD_WAKEUP   0x00 
#define ADS1256_CMD_RDATA    0x01 
#define ADS1256_CMD_RDATAC   0x03 
#define ADS1256_CMD_SDATAC   0x0f 
#define ADS1256_CMD_RREG     0x10 
#define ADS1256_CMD_WREG     0x50 
#define ADS1256_CMD_SELFCAL  0xf0 
#define ADS1256_CMD_SELFOCAL 0xf1 
#define ADS1256_CMD_SELFGCAL 0xf2 
#define ADS1256_CMD_SYSOCAL  0xf3 
#define ADS1256_CMD_SYSGCAL  0xf4 
#define ADS1256_CMD_SYNC     0xfc 
#define ADS1256_CMD_STANDBY  0xfd 
#define ADS1256_CMD_REST     0xfe 
 
// define the ADS1256 register values 
#define ADS1256_STATUS       0x00   
#define ADS1256_MUX          0x01   
#define ADS1256_ADCON        0x02   
#define ADS1256_DRATE        0x03   
#define ADS1256_IO           0x04   
#define ADS1256_OFC0         0x05   
#define ADS1256_OFC1         0x06   
#define ADS1256_OFC2         0x07   
#define ADS1256_FSC0         0x08   
#define ADS1256_FSC1         0x09   
#define ADS1256_FSC2         0x0A 
 
 
// define multiplexer codes ����
#define ADS1256_MUXP_AIN0   0x00 
#define ADS1256_MUXP_AIN1   0x10 
#define ADS1256_MUXP_AIN2   0x20 
#define ADS1256_MUXP_AIN3   0x30 
#define ADS1256_MUXP_AIN4   0x40 
#define ADS1256_MUXP_AIN5   0x50 
#define ADS1256_MUXP_AIN6   0x60 
#define ADS1256_MUXP_AIN7   0x70 
#define ADS1256_MUXP_AINCOM 0x80 
 
 // define multiplexer codes ����
#define ADS1256_MUXN_AIN0   0x00 
#define ADS1256_MUXN_AIN1   0x01 
#define ADS1256_MUXN_AIN2   0x02 
#define ADS1256_MUXN_AIN3   0x03 
#define ADS1256_MUXN_AIN4   0x04 
#define ADS1256_MUXN_AIN5   0x05 
#define ADS1256_MUXN_AIN6   0x06 
#define ADS1256_MUXN_AIN7   0x07 
#define ADS1256_MUXN_AINCOM 0x08   
 
 
// define gain codes 
#define ADS1256_GAIN_1      0x00 
#define ADS1256_GAIN_2      0x01 
#define ADS1256_GAIN_4      0x02 
#define ADS1256_GAIN_8      0x03 
#define ADS1256_GAIN_16     0x04 
#define ADS1256_GAIN_32     0x05 
#define ADS1256_GAIN_64     0x06 
 
//define drate codes 
#define ADS1256_DRATE_30000SPS   0xF0 
#define ADS1256_DRATE_15000SPS   0xE0 
#define ADS1256_DRATE_7500SPS    0xD0 
#define ADS1256_DRATE_3750SPS    0xC0 
#define ADS1256_DRATE_2000SPS    0xB0 
#define ADS1256_DRATE_1000SPS    0xA1 
#define ADS1256_DRATE_500SPS     0x92 
#define ADS1256_DRATE_100SPS     0x82 
#define ADS1256_DRATE_60SPS      0x72 
#define ADS1256_DRATE_50SPS      0x63 
#define ADS1256_DRATE_30SPS      0x53 
#define ADS1256_DRATE_25SPS      0x43 
#define ADS1256_DRATE_15SPS      0x33 
#define ADS1256_DRATE_10SPS      0x23 
#define ADS1256_DRATE_5SPS       0x13 
#define ADS1256_DRATE_2_5SPS     0x03

#if 0
// ״̬����
#define ADS1256_MSTATE_RESET             0
#define ADS1256_MSTATE_INIT              1
#define ADS1256_MSTATE_FREE              2
#define ADS1256_MSTATE_SELECT_CHNL       3
#define ADS1256_MSTATE_START_SAMPLE      4
#define ADS1256_MSTATE_READ_SAMPLE       5

#define ADS1256_SSTATE_REG_STATUS        0
#define ADS1256_SSTATE_REG_MUX           1
#define ADS1256_SSTATE_REG_ADCON         2
#define ADS1256_SSTATE_REG_DRATE         3
#define ADS1256_SSTATE_REG_IO            4
#define ADS1256_SSTATE_CALIBRATE         5




#endif

void SPI_GPIO_Init(void);
void ADS1256_GPIO_Init(void);
void ADS1256_WriteReg(uint8_t regAddr, uint8_t data);
void ADS1256_Init(void);
uint8_t SPI_ReadWriteByte(uint8_t TxData);
uint32_t ADS1256_ReadData(uint8_t channel, uint32_t *data);

unsigned long ADS_sum(unsigned char road);
#endif
