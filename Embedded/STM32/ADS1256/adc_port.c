/**
  ******************************************************************************
  * @file    adc_port.c
  * @author  zsh
  * @version V1.0
  * @date    12-23-2019
  * @brief   ADS1256接口程序
  *          SPI2_NSS---PB12    SPI2_SCK---PB13        SPI2_MISO---PB14   
  *          SPI2_MOSI---PB15   DRDY---AD_IRQ---PA11
  ******************************************************************************
  */
#include "adc_port.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stdio.h"

#define RCC_DRDY                        RCC_APB2Periph_GPIOA
#define PORT_DRDY                       GPIOA
#define PIN_DRDY                        GPIO_Pin_11

#define RCC_CS                          RCC_APB2Periph_GPIOB
#define PORT_CS                         GPIOB
#define PIN_CS                          GPIO_Pin_12

#define ADS1256_CS_LOW()                GPIO_ResetBits(PORT_CS, PIN_CS) //PORT_CS->BRR  = PIN_CS
#define ADS1256_CS_HIGH()               GPIO_SetBits(PORT_CS, PIN_CS)   //PORT_CS->BSRR = PIN_CS
#define ADS1256_DRDY                    (PORT_DRDY->IDR & PIN_DRDY)     //GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)


void ADS1256_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	// SPI2_NSS---PB12    SPI2_SCK---PB13   SPI2_MISO---PB14   SPI2_MOSI---PB15
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;      // 决定SPI的时钟参数  72M/256=28.125k
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                              // 时钟相位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                                // 串行时钟在不操作时，时钟为低电平
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;        // 双线双向全双工
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                        // 数据传输高位在前
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                             // 主机模式
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                 // 设置NSS信号有软件控制
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	
	// CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                          // 推完输出
	GPIO_InitStructure.GPIO_Pin = PIN_CS;                                     // PB12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                         
	GPIO_Init(PORT_CS, &GPIO_InitStructure);                                  
													                          
	ADS1256_CS_HIGH();                                                        // 默认设置片选为高
													                          
	// DRDY                                                                   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // 上拉输入 
	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;                                   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
}

// SPI发送一个字节，模拟SPI通信
uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);  // 判断发送区是否为空
	SPI_I2S_SendData(SPI2, TxData);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); // 等待接收完成
	return SPI_I2S_ReceiveData(SPI2);
}

// 往寄存器里面写值
void ADS1256_WriteReg(uint8_t regAddr, uint8_t data)
{
	ADS1256_CS_LOW();
	while(ADS1256_DRDY);                                            // 当ADS1256_DRDY为低时才能向寄存器写数据
	SPI_ReadWriteByte(ADS1256_CMD_WREG|(regAddr));                  // 向寄存器写入数据地址
	SPI_ReadWriteByte(0x00);                                        
	SPI_ReadWriteByte(data);                                        // 写入数据
	ADS1256_CS_HIGH();
}

// 从寄存器里面读值
uint8_t ADS1256_ReadReg(uint8_t regAddr)
{
	uint8_t val = 0;
	ADS1256_CS_LOW();
	while(ADS1256_DRDY);  
	SPI_ReadWriteByte(ADS1256_CMD_RREG|(regAddr));
	SPI_ReadWriteByte(0x00);
	val = SPI_ReadWriteByte(0xFF);
	ADS1256_CS_HIGH();
	return val;
}

// 初始化ADS1256
void ADS1256_Init(void)
{
	ADS1256_GPIO_Init();

	ADS1256_CS_LOW();                                      // 在进行所有的命令和数据操作之前，CS必须为低
	while(ADS1256_DRDY);  
	SPI_ReadWriteByte(ADS1256_CMD_SELFCAL);                //偏移和增益自动校准
	ADS1256_CS_HIGH();
	
	ADS1256_WriteReg(ADS1256_STATUS,0x06);                 // 高位在前、使用缓冲器Buffer
//	ADS1256_WriteReg(ADS1256_STATUS,0x04);                 // 高位在前、不使用缓冲Buffer
	ADS1256_WriteReg(ADS1256_ADCON, ADS1256_GAIN_1);       // 放大倍数1
//	ADS1256_WriteReg(ADS1256_DRATE, ADS1256_DRATE_10SPS);  // 数据10sps----需要100.18ms达到稳定
	ADS1256_WriteReg(ADS1256_DRATE, ADS1256_DRATE_5SPS);   // 设置采集速度为5sps----需要200.18ms达到稳定
	ADS1256_WriteReg(ADS1256_IO,0x00); 

	ADS1256_CS_LOW();
	while(ADS1256_DRDY);  
	SPI_ReadWriteByte(ADS1256_CMD_SELFCAL);
	ADS1256_CS_HIGH(); 
}

// 每次工作一个通道时，读取AD值
uint32_t ADS1256_ReadData(uint8_t channel, uint32_t *data)
{
	uint32_t sum = 0;
	
	//第一步
	while(ADS1256_DRDY);                                             // 当ADS1256_DRDY为低时，数据可以检索
	ADS1256_WriteReg(ADS1256_MUX, channel | ADS1256_MUXN_AINCOM);    // 通道设置要在片选ADS1256_CS_LOW之前
	
	// 第二步
	ADS1256_CS_LOW();	
	SPI_ReadWriteByte(ADS1256_CMD_SYNC);                             //同步命令
	SPI_ReadWriteByte(ADS1256_CMD_WAKEUP);                           //同步唤醒,   重新启动转换过程 
											                         
	// 第三步	                                                     
	SPI_ReadWriteByte(ADS1256_CMD_RDATA);	                         // 读之前转换通道的数据
	sum |= (SPI_ReadWriteByte(0xFF) << 16);
	sum |= (SPI_ReadWriteByte(0xFF) << 8);
	sum |=  SPI_ReadWriteByte(0xFF);
	ADS1256_CS_HIGH();
	
	// 第四步 当DRDY再次降低时，重复这个循环，首先更新多路寄存器，然后读取以前的数据。

	channel = (channel >> 4);
	printf("传入第%d路", channel);
	printf("sum value is: %d \r\n", sum);
	if(data != (uint32_t *)0)
	{
	    *data = sum;
	}
	return 1;
}
