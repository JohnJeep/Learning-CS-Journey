/**
  ******************************************************************************
  * @file    adc_port.c
  * @author  zsh
  * @version V1.0
  * @date    12-23-2019
  * @brief   ADS1256�ӿڳ���
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
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;      // ����SPI��ʱ�Ӳ���  72M/256=28.125k
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                              // ʱ����λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                                // ����ʱ���ڲ�����ʱ��ʱ��Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;        // ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                        // ���ݴ����λ��ǰ
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                             // ����ģʽ
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                                 // ����NSS�ź����������
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	
	// CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                          // �������
	GPIO_InitStructure.GPIO_Pin = PIN_CS;                                     // PB12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                         
	GPIO_Init(PORT_CS, &GPIO_InitStructure);                                  
													                          
	ADS1256_CS_HIGH();                                                        // Ĭ������ƬѡΪ��
													                          
	// DRDY                                                                   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             // �������� 
	GPIO_InitStructure.GPIO_Pin = PIN_DRDY;                                   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORT_DRDY, &GPIO_InitStructure);
}

// SPI����һ���ֽڣ�ģ��SPIͨ��
uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);  // �жϷ������Ƿ�Ϊ��
	SPI_I2S_SendData(SPI2, TxData);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); // �ȴ��������
	return SPI_I2S_ReceiveData(SPI2);
}

// ���Ĵ�������дֵ
void ADS1256_WriteReg(uint8_t regAddr, uint8_t data)
{
	ADS1256_CS_LOW();
	while(ADS1256_DRDY);                                            // ��ADS1256_DRDYΪ��ʱ������Ĵ���д����
	SPI_ReadWriteByte(ADS1256_CMD_WREG|(regAddr));                  // ��Ĵ���д�����ݵ�ַ
	SPI_ReadWriteByte(0x00);                                        
	SPI_ReadWriteByte(data);                                        // д������
	ADS1256_CS_HIGH();
}

// �ӼĴ��������ֵ
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

// ��ʼ��ADS1256
void ADS1256_Init(void)
{
	ADS1256_GPIO_Init();

	ADS1256_CS_LOW();                                      // �ڽ������е���������ݲ���֮ǰ��CS����Ϊ��
	while(ADS1256_DRDY);  
	SPI_ReadWriteByte(ADS1256_CMD_SELFCAL);                //ƫ�ƺ������Զ�У׼
	ADS1256_CS_HIGH();
	
	ADS1256_WriteReg(ADS1256_STATUS,0x06);                 // ��λ��ǰ��ʹ�û�����Buffer
//	ADS1256_WriteReg(ADS1256_STATUS,0x04);                 // ��λ��ǰ����ʹ�û���Buffer
	ADS1256_WriteReg(ADS1256_ADCON, ADS1256_GAIN_1);       // �Ŵ���1
//	ADS1256_WriteReg(ADS1256_DRATE, ADS1256_DRATE_10SPS);  // ����10sps----��Ҫ100.18ms�ﵽ�ȶ�
	ADS1256_WriteReg(ADS1256_DRATE, ADS1256_DRATE_5SPS);   // ���òɼ��ٶ�Ϊ5sps----��Ҫ200.18ms�ﵽ�ȶ�
	ADS1256_WriteReg(ADS1256_IO,0x00); 

	ADS1256_CS_LOW();
	while(ADS1256_DRDY);  
	SPI_ReadWriteByte(ADS1256_CMD_SELFCAL);
	ADS1256_CS_HIGH(); 
}

// ÿ�ι���һ��ͨ��ʱ����ȡADֵ
uint32_t ADS1256_ReadData(uint8_t channel, uint32_t *data)
{
	uint32_t sum = 0;
	
	//��һ��
	while(ADS1256_DRDY);                                             // ��ADS1256_DRDYΪ��ʱ�����ݿ��Լ���
	ADS1256_WriteReg(ADS1256_MUX, channel | ADS1256_MUXN_AINCOM);    // ͨ������Ҫ��ƬѡADS1256_CS_LOW֮ǰ
	
	// �ڶ���
	ADS1256_CS_LOW();	
	SPI_ReadWriteByte(ADS1256_CMD_SYNC);                             //ͬ������
	SPI_ReadWriteByte(ADS1256_CMD_WAKEUP);                           //ͬ������,   ��������ת������ 
											                         
	// ������	                                                     
	SPI_ReadWriteByte(ADS1256_CMD_RDATA);	                         // ��֮ǰת��ͨ��������
	sum |= (SPI_ReadWriteByte(0xFF) << 16);
	sum |= (SPI_ReadWriteByte(0xFF) << 8);
	sum |=  SPI_ReadWriteByte(0xFF);
	ADS1256_CS_HIGH();
	
	// ���Ĳ� ��DRDY�ٴν���ʱ���ظ����ѭ�������ȸ��¶�·�Ĵ�����Ȼ���ȡ��ǰ�����ݡ�

	channel = (channel >> 4);
	printf("�����%d·", channel);
	printf("sum value is: %d \r\n", sum);
	if(data != (uint32_t *)0)
	{
	    *data = sum;
	}
	return 1;
}
