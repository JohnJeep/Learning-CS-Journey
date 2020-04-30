/**
  ******************************************************************************
  * @file    dac_port.c
  * @author  zsh
  * @version V1.0
  * @date    12-23-2019
  * @brief   DAC8571接口程序，采用I2C接口的方式与单片机进行通信
  *          I2C2---SDA---PB11             I2C2---SCL---PB10
  *
  ******************************************************************************
  */

#include "dac_port.h"
#include "stm32f10x_i2c.h"
#include "stdio.h"
#include "stm32f10x_gpio.h"

//IO口方向
#define SDA_IN()                         \
    {                                    \
        GPIOB->CRH &= 0XFFFF0FFF;        \
        GPIOB->CRH |= (uint32_t)8 << 12; \
    } // PB11设置为输入

#define SDA_OUT()                        \
    {                                    \
        GPIOB->CRH &= 0XFFFF0FFF;        \
        GPIOB->CRH |= (uint32_t)3 << 12; \
    } // PB11设置为推挽输出, 速度为50MHz

//IO操作函数
#define I2C_SCL_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_10)   // SCL
#define I2C_SCL_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_10)    // SCL
#define I2C_SDA_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_11)   // SDA
#define I2C_SDA_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_11)    // SDA
#define READ_SDA GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) // 输入SDA

// ①初始化I2C GPIO口
void I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 默认设置PB10, PB11为高电平
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
}

// 设置一个延迟
void I2C_delay(uint8_t i)
{
    volatile uint32_t delay;
    for (delay = 0; delay < i; delay++)
        ;
}

// ②配置I2C的时序
//产生 IIC 起始信号
void I2C_Start(void)
{
    SDA_OUT();
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_delay(4);
    I2C_SDA_LOW(); //START:when CLK is high,DATA change form high to low
    I2C_delay(4);
    I2C_SCL_LOW(); //钳住 I2C 总线，准备发送或接收数据
}

//产生 IIC 停止信号
void I2C_Stop(void)
{
    SDA_OUT();
    I2C_SCL_LOW();
    I2C_SDA_LOW(); //STOP:when CLK is high DATA change form low to high
    I2C_delay(4);
    I2C_SCL_HIGH();
    I2C_SDA_HIGH(); //发送 I2C 总线结束信号
    I2C_delay(4);
}

//等待应答信号到来
//返回值： 1，接收应答失败; 0，接收应答成功
uint8_t I2C_WaitAck(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();         //SDA 设置为输入
    I2C_SDA_HIGH();
    I2C_delay(1);
    I2C_SCL_HIGH();
    I2C_delay(1);
    while (READ_SDA)  // 读入的数据位高电平，则无效
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_SCL_LOW(); //时钟输出 0
    return 0;
}

//产生 ACK 应答
void I2C_Ack(void)
{
    I2C_SCL_LOW();
    SDA_OUT();
    I2C_SDA_LOW();
    I2C_delay(2);
    I2C_SCL_HIGH();
    I2C_delay(2);
    I2C_SCL_LOW();
}

//不产生 ACK 应答
void I2C_NAck(void)
{
    I2C_SCL_LOW();
    SDA_OUT();
    I2C_SDA_HIGH();
    I2C_delay(2);
    I2C_SCL_HIGH();
    I2C_delay(2);
    I2C_SCL_LOW();
}

//IIC 发送一个字节
//返回从机有无应答
//1，有应答; 0，无应答
void I2C_SendByte(uint8_t TxData)
{
    uint8_t i;
    SDA_OUT();
    I2C_SCL_LOW(); //拉低时钟开始数据传输
    for (i = 0; i < 8; i++)
    {
        if (TxData & 0x80)
        {
            I2C_SDA_HIGH();
        }
        else
        {
            I2C_SDA_LOW();
        }
        TxData <<= 1;
        I2C_delay(2);
        I2C_SCL_HIGH();
        I2C_delay(2);
        I2C_SCL_LOW();
        I2C_delay(2);
    }
}

//读 1 个字节， ack=1 时，发送 ACK， ack=0，发送 nACK
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN(); //SDA 设置为输入
    for (i = 0; i < 8; i++)
    {
        I2C_SCL_LOW();
        I2C_delay(2);
        I2C_SCL_HIGH();
        receive <<= 1;
        if (READ_SDA)
        {
            receive++;
        }
        I2C_delay(1);
    }
    if (!ack)
        I2C_NAck(); // 发送 nACK
    else
        I2C_Ack();  // 发送 ACK
    return receive;
}

// ③设置DAC8571的时序
void DAC8571_Write(uint16_t data)
{
    uint8_t H_Byte, L_Byte;

    I2C_Start();
    I2C_SendByte(0x98); // 写数据命令
    I2C_WaitAck();
    I2C_SendByte(0x10); // 更新数据模式
    I2C_WaitAck();

    // 写高字节
    H_Byte = (data >> 8) & 0xFF;
    I2C_SendByte(H_Byte);
    I2C_WaitAck();

    // 写低字节
    L_Byte = data & 0xFF;
    I2C_SendByte(L_Byte);
    I2C_WaitAck();
    I2C_Stop();
}

void DAC8571_Read(void)
{
    uint8_t H_Byte, L_Byte;
    uint16_t data;
    float val = 0.0;

    I2C_Start();
    I2C_SendByte(0x99);       // 读数据
    I2C_WaitAck();
    H_Byte = I2C_ReadByte(1); // 读低字节
    L_Byte = I2C_ReadByte(1); // 读高字节
    data = (H_Byte << 8) | L_Byte;
    //	I2C_SendByte(0x10);                   // 操作temporary registe，并加载DAC的数据; update DAC with I2C data
    I2C_SendByte(0x20);                       // Control byte: 更新以前的数据，之前的数据存储在temporary registe
    I2C_NAck();                               // 主机信号结束语读数据
    I2C_Stop();

    val = 2.5 * ((float)data / 65535.0);
    printf("DAC sample value: %d \r\n", data);
    printf("DAC voltage value: %fV \r\n", val);
}
