/**
  ******************************************************************************
  * @file    dac_port.c
  * @author  zsh
  * @version V1.0
  * @date    12-23-2019
  * @brief   DAC8571�ӿڳ��򣬲���I2C�ӿڵķ�ʽ�뵥Ƭ������ͨ��
  *          I2C2---SDA---PB11             I2C2---SCL---PB10
  *
  ******************************************************************************
  */

#include "dac_port.h"
#include "stm32f10x_i2c.h"
#include "stdio.h"
#include "stm32f10x_gpio.h"

//IO�ڷ���
#define SDA_IN()                         \
    {                                    \
        GPIOB->CRH &= 0XFFFF0FFF;        \
        GPIOB->CRH |= (uint32_t)8 << 12; \
    } // PB11����Ϊ����

#define SDA_OUT()                        \
    {                                    \
        GPIOB->CRH &= 0XFFFF0FFF;        \
        GPIOB->CRH |= (uint32_t)3 << 12; \
    } // PB11����Ϊ�������, �ٶ�Ϊ50MHz

//IO��������
#define I2C_SCL_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_10)   // SCL
#define I2C_SCL_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_10)    // SCL
#define I2C_SDA_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_11)   // SDA
#define I2C_SDA_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_11)    // SDA
#define READ_SDA GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) // ����SDA

// �ٳ�ʼ��I2C GPIO��
void I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Ĭ������PB10, PB11Ϊ�ߵ�ƽ
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
}

// ����һ���ӳ�
void I2C_delay(uint8_t i)
{
    volatile uint32_t delay;
    for (delay = 0; delay < i; delay++)
        ;
}

// ������I2C��ʱ��
//���� IIC ��ʼ�ź�
void I2C_Start(void)
{
    SDA_OUT();
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_delay(4);
    I2C_SDA_LOW(); //START:when CLK is high,DATA change form high to low
    I2C_delay(4);
    I2C_SCL_LOW(); //ǯס I2C ���ߣ�׼�����ͻ��������
}

//���� IIC ֹͣ�ź�
void I2C_Stop(void)
{
    SDA_OUT();
    I2C_SCL_LOW();
    I2C_SDA_LOW(); //STOP:when CLK is high DATA change form low to high
    I2C_delay(4);
    I2C_SCL_HIGH();
    I2C_SDA_HIGH(); //���� I2C ���߽����ź�
    I2C_delay(4);
}

//�ȴ�Ӧ���źŵ���
//����ֵ�� 1������Ӧ��ʧ��; 0������Ӧ��ɹ�
uint8_t I2C_WaitAck(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN();         //SDA ����Ϊ����
    I2C_SDA_HIGH();
    I2C_delay(1);
    I2C_SCL_HIGH();
    I2C_delay(1);
    while (READ_SDA)  // ���������λ�ߵ�ƽ������Ч
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            I2C_Stop();
            return 1;
        }
    }
    I2C_SCL_LOW(); //ʱ����� 0
    return 0;
}

//���� ACK Ӧ��
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

//������ ACK Ӧ��
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

//IIC ����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��; 0����Ӧ��
void I2C_SendByte(uint8_t TxData)
{
    uint8_t i;
    SDA_OUT();
    I2C_SCL_LOW(); //����ʱ�ӿ�ʼ���ݴ���
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

//�� 1 ���ֽڣ� ack=1 ʱ������ ACK�� ack=0������ nACK
uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN(); //SDA ����Ϊ����
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
        I2C_NAck(); // ���� nACK
    else
        I2C_Ack();  // ���� ACK
    return receive;
}

// ������DAC8571��ʱ��
void DAC8571_Write(uint16_t data)
{
    uint8_t H_Byte, L_Byte;

    I2C_Start();
    I2C_SendByte(0x98); // д��������
    I2C_WaitAck();
    I2C_SendByte(0x10); // ��������ģʽ
    I2C_WaitAck();

    // д���ֽ�
    H_Byte = (data >> 8) & 0xFF;
    I2C_SendByte(H_Byte);
    I2C_WaitAck();

    // д���ֽ�
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
    I2C_SendByte(0x99);       // ������
    I2C_WaitAck();
    H_Byte = I2C_ReadByte(1); // �����ֽ�
    L_Byte = I2C_ReadByte(1); // �����ֽ�
    data = (H_Byte << 8) | L_Byte;
    //	I2C_SendByte(0x10);                   // ����temporary registe��������DAC������; update DAC with I2C data
    I2C_SendByte(0x20);                       // Control byte: ������ǰ�����ݣ�֮ǰ�����ݴ洢��temporary registe
    I2C_NAck();                               // �����źŽ����������
    I2C_Stop();

    val = 2.5 * ((float)data / 65535.0);
    printf("DAC sample value: %d \r\n", data);
    printf("DAC voltage value: %fV \r\n", val);
}
