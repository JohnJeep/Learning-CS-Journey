#ifndef __KEY_H
#define __KEY_H

// PF6---K1     PF7---K2     PF8---K3 
#include "stm32f4xx.h"

#define RCC_xxxPeriphClockCmd               RCC_AHB1PeriphClockCmd
#define RCC_xxxPeriph_KEY                   RCC_AHB1Periph_GPIOF

#define GPIO_PIN_K1                         GPIO_Pin_6
#define GPIO_PORT_K1                        GPIOF
							                							            
#define GPIO_PIN_K2                         GPIO_Pin_7
#define GPIO_PORT_K2                        GPIOF

#define GPIO_PIN_K3                         GPIO_Pin_8
#define GPIO_PORT_K3                        GPIOF



#define KEY_COUNT                           3           // ��������
#define KEY_FILTER_TIME                     5           // �����˲��Ĵ���
#define KEY_LONG_TIME                       100			// ��λ10ms�� ����1�룬��Ϊ�����¼�




// ����״̬��up��down��long
typedef enum
{
	KEY_NONE = 0,                          // 0 ��ʾ�����¼�
	
	KEY_1_DOWN,				               // 1������ 
	KEY_1_UP,				               // 1������ 
	KEY_1_LONG,				               // 1������ 	

	KEY_2_DOWN,				               // 2������ 
	KEY_2_UP,				               // 2������ 
	KEY_2_LONG,				               // 2������ 

	KEY_3_DOWN,				               // 3������ 
	KEY_3_UP,				               // 3������ 
	KEY_3_LONG,				               // 3������ 	
	
}KEY_CodeTypeDef;



// ����ID������BSP_KeyState()��������ڲ���
typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3
}KEY_ID_TypeDef;

// key value structure
typedef struct
{
	uint8_t Count;                          // �˲���������
	uint8_t LongCount;                      // ����������
	uint8_t LongTime;                       // �������³���ʱ��, 0��ʾ����ⳤ��
	uint8_t State;                          // ������ǰ״̬�����»��ǵ���
	uint8_t RepeatSpeed;                    // ������������
	uint8_t RepeatCount;                    // ��������������
	
	uint8_t (*WhetherKeyPress)(void);       // �жϰ����Ƿ���
}KEY_ValueTypeDef;


// FIFO structure
#define KEY_FIFO_SIZE                       10
typedef struct
{
	uint8_t Buffer[KEY_FIFO_SIZE];           // ��ֵ������
	uint8_t Read;                            // ��������ָ��1
	uint8_t Write;                           // ������дָ�� 
	uint8_t Read2;                           // ��������ָ��2
}KEY_FIFO_TypeDef;

// function declare

void BSP_KeyInit(void);
void BSP_ConfigIinit(void);
void BSP_KeyValueInit(void);
uint8_t WhetherKeyPress1(void);
uint8_t WhetherKeyPress2(void);
uint8_t WhetherKeyPress3(void);
void BSP_PutKey(uint8_t KeyCode);
uint8_t BSP_GetKey(void);
uint8_t BSP_GetKey2(void);
uint8_t BSP_GetKeyState(KEY_ID_TypeDef KeyID);
void BSP_SetKeyParam(uint8_t KeyID, uint16_t LongTime, uint8_t RepeatSpeed);
void BSP_DetectKey(uint8_t KeyID);
void BSP_KeyScan(void);

#endif

#if 1

#define KEY0 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6) 
#define KEY1 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_7) 
#define KEY2 GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_8) 

// �������º�ķ���ֵ
#define KEY0_PRES 1
#define KEY1_PRES 2
#define KEY2_PRES 3
#define WKUP_PRES 4


//void KEY_Init(void);
unsigned char Key_Query(u8 mode);
#endif
