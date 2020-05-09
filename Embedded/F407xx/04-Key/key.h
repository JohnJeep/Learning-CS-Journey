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



#define KEY_COUNT                           3           // 按键个数
#define KEY_FILTER_TIME                     5           // 按键滤波的次数
#define KEY_LONG_TIME                       100			// 单位10ms， 持续1秒，认为长按事件




// 按键状态：up、down、long
typedef enum
{
	KEY_NONE = 0,                          // 0 表示按键事件
	
	KEY_1_DOWN,				               // 1键按下 
	KEY_1_UP,				               // 1键弹起 
	KEY_1_LONG,				               // 1键长按 	

	KEY_2_DOWN,				               // 2键按下 
	KEY_2_UP,				               // 2键弹起 
	KEY_2_LONG,				               // 2键长按 

	KEY_3_DOWN,				               // 3键按下 
	KEY_3_UP,				               // 3键弹起 
	KEY_3_LONG,				               // 3键长按 	
	
}KEY_CodeTypeDef;



// 按键ID，用于BSP_KeyState()函数的入口参数
typedef enum
{
	KID_K1 = 0,
	KID_K2,
	KID_K3
}KEY_ID_TypeDef;

// key value structure
typedef struct
{
	uint8_t Count;                          // 滤波器计数器
	uint8_t LongCount;                      // 长按计数器
	uint8_t LongTime;                       // 按键按下持续时间, 0表示不检测长按
	uint8_t State;                          // 按键当前状态（按下还是弹起）
	uint8_t RepeatSpeed;                    // 连续按键周期
	uint8_t RepeatCount;                    // 连续按键计数器
	
	uint8_t (*WhetherKeyPress)(void);       // 判断按键是否按下
}KEY_ValueTypeDef;


// FIFO structure
#define KEY_FIFO_SIZE                       10
typedef struct
{
	uint8_t Buffer[KEY_FIFO_SIZE];           // 键值缓冲区
	uint8_t Read;                            // 缓冲区读指针1
	uint8_t Write;                           // 缓冲区写指针 
	uint8_t Read2;                           // 缓冲区读指针2
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

// 按键按下后的返回值
#define KEY0_PRES 1
#define KEY1_PRES 2
#define KEY2_PRES 3
#define WKUP_PRES 4


//void KEY_Init(void);
unsigned char Key_Query(u8 mode);
#endif
