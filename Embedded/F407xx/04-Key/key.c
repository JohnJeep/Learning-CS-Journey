#include "key.h"

// 声明结构体
KEY_FIFO_TypeDef KEY_FIFOStruct;
KEY_ValueTypeDef KEY_ValueStruct[KEY_COUNT];


void BSP_KeyInit(void)
{
	BSP_ConfigIinit();
	BSP_KeyValueInit();
}

// 底层驱动初始化
void BSP_ConfigIinit(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_xxxPeriphClockCmd(RCC_xxxPeriph_KEY, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;       // 输入
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // 推挽模式
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;       // 无需上拉电阻
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // IO口最大速度
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K1;
	GPIO_Init(GPIO_PORT_K1, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K2;
	GPIO_Init(GPIO_PORT_K2, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K3;
	GPIO_Init(GPIO_PORT_K3, &GPIO_InitStruct);
}

// 键值初始化
void BSP_KeyValueInit(void)
{
	uint8_t i;
	
	// 对按键FIFO读写指针清零
	KEY_FIFOStruct.Read = 0;
	KEY_FIFOStruct.Write = 0;
	KEY_FIFOStruct.Read2 = 0;
	
	// 给每个按键结构体成员变量赋一组缺省值
	for(i=0; i<KEY_COUNT; i++)
	{
		KEY_ValueStruct[i].LongTime = KEY_LONG_TIME;     // 0 表示不检测长按键事件
		KEY_ValueStruct[i].Count = KEY_FILTER_TIME / 2;  // 计数器设置为滤波时间的一半
		KEY_ValueStruct[i].State = 0;                    // 0表示未按下
		KEY_ValueStruct[i].RepeatSpeed = 0;              // 按键连发的速度，0表示不支持连发
		KEY_ValueStruct[i].RepeatCount = 0;              // 连发计数器
	}
	
	// 判断按键是否按下
	KEY_ValueStruct[0].WhetherKeyPress = WhetherKeyPress1;
	KEY_ValueStruct[1].WhetherKeyPress = WhetherKeyPress2;
	KEY_ValueStruct[2].WhetherKeyPress = WhetherKeyPress3;
	

}


static uint8_t WhetherKeyPress1(void) 
{
	if ((GPIO_PORT_K1->IDR & GPIO_PIN_K1) == 0)
		return 1;
	else 
		return 0;
}

static uint8_t WhetherKeyPress2(void) 
{	
	if ((GPIO_PORT_K2->IDR & GPIO_PIN_K2) == 0) 
		return 1;
	else 
		return 0;
}
static uint8_t WhetherKeyPress3(void) 
{
	if ((GPIO_PORT_K3->IDR & GPIO_PIN_K3) == 0) 
		return 1;
	else 
		return 0;
}

/**
  * @brief  将1个键值压入按键FIFO缓冲区。可用于模拟一个按键。
  * @note   
  * @param  KeyCode
  *                           
  * @retval None
  */ 
void BSP_PutKey(uint8_t KeyCode)
{
	KEY_FIFOStruct.Buffer[KEY_FIFOStruct.Write] = KeyCode;
	if(++KEY_FIFOStruct.Write >= KEY_FIFO_SIZE)
	{	
		KEY_FIFOStruct.Write = 0;
	}
}

/**
  * @brief  从按键FIFO缓冲区读取一个键值。
  * @note   
  * @param  None
  *                           
  * @retval None
  */ 
uint8_t BSP_GetKey(void)
{
	uint8_t ret;
	
	if(KEY_FIFOStruct.Read == KEY_FIFOStruct.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = KEY_FIFOStruct.Buffer[KEY_FIFOStruct.Read];
		if(++KEY_FIFOStruct.Read >= KEY_FIFO_SIZE)
		{
			KEY_FIFOStruct.Read = 0;
		}
		return ret;
	}
}


/**
  * @brief  从按键FIFO缓冲区读取一个键值。独立的读指针。
  * @note   
  * @param  None
  *                           
  * @retval None
  */ 
uint8_t BSP_GetKey2(void)
{
	uint8_t ret;
	
	if(KEY_FIFOStruct.Read2 == KEY_FIFOStruct.Write)
	{
		return KEY_NONE;
	}
	else
	{
		ret = KEY_FIFOStruct.Buffer[KEY_FIFOStruct.Read2];
		if(++KEY_FIFOStruct.Read2 >= KEY_FIFO_SIZE)
		{
			KEY_FIFOStruct.Read2 = 0;
		}
		return ret;
	}
}

/**
  * @brief  读取按键的状态
  * @note   
  * @param  按键ID：KeyID
  *                           
  * @retval 1 表示按下， 0 表示未按下
  */ 
uint8_t BSP_GetKeyState(KEY_ID_TypeDef KeyID)
{
	return KEY_ValueStruct[KeyID].State;
}

/**
  * @brief  设置按键参数
  * @note   
  * @param  KeyID: 按键ID
  *         LongTime: 长按事件时间
  *         RepeatSpeed：连发速度
  * @retval None
  */
void BSP_SetKeyParam(uint8_t KeyID, uint16_t LongTime, uint8_t RepeatSpeed)
{
	KEY_ValueStruct[KeyID].LongTime = LongTime;        // 长按时间 0 表示不检测长按键事件
	KEY_ValueStruct[KeyID].RepeatCount = RepeatSpeed;  // 按键连发的速度，0表示不支持连发 
	KEY_ValueStruct[KeyID].Count = 0;                  // 连发计数器
}

/**
  * @brief  清空按键FIFO缓冲区
  * @note   
  * @param  None
  *
  * @retval None
  */
void BSP_ClearKey(void)
{

}

/**
  * @brief  检测一个按键。非阻塞状态，必须被周期性的调用。
  * @note   
  * @param  KeyID: 哪个按键
  *
  * @retval None
  */
void BSP_DetectKey(uint8_t KeyID)
{
	KEY_ValueTypeDef  *pkey;                // 采用指针的方式对按键值结构体的操作
	
	pkey = &KEY_ValueStruct[KeyID];         // 读取按键结构体的地址
	if(pkey->WhetherKeyPress())          // 按键按下处理
	{
		if(pkey->Count < KEY_FILTER_TIME)
		{
			pkey->Count = KEY_FILTER_TIME;
		}
		else if(pkey->Count < (2*KEY_FILTER_TIME))
		{
			pkey->Count++;                    // 实现KEY_FILTER_TIME时间长度的延迟
		}
		else
		{
			// 按键按下后，其值设置为1；未按下，值一直为 0
			if(pkey->State == 0)
			{	
				pkey->State = 1;
				BSP_PutKey((uint8_t)(3*KeyID + 1));
			}
			
			if(pkey->LongTime > 0)  // 如果长按下
			{
				if(pkey->LongCount < pkey->LongTime)
				{	
					if(++pkey->LongCount == pkey->LongTime)
					{
						BSP_PutKey((uint8_t)(3*KeyID + 1));				
					}		
				}
			}
			else    
			{
				if(pkey->RepeatSpeed > 0)  //  连续按键周期
				{
					if(++pkey->RepeatCount >= pkey->RepeatSpeed)
					{
						pkey->RepeatCount = 0;
						BSP_PutKey((uint8_t)(3*KeyID + 1));   // 长按后，每隔10ms发送1个按键
					}
				}
			}
		}	
	}
	else       // 按键未按下或者按键松开处理
	{
		if(pkey->Count > KEY_FILTER_TIME)
		{
			// 用于按键滤波前给Count设置一个初值
			pkey->Count = KEY_FILTER_TIME;
		}
		else if(pkey->Count != 0)
		{
			pkey->Count--;
		}
		else
		{
			if(pkey->State == 1)  // 按键松开
			{
				pkey->State = 0;
				BSP_PutKey((uint8_t)(3*KeyID + 2));
			}
		}
		pkey->LongCount = 0;
		pkey->RepeatCount = 0;
	}
}

/**
  * @brief  扫描所有按键。非阻塞，在systick中断中周期性的被调用
  * @note   
  * @param  None
  *
  * @retval None
  */
void BSP_KeyScan(void)
{
	uint8_t i;
	
	for(i=0; i<KEY_COUNT; i++)
	{
		BSP_DetectKey(i);
	}
}


#if 1
unsigned char Key_Query(u8 mode)
{
	static char key_up = 1;  //按键按松开标志
	if(mode)
		key_up = 1;   // 支持连续按下
	if(key_up &&(KEY0 == 0 || KEY1 == 0 || KEY2 == 0))   // 按键低电平有效，判断是否有按键按下
	{
		//delay_ms(10);  //消抖动
		key_up = 0;
		if(KEY0 == 0)   // 按键 1 按下 
			return 1;
		else if(KEY1 == 0)
			return 2;
		else if(KEY2 == 0)
			return 3;
	}
	else if((KEY0 == 1) && (KEY1 == 1) && (KEY2 == 1))  // 按下按键后没有松开
		key_up = 1;
	return 0;   // 没有按键按下
}
#endif
