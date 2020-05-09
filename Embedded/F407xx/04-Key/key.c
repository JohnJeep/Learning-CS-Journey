#include "key.h"

// �����ṹ��
KEY_FIFO_TypeDef KEY_FIFOStruct;
KEY_ValueTypeDef KEY_ValueStruct[KEY_COUNT];


void BSP_KeyInit(void)
{
	BSP_ConfigIinit();
	BSP_KeyValueInit();
}

// �ײ�������ʼ��
void BSP_ConfigIinit(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_xxxPeriphClockCmd(RCC_xxxPeriph_KEY, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;       // ����
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // ����ģʽ
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;       // ������������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // IO������ٶ�
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K1;
	GPIO_Init(GPIO_PORT_K1, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K2;
	GPIO_Init(GPIO_PORT_K2, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_K3;
	GPIO_Init(GPIO_PORT_K3, &GPIO_InitStruct);
}

// ��ֵ��ʼ��
void BSP_KeyValueInit(void)
{
	uint8_t i;
	
	// �԰���FIFO��дָ������
	KEY_FIFOStruct.Read = 0;
	KEY_FIFOStruct.Write = 0;
	KEY_FIFOStruct.Read2 = 0;
	
	// ��ÿ�������ṹ���Ա������һ��ȱʡֵ
	for(i=0; i<KEY_COUNT; i++)
	{
		KEY_ValueStruct[i].LongTime = KEY_LONG_TIME;     // 0 ��ʾ����ⳤ�����¼�
		KEY_ValueStruct[i].Count = KEY_FILTER_TIME / 2;  // ����������Ϊ�˲�ʱ���һ��
		KEY_ValueStruct[i].State = 0;                    // 0��ʾδ����
		KEY_ValueStruct[i].RepeatSpeed = 0;              // �����������ٶȣ�0��ʾ��֧������
		KEY_ValueStruct[i].RepeatCount = 0;              // ����������
	}
	
	// �жϰ����Ƿ���
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
  * @brief  ��1����ֵѹ�밴��FIFO��������������ģ��һ��������
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
  * @brief  �Ӱ���FIFO��������ȡһ����ֵ��
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
  * @brief  �Ӱ���FIFO��������ȡһ����ֵ�������Ķ�ָ�롣
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
  * @brief  ��ȡ������״̬
  * @note   
  * @param  ����ID��KeyID
  *                           
  * @retval 1 ��ʾ���£� 0 ��ʾδ����
  */ 
uint8_t BSP_GetKeyState(KEY_ID_TypeDef KeyID)
{
	return KEY_ValueStruct[KeyID].State;
}

/**
  * @brief  ���ð�������
  * @note   
  * @param  KeyID: ����ID
  *         LongTime: �����¼�ʱ��
  *         RepeatSpeed�������ٶ�
  * @retval None
  */
void BSP_SetKeyParam(uint8_t KeyID, uint16_t LongTime, uint8_t RepeatSpeed)
{
	KEY_ValueStruct[KeyID].LongTime = LongTime;        // ����ʱ�� 0 ��ʾ����ⳤ�����¼�
	KEY_ValueStruct[KeyID].RepeatCount = RepeatSpeed;  // �����������ٶȣ�0��ʾ��֧������ 
	KEY_ValueStruct[KeyID].Count = 0;                  // ����������
}

/**
  * @brief  ��հ���FIFO������
  * @note   
  * @param  None
  *
  * @retval None
  */
void BSP_ClearKey(void)
{

}

/**
  * @brief  ���һ��������������״̬�����뱻�����Եĵ��á�
  * @note   
  * @param  KeyID: �ĸ�����
  *
  * @retval None
  */
void BSP_DetectKey(uint8_t KeyID)
{
	KEY_ValueTypeDef  *pkey;                // ����ָ��ķ�ʽ�԰���ֵ�ṹ��Ĳ���
	
	pkey = &KEY_ValueStruct[KeyID];         // ��ȡ�����ṹ��ĵ�ַ
	if(pkey->WhetherKeyPress())          // �������´���
	{
		if(pkey->Count < KEY_FILTER_TIME)
		{
			pkey->Count = KEY_FILTER_TIME;
		}
		else if(pkey->Count < (2*KEY_FILTER_TIME))
		{
			pkey->Count++;                    // ʵ��KEY_FILTER_TIMEʱ�䳤�ȵ��ӳ�
		}
		else
		{
			// �������º���ֵ����Ϊ1��δ���£�ֵһֱΪ 0
			if(pkey->State == 0)
			{	
				pkey->State = 1;
				BSP_PutKey((uint8_t)(3*KeyID + 1));
			}
			
			if(pkey->LongTime > 0)  // ���������
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
				if(pkey->RepeatSpeed > 0)  //  ������������
				{
					if(++pkey->RepeatCount >= pkey->RepeatSpeed)
					{
						pkey->RepeatCount = 0;
						BSP_PutKey((uint8_t)(3*KeyID + 1));   // ������ÿ��10ms����1������
					}
				}
			}
		}	
	}
	else       // ����δ���»��߰����ɿ�����
	{
		if(pkey->Count > KEY_FILTER_TIME)
		{
			// ���ڰ����˲�ǰ��Count����һ����ֵ
			pkey->Count = KEY_FILTER_TIME;
		}
		else if(pkey->Count != 0)
		{
			pkey->Count--;
		}
		else
		{
			if(pkey->State == 1)  // �����ɿ�
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
  * @brief  ɨ�����а���������������systick�ж��������Եı�����
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
	static char key_up = 1;  //�������ɿ���־
	if(mode)
		key_up = 1;   // ֧����������
	if(key_up &&(KEY0 == 0 || KEY1 == 0 || KEY2 == 0))   // �����͵�ƽ��Ч���ж��Ƿ��а�������
	{
		//delay_ms(10);  //������
		key_up = 0;
		if(KEY0 == 0)   // ���� 1 ���� 
			return 1;
		else if(KEY1 == 0)
			return 2;
		else if(KEY2 == 0)
			return 3;
	}
	else if((KEY0 == 1) && (KEY1 == 1) && (KEY2 == 1))  // ���°�����û���ɿ�
		key_up = 1;
	return 0;   // û�а�������
}
#endif
