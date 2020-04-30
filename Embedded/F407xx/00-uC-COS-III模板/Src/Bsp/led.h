#ifndef __LED__H
#define __LED__H

#include "stm32f4xx.h"

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			           {p->BSRRL=i;}		// ����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			           {p->BSRRH=i;}		// ����͵�ƽ
#define digitalToggle(p,i)	 	           {p->ODR ^=i;}		// �����ת״̬

/* �������IO�ĺ� */
#define LED0_TOGGLE		digitalToggle(GPIOG,GPIO_Pin_13)
#define LED0_OFF			digitalHi(GPIOG,GPIO_Pin_13)
#define LED0_ON				digitalLo(GPIOG,GPIO_Pin_13)

#define LED1_TOGGLE		digitalToggle(GPIOG,GPIO_Pin_14)
#define LED1_OFF			digitalHi(GPIOG,GPIO_Pin_14)
#define LED1_ON				digitalLo(GPIOG,GPIO_Pin_14)

#define LED2_TOGGLE		digitalToggle(GPIOG,GPIO_Pin_15)
#define LED2_OFF			digitalHi(GPIOG,GPIO_Pin_15)
#define LED2_ON				digitalLo(GPIOG,GPIO_Pin_15)

void LED_Init(void);



#if 0
//���Ŷ���
/*******************************************************/
//R ��ɫ��
#define LED1_PIN                  GPIO_Pin_13                 
#define LED1_GPIO_PORT            GPIOG                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOG

//#define LED1_PIN                  GPIO_Pin_6                 
//#define LED1_GPIO_PORT            GPIOF                      
//#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOF


//G ��ɫ��
#define LED2_PIN                  GPIO_Pin_7                 
#define LED2_GPIO_PORT            GPIOF                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOF

//B ��ɫ��
#define LED3_PIN                  GPIO_Pin_8                 
#define LED3_GPIO_PORT            GPIOF                       
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOF
/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 	 {p->ODR ^=i;}		//�����ת״̬

/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF			digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF			digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF			digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON				digitalLo(LED3_GPIO_PORT,LED3_PIN)

void LED_GPIO_Config(void);

#endif 



#endif

