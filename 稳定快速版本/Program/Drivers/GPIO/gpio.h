#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"

#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////	 
			  
////////////////////////////////////////////////////////////////////////////////// 	

//引脚定义
/*******************************************************/

#define LED1_PIN                  GPIO_Pin_10

#define LED2_PIN                  GPIO_Pin_11

#define LED3_PIN                  GPIO_Pin_12

#define LED4_PIN                  GPIO_Pin_2

/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  0
#define OFF 1

#define KEY0 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//设置为高电平
#define digitalLo(p,i)			 {p->BSRRH=i;}		//输出低电平
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//输出反转状态


/* 带参宏，可以像内联函数一样使用 */

/*================== LED ====================*/

#define LED1(a)	  \
          if (a)	\
					GPIO_SetBits(GPIOC , LED1_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED1_PIN)

#define LED2(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOC , LED2_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED2_PIN)

#define LED3(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOC , LED3_PIN);\
					else		\
					GPIO_ResetBits(GPIOC , LED3_PIN)

#define LED4(a)	  \
					if (a)	\
					GPIO_SetBits(GPIOD , LED4_PIN);\
					else		\
					GPIO_ResetBits(GPIOD , LED4_PIN)

/* 定义控制IO的宏 */

#define LED1_TOGGLE			GPIO_ToggleBits(GPIOC , LED1_PIN)
#define LED2_TOGGLE			GPIO_ToggleBits(GPIOC , LED2_PIN)
#define LED3_TOGGLE			GPIO_ToggleBits(GPIOC , LED3_PIN)
#define LED4_TOGGLE			GPIO_ToggleBits(GPIOD , LED4_PIN)


#define LED_ALL_OFF	\
					LED1(OFF);\
					LED2(OFF);\
					LED3(OFF);\
          LED4(OFF);

#define LED_ALL_TOGGLE	\
					LED1_TOGGLE;\
					LED2_TOGGLE;\
					LED3_TOGGLE;\
          LED4_TOGGLE;

#define LED_ALL_ON	\
					LED1(ON);\
					LED2(ON);\
					LED3(ON);\
          LED4(ON);

/*==========================*/

//电磁阀
#define BULLET_STOREHOUSE_ON 	 			GPIO_SetBits(GPIOE,GPIO_Pin_11)         //弹丸阀门打开
#define BULLET_STOREHOUSE_OFF	 			GPIO_ResetBits(GPIOE,GPIO_Pin_11)

#define STICK_ON  									GPIO_SetBits(GPIOE,GPIO_Pin_8)         //夹手推出..
#define STICK_OFF 									GPIO_ResetBits(GPIOE,GPIO_Pin_8)

#define TURN_ON  										GPIO_SetBits(GPIOE,GPIO_Pin_13)         //旋转夹手..
#define TURN_OFF 										GPIO_ResetBits(GPIOE,GPIO_Pin_13)

#define CLAMP_ON  									GPIO_SetBits(GPIOE,GPIO_Pin_9)         //夹手开关..
#define CLAMP_OFF 									GPIO_ResetBits(GPIOE,GPIO_Pin_9)

#define BULLET_BOX_ON 							GPIO_SetBits(GPIOE,GPIO_Pin_12)         //弹药箱弹出
#define BULLET_BOX_OFF 							GPIO_ResetBits(GPIOE,GPIO_Pin_12)
  
#define INSTITUTIONAL_ON  					GPIO_SetBits(GPIOE,GPIO_Pin_10)         //整体抬升
#define INSTITUTIONAL_OFF 					GPIO_ResetBits(GPIOE,GPIO_Pin_10)

//翻转电平
#define BULLET_STOREHOUSE_TOGGLE		GPIO_ToggleBits(GPIOE,GPIO_Pin_11)
#define STICK_TOGGLE								GPIO_ToggleBits(GPIOE,GPIO_Pin_8)
#define TURN_TOGGLE									GPIO_ToggleBits(GPIOE,GPIO_Pin_13)
#define CLAMP_TOGGLE								GPIO_ToggleBits(GPIOE,GPIO_Pin_9)
#define BULLET_BOX_TOGGLE						GPIO_ToggleBits(GPIOE,GPIO_Pin_12)
#define INSTITUTIONAL_TOGGLE				GPIO_ToggleBits(GPIOE,GPIO_Pin_10)

//气路关闭
#define ALLAIR_OFF  								\
				{    			  								\
				  STICK_OFF;								\
					BULLET_STOREHOUSE_OFF;		\
					TURN_OFF;									\
					CLAMP_OFF;								\
					BULLET_BOX_OFF;						\
					INSTITUTIONAL_OFF;				\
				}


typedef enum
{
	Reset_unfinished = 0,
	Reset_Finish = 1,
}System_Reset_State;

typedef enum
{
	Sensor_NotTrigger = 0,
	Sensor_Trigger = 1,
}Sensor_State;

void Key_Init(void);
void Led_Init(void);
void GPIO_Init_Configuration(void);
#endif
