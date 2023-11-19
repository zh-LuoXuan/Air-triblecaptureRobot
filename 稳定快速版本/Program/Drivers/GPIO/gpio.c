#include "gpio.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	 

void GPIO_Init_Configuration(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
/*******************************************使能时钟**********************************************/  
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
  
 /*******************************************电磁阀**********************************************/ 
      GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOE, &GPIO_InitStruct);
			
 /*****************************************************************************************/ 
}

