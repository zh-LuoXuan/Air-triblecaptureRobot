#ifndef _USART1_H_
#define _USART1_H_

#include "stm32f4xx.h"                  // Device header
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "FreeRTOSConfig.h"


extern QueueHandle_t TxCOM1;
extern QueueHandle_t RxCOM1;

void Device_Usart1_ENABLE_Init(u32 bound);



#endif

