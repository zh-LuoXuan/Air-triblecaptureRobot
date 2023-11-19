#include "send_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "gpio.h" 

/*==============================================================*/
#define SEND_TASK_PRIO 22
#define SEND_STK_SIZE 512
TaskHandle_t SendTask_Handler;
void send_task(void *pvParameters);

/*==============================================================*/

void task_Send_Create(void)
{
	xTaskCreate((TaskFunction_t)send_task,
                (const char *)"send_task",
                (uint16_t)SEND_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SEND_TASK_PRIO,
                (TaskHandle_t *)&SendTask_Handler);
}

void send_task(void *pvParameters)
{
	u8 KEY;
	while(1)
	{
		KEY = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
		CAN_CMD_RC(&rc_ctrl);
		CAN_CMD_KEYMOUSE(&rc_ctrl);
		CAN_CMD_IO(KEY);
		vTaskDelay(1);
	}
}

