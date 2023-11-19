#include "start_task.h"
#include "sys.h"
#include "delay.h"
#include "gpio.h" 
#include "IMUTask.h"
#include "pwm.h"

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
#include "can.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "chassis_task.h"
#include "control_task.h"
#include "Time7.h"

#define START_TASK_PRIO		31
#define STAET_STK_SIZE 		152
TaskHandle_t Start_Task_Handler;
void start_task(void *pvParameters);


void start_task_Create(void)
{
    xTaskCreate((TaskFunction_t )start_task,
                (const char*    )"start_task",
                (uint16_t       )STAET_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )START_TASK_PRIO,
                (TaskHandle_t*  )&Start_Task_Handler);
    vTaskStartScheduler();
}


void System_Init(void)
{
	delay_init(168);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	remote_control_init();
	GPIO_Init_Configuration();
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);//1Mbps
  CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_4tq, CAN_BS1_9tq, 3, CAN_Mode_Normal);
	TIM7_BAS_Init(10-1, 8400-1);

	delay_ms(500);
}   


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	  task_Chsssis_Create();
		task_control_Create();
	
    vTaskDelete(Start_Task_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

