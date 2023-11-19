#include "CAN_Receive.h"

#include "stm32f4xx.h"
#include "rng.h"
#include "gpio.h" 

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Detect_Task.h"
#include "RemoteControl.h"


//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)  \
		{                                       \
				if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ; \
				else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ; \
				(ptr)->last_ecd = (ptr)->ecd; \
				(ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
				(ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 |(rx_message)->Data[3]); \
				(ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
				(ptr)->temperate = (rx_message)->Data[6];  \
				(ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd; 	\
				(ptr)->real_ecd=(ptr)->all_ecd/ECD_RATE;   \
		}
		
  
 motor_measure_t motor_chassis[4],
								 motor_top;
/**********************************************************************************
返回电机变量，通过指针传递方式传递信息
***********************************************************************************/

//返回横移电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Top_Motor_Measure_Point(void)
{
    return &motor_top;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[i];
}


//统一处理can接收函数
static void CAN_hook1(CanRxMsg *rx_message);
//统一处理can接收函数
static void CAN_hook2(CanRxMsg *rx_message);


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif
//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook1(&rx1_message);
    }
}

//can2中断
void CAN2_RX1_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP1) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
        CAN_Receive(CAN2, CAN_FIFO1, &rx2_message);
        CAN_hook2(&rx2_message);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{

    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_M3508_SET_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    CAN_Transmit(CAN1, &TxMessage);
}


/**
  * @brief  CAN1发送电机控制命令
  * @param  PID输出值
  * @retval void
  */
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_CHASSIS_ALL_ID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data1>>8;
		TxMessage.Data[1] = data1;
		TxMessage.Data[2] = data2>>8;;
		TxMessage.Data[3] = data2;
		TxMessage.Data[4] = data3>>8;;
		TxMessage.Data[5] = data3;
		TxMessage.Data[6] = data4>>8;;
		TxMessage.Data[7] = data4;

		CAN_Transmit(CAN1 , &TxMessage);
}
/**
  * @brief  CAN1发送电机控制命令
  * @param  PID输出值
  * @retval void
  */
void CAN1_CMD_TOP(int16_t data)
{
		CanTxMsg TxMessage;
		TxMessage.StdId = CAN_TOP_ID;
		TxMessage.IDE = CAN_ID_STD;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 0x08;
		TxMessage.Data[0] = data>>8;
		TxMessage.Data[1] = data;
		TxMessage.Data[2] = 0;
		TxMessage.Data[3] = 0;
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;
		
		 
		CAN_Transmit(CAN1 , &TxMessage);

}


//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook1(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_CHASSIS_M1_ID:
			case CAN_CHASSIS_M2_ID:
			case CAN_CHASSIS_M3_ID:
			case CAN_CHASSIS_M4_ID:
				{
					static uint8_t i = 0;
					//处理电机ID号
					i = rx_message->StdId - CAN_CHASSIS_M1_ID;
					//处理电机数据宏函数
					get_motor_measure(&motor_chassis[i], rx_message);
					//记录时间
					DetectHook(YawChassisMotorTOE);
					break;
				}
			case CAN_TOP_M1_ID:
				{
					//处理电机数据宏函数
					get_motor_measure(&motor_top, rx_message);
					//记录时间
					DetectHook(YawChassisMotorTOE);
					break;
				}
			default:
			{
				break;
			}
    }
}

static void CAN_hook2(CanRxMsg *rx_message)
{
 
}

