#include "chassis_task.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"
#include "control_task.h"
/*==============================================================*/
#define CHASSIS_TASK_PRIO 23
#define CHASSIS_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;
void Chassis_task(void *pvParameters);

/*==============================================================*/

void task_Chsssis_Create(void)
{
	xTaskCreate((TaskFunction_t)Chassis_task,
                (const char *)"Chassisl_task",
                (uint16_t)CHASSIS_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CHASSIS_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
}
/*==============================================================*/

Chassis_Control_t chassis_control;
fp32 chassis_power_spd_pid[3] = {CHASSIS_SPEED_PID_KP , CHASSIS_SPEED_PID_KI , CHASSIS_SPEED_PID_KD};
#if ISGIMBALNULL
	fp32 chassis_follow_pid[3] = {CHASSIS_FOLLOW_ANGLE_PID_KP , CHASSIS_FOLLOW_ANGLE_PID_KI , CHASSIS_FOLLOW_ANGLE_PID_KD};
#endif
extern topControl_t top_Control;
static void mecanum_calc(fp32 vx, fp32 vy, fp32 vz, Chassis_Control_t *chassis_mec);
static void Chassis_Motor_Control(Chassis_Control_t *chassis_motor_control);
static void Chassis_Mode_Set(Chassis_Control_t *chassis_move_control);
static void Mecanum_ref_set(Chassis_Control_t *mec_ref);
static void Chassis_Init(Chassis_Control_t *chassis_init);
static void Chassis_fdb_set(Chassis_Control_t *chassis_fdb);


/**
  * @brief  底盘任务
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_task(void *pvParameters)
{
	Chassis_Init(&chassis_control);
	while(1)
	{
		Chassis_Mode_Set(&chassis_control);
		Chassis_fdb_set(&chassis_control); 
		Mecanum_ref_set(&chassis_control);
		Chassis_Motor_Control(&chassis_control);
		PID_Calc(&top_Control.topAnglePID, top_Control.top_motorPoint->real_ecd, top_Control.topAglRef);
		PID_Calc(&top_Control.topSpeedPID, top_Control.top_motorPoint->speed_rpm, top_Control.topAnglePID.out);
		if(top_Control.topMode == ALLSTOP)
		{
			CAN1_CMD_TOP(0);
		}
		else
		CAN1_CMD_TOP(top_Control.topSpeedPID.out);
		vTaskDelay(1);  //系统延时
	}
}


/**
  * @brief  底盘初始化
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Chassis_Init(Chassis_Control_t *chassis_init)
{
	uint8_t i = 0;
	if (chassis_init == NULL)
    {
        return;
    }
  
	chassis_init->chassis_rc_ctrl = get_remote_control_point();
	chassis_init->chassis_vx = 0;
	chassis_init->chassis_vy = 0;
	chassis_init->chassis_vw = 0;
	
	for(i = 0; i < 4; i++)
  {			
		PID_Init(&chassis_init->chassis_motor[i].chassis_speed_pid , PID_POSITION , chassis_power_spd_pid , CHASSIS_SPEED_PID_MAX_OUT , CHASSIS_SPEED_PID_MAX_IOUT);
		chassis_init->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		chassis_init->chassis_motor[i].SpeedRef = 0;
	}
	
	chassis_init->control_mode = ALL_STOP;
	#if ISGIMBALNULL
	PID_Init(&chassis_init->follow_gimbal_pid , PID_POSITION , chassis_follow_pid , CHASSIS_FOLLOW_ANGLE_PID_MAX_OUT , CHASSIS_FOLLOW_ANGLE_PID_MAX_IOUT);	
	#endif
}

/**
  * @brief  麦克纳姆参数更新
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Mecanum_ref_set(Chassis_Control_t *mec_ref)
{
	if(mec_ref == NULL)
	{
		return ;
	}
	switch((uint16_t)mec_ref->control_mode)
	{
		case ALL_STOP: 
		case REMOTE_CONTROL: 
		{
			mec_ref->chassis_vw = Constrain_float(fp32_deadline(RC_Control_CH2, -10, 10) * RC_VW_RESPONSE, -CHASSIS_RC_MAX_SPEED_W, CHASSIS_RC_MAX_SPEED_W);
			mec_ref->chassis_vx = Constrain_float(fp32_deadline(RC_Control_CH1, -10, 10) * RC_VX_RESPONSE, -CHASSIS_RC_MAX_SPEED_X, CHASSIS_RC_MAX_SPEED_X);
			mec_ref->chassis_vy = Constrain_float(fp32_deadline(RC_Control_CH0, -10, 10) * RC_VY_RESPONSE, -CHASSIS_RC_MAX_SPEED_Y, CHASSIS_RC_MAX_SPEED_Y);
		}	
	}
}


/**
  * @brief  底盘反馈更新
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Chassis_fdb_set(Chassis_Control_t *chassis_fdb)
{
	if(chassis_fdb == NULL)
	{
		return ;
	}
	static uint16_t i = 0;
	switch((uint16_t)chassis_fdb->control_mode)
	{
		case ALL_STOP: 
		case REMOTE_CONTROL: 
		{
			for(i = 0; i < 4; i++)
			{
				chassis_fdb->chassis_motor[i].SpeedFdb = chassis_fdb->chassis_motor[i].chassis_motor_measure->speed_rpm;
			}
			break;
		}	
		default:
			break;
	}
}


/**
  * @brief  底盘模式设置
  * @param  底盘数据结构体指针
  * @retval void
  */
static void Chassis_Mode_Set(Chassis_Control_t *chassis_move_control)
{
	if(chassis_move_control == NULL)
	{
		return ;
	}
	if(switch_is_down(chassis_move_control->chassis_rc_ctrl->rc.s[ModeChannel_R]))
	{
		chassis_move_control->control_mode = ALL_STOP;
	}
	else
	{
		chassis_move_control->control_mode = REMOTE_CONTROL;
	}
}


/**
  * @brief  电机输出
  * @param  底盘数据结构体
  * @retval void
  * @attention  
  */
static void Chassis_Motor_Control(Chassis_Control_t *chassis_motor_control)
{
	if (chassis_motor_control == NULL)
	{
			return;
	}
	static uint16_t i = 0;
	switch((uint16_t)chassis_motor_control->control_mode)
	{
		case ALL_STOP:
		{
			CAN1_CMD_CHASSIS(0, 0, 0, 0);
			break;
		}
		case REMOTE_CONTROL: 
		{
			mecanum_calc(chassis_motor_control->chassis_vx,chassis_motor_control->chassis_vy,chassis_motor_control->chassis_vw,chassis_motor_control);
			for(i = 0; i < 4; i++)
			{
				chassis_motor_control->chassis_motor[i].GiveCurrent = PID_Calc(&chassis_motor_control->chassis_motor[i].chassis_speed_pid, 
																																				chassis_motor_control->chassis_motor[i].SpeedFdb, 
																																				chassis_motor_control->chassis_motor[i].SpeedRef);
			}	
			CAN1_CMD_CHASSIS(chassis_motor_control->chassis_motor[0].GiveCurrent,
											 chassis_motor_control->chassis_motor[1].GiveCurrent,
											 chassis_motor_control->chassis_motor[2].GiveCurrent,
											 chassis_motor_control->chassis_motor[3].GiveCurrent);
			break; 
		}	
		default:
			break;
	}
	vTaskDelay(1);
}

static void mecanum_calc(fp32 vx, fp32 vy, fp32 vz, Chassis_Control_t *chassis_mec)
{
	if(chassis_mec == NULL)
	{
		return;
	}
	  static fp32 wheel_rpm[4] = {0.0f};
		static uint16_t i = 0;
    static float rotate_ratio_fr;//前右
    static float rotate_ratio_fl;//前左
    static float rotate_ratio_bl;//后左
    static float rotate_ratio_br;//后右
    static float wheel_rpm_ratio;

#if ISGIMBALNULL 
		
    rotate_ratio_fr 	= ((WHEELBASE + WHEELTRACK) / 2.0f - chassis_mec->rotate_x_offset + chassis_mec->rotate_y_offset) / RADIAN_COEF; //6.63
    rotate_ratio_fl 	= ((WHEELBASE + WHEELTRACK) / 2.0f - chassis_mec->rotate_x_offset - chassis_mec->rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis_mec->rotate_x_offset - chassis_mec->rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + chassis_mec->rotate_x_offset + chassis_mec->rotate_y_offset) / RADIAN_COEF;
#else
		
		rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    rotate_ratio_fl = rotate_ratio_fr;
    rotate_ratio_bl = rotate_ratio_fr;
    rotate_ratio_br = rotate_ratio_fr;
#endif
		
		
    wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO); //25.157

    wheel_rpm[0] = (-vx + vy + vz * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[1] = (+vx + vy + vz * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[2] = (+vx - vy + vz * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[3] = (-vx - vy + vz * rotate_ratio_br) * wheel_rpm_ratio;
		
		for(i = 0; i < 4; i++)
		{
			memcpy(&chassis_mec->chassis_motor[i].SpeedRef, &wheel_rpm[i], sizeof(float));
		}
}



