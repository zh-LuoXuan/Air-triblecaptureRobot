#include "gimbal_task.h"
#include "sys.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "kalman.h"
#include "kalman_filter.h"

#include "pwm.h"

/*==============================================================*/
#define GIMBAL_TASK_PRIO 25
#define GIMBAL_STK_SIZE 512
TaskHandle_t GimbalTask_Handler;
void Gimbal_task(void *pvParameters);

/*==============================================================*/

void task_Gimbal_Create(void)
{
	xTaskCreate((TaskFunction_t)Gimbal_task,
                (const char *)"Gimbal_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GimbalTask_Handler);
}

/*==============================================================*/
Drive_mode_e DRIVE_MODE = NORMAL;

Gimbal_Control_t gimbal_control;

fp32 gimbal_pitch_age_pid[3] = {GIMBAL_PITCH_ANGLE_PID_KP , GIMBAL_PITCH_ANGLE_PID_KI , GIMBAL_PITCH_ANGLE_PID_KD};
fp32 gimbal_pitch_spd_pid[3] = {GIMBAL_PITCH_SPEED_PID_KP , GIMBAL_PITCH_SPEED_PID_KI , GIMBAL_PITCH_SPEED_PID_KD};

fp32 gimbal_yaw_age_pid[3] = {GIMBAL_YAW_ANGLE_PID_KP , GIMBAL_YAW_ANGLE_PID_KI , GIMBAL_YAW_ANGLE_PID_KD};
fp32 gimbal_yaw_spd_pid[3] = {GIMBAL_YAW_SPEED_PID_KP , GIMBAL_YAW_SPEED_PID_KI , GIMBAL_YAW_SPEED_PID_KD};

int16_t pitch_out;
int16_t yaw_out;

extern remote_mode_e INPUTMOD;
extern Chassis_mode_e CHASSIS_MODE;
/*==============================================================*/
/**
  * @brief  云台数据初始化
  * @param  云台数据结构体
  * @retval void
  * @attention  
  */
static void Gimbal_Init(Gimbal_Control_t *gimbal_init)
{
	if(gimbal_init == NULL)
	{
		return;
	}
	//遥控器数据指针获取
  gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//陀螺仪数据指针获取
	gimbal_init->gimbal_angle_gyro_point = get_Gyro_Angle_Point();
	//电机数据指针获取
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure		=  get_Yaw_Motor_Measure_Point();
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure	=  get_Pitch_Motor_Measure_Point();
	
	//参数初始化
	gimbal_init->gimbal_pitch_motor.angle_offset = GIMBAL_PITCH_ANGLE_OFFSET;
	gimbal_init->gimbal_yaw_motor.angle_offset = GIMBAL_YAW_ANGLE_OFFSET;
	
	//pitch电机PID初始化
	PID_Init(&gimbal_init->gimbal_pitch_angle_pid, PID_POSITION, gimbal_pitch_age_pid, GIMBAL_PITCH_ANGLE_PID_MAX_OUT, GIMBAL_PITCH_ANGLE_PID_MAX_IOUT);
	PID_Init(&gimbal_init->gimbal_pitch_speed_pid, PID_POSITION, gimbal_pitch_spd_pid, GIMBAL_PITCH_SPEED_PID_MAX_OUT, GIMBAL_PITCH_SPEED_PID_MAX_IOUT);
	//yaw电机PID初始化
	PID_Init(&gimbal_init->gimbal_yaw_angle_pid, PID_POSITION, gimbal_yaw_age_pid, GIMBAL_YAW_ANGLE_PID_MAX_OUT, GIMBAL_YAW_ANGLE_PID_MAX_IOUT);
	PID_Init(&gimbal_init->gimbal_yaw_speed_pid, PID_POSITION, gimbal_yaw_spd_pid, GIMBAL_YAW_SPEED_PID_MAX_OUT, GIMBAL_YAW_SPEED_PID_MAX_IOUT);
}
/**
  * @brief  模式切换状态保存
  * @param  云台数据结构体
  * @retval void
  * @attention  
  */
static void Gimbal_Mode_Change(Gimbal_Control_t *gimbal_mode_change)
{

		if((gimbal_mode_change->Gimbal_now_mode!=gimbal_mode_change->Gimbal_Last_mode) && (gimbal_mode_change->Gimbal_now_mode == REMOTE_INPUT))
		{
				gimbal_mode_change->gimbal_yaw_motor.set_angle = gimbal_mode_change->gimbal_yaw_motor.angle_offset;
				gimbal_mode_change->gimbal_pitch_motor.set_angle=gimbal_mode_change->gimbal_angle_gyro_point->Pitch;
				gimbal_mode_change->Gimbal_Last_mode=gimbal_mode_change->Gimbal_now_mode;
		}
		if((gimbal_mode_change->Gimbal_now_mode!=gimbal_mode_change->Gimbal_Last_mode) && (gimbal_mode_change->Gimbal_now_mode == KEYMOUSE_INPUT))
		{
				gimbal_mode_change->gimbal_yaw_motor.set_angle = gimbal_mode_change->gimbal_angle_gyro_point->YAW;
				gimbal_mode_change->gimbal_pitch_motor.set_angle=gimbal_mode_change->gimbal_angle_gyro_point->Pitch;
				gimbal_mode_change->Gimbal_Last_mode=gimbal_mode_change->Gimbal_now_mode;
		}

}
/**
  * @brief  云台模式设置
  * @param  云台数据结构体，遥控器数据指针
  * @retval void
  * @attention  
  */
static void Gimbal_Mode_Set(Gimbal_Control_t *gimbal_mode_set , remote_mode_e *inputmode)
{
	gimbal_mode_set->gimbal_pitch_motor.absolute_angle = gimbal_mode_set->gimbal_angle_gyro_point->Pitch;
	gimbal_mode_set->gimbal_yaw_motor.absolute_angle = gimbal_mode_set->gimbal_angle_gyro_point->YAW;
	
	gimbal_mode_set->gimbal_yaw_motor.relative_angle = (get_relative_pos(gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->real_ecd , gimbal_mode_set->gimbal_yaw_motor.angle_offset));
	
	if(*inputmode == RUN_STOP)
	{
		gimbal_mode_set->Gimbal_now_mode=RUN_STOP;
		
		gimbal_mode_set->gimbal_pitch_motor.set_angle = gimbal_mode_set->gimbal_pitch_motor.angle_offset;
		gimbal_mode_set->gimbal_yaw_motor.set_angle = gimbal_mode_set->gimbal_yaw_motor.angle_offset;
	}
	else if(*inputmode == REMOTE_INPUT)
	{
		gimbal_mode_set->Gimbal_now_mode=REMOTE_INPUT;
		gimbal_mode_set->gimbal_pitch_motor.set_angle -= RC_Control_CH3 * RC_GAMBAL_RESPONSE;
		gimbal_mode_set->gimbal_pitch_motor.set_angle = Constrain_float(gimbal_mode_set->gimbal_pitch_motor.set_angle, GIMBAL_PITCH_ANGLE_MIN, GIMBAL_PITCH_ANGLE_MAX);
		
	}
	else if(*inputmode == KEYMOUSE_INPUT)
	{
		static char V_FLAG;
		static char MODE_COUNT = 0;
		gimbal_mode_set->Gimbal_now_mode = KEYMOUSE_INPUT;
		if(!IF_KEY_PRESSED_V)
		{
			V_FLAG = 1;
		}
		if((IF_KEY_PRESSED_V)&&(V_FLAG == 1))
		{
			V_FLAG = 0;
			MODE_COUNT ++;
			
			if(MODE_COUNT % 2 == 1)
			{
				gimbal_mode_set->gimbal_yaw_motor.angle_offset += 180.0f;
				gimbal_mode_set->gimbal_yaw_motor.set_angle = gimbal_mode_set->gimbal_angle_gyro_point->YAW+180.0f;
				DRIVE_MODE = TURN_BACK;
			}
			if(MODE_COUNT % 2 == 0)
			{
				gimbal_mode_set->gimbal_yaw_motor.angle_offset -= 180.0f;
				gimbal_mode_set->gimbal_yaw_motor.set_angle = gimbal_mode_set->gimbal_angle_gyro_point->YAW-170.0f;
				DRIVE_MODE = NORMAL;
			}
			
			
			
		}
		
		gimbal_mode_set->gimbal_yaw_motor.set_angle += Constrain_float((gimbal_mode_set->gimbal_rc_ctrl->mouse.x * RC_GAMBAL_RESPONSE), GIMBAL_YAW_ANGLE_MIN, GIMBAL_YAW_ANGLE_MAX);
	  
		gimbal_mode_set->gimbal_pitch_motor.set_angle += Constrain_float((gimbal_mode_set->gimbal_rc_ctrl->mouse.y * RC_GAMBAL_RESPONSE), GIMBAL_PITCH_ANGLE_MIN, GIMBAL_PITCH_ANGLE_MAX);
		
		
		}
}
/**
  * @brief  云台电机控制
  * @param  云台数据结构体
  * @retval void
  * @attention  
  */
static void Gimbal_Motor_Control(Gimbal_Control_t *gimbal_motor_control)
{
	if (gimbal_motor_control == NULL)
	{
			return;
	}
	gimbal_motor_control->gimbal_yaw_motor.set_angle = gimbal_motor_control->gimbal_yaw_motor.set_angle;

	gimbal_motor_control->gimbal_yaw_motor.PidFb = gimbal_motor_control->gimbal_angle_gyro_point->YAW;

	gimbal_motor_control->gimbal_yaw_motor.speed_fdb = gimbal_motor_control->gimbal_angle_gyro_point->V_Z;
	
	if((gimbal_motor_control->Gimbal_now_mode == RUN_STOP) || (gimbal_motor_control->Gimbal_now_mode == REMOTE_INPUT) )
	{
		gimbal_motor_control->gimbal_yaw_motor.PidFb = gimbal_motor_control->gimbal_yaw_motor.gimbal_motor_measure->real_ecd;
	}

	Gimbal_Mode_Change(gimbal_motor_control);
	
	PID_Calc(&gimbal_motor_control->gimbal_pitch_angle_pid , gimbal_motor_control->gimbal_pitch_motor.gimbal_motor_measure->real_ecd , gimbal_motor_control->gimbal_pitch_motor.set_angle);
	PID_Calc(&gimbal_motor_control->gimbal_pitch_speed_pid , gimbal_motor_control->gimbal_angle_gyro_point->V_Y , gimbal_motor_control->gimbal_pitch_angle_pid.out);
	
	PID_Calc(&gimbal_motor_control->gimbal_yaw_angle_pid , gimbal_motor_control->gimbal_yaw_motor.PidFb, gimbal_motor_control->gimbal_yaw_motor.set_angle);
	PID_Calc(&gimbal_motor_control->gimbal_yaw_speed_pid , gimbal_motor_control->gimbal_yaw_motor.speed_fdb , gimbal_motor_control->gimbal_yaw_angle_pid.out);
	
	
	gimbal_motor_control->gimbal_yaw_motor.last_angle=gimbal_motor_control->gimbal_yaw_motor.set_angle;
	gimbal_motor_control->gimbal_yaw_motor.last_ecd=gimbal_motor_control->gimbal_yaw_motor.PidFb;
	gimbal_motor_control->gimbal_yaw_motor.last_spd=gimbal_motor_control->gimbal_yaw_motor.speed_fdb;
	
	pitch_out = gimbal_motor_control->gimbal_pitch_speed_pid.out;
	yaw_out = gimbal_motor_control->gimbal_yaw_speed_pid.out;
	CAN1_CMD_GIMBAL(yaw_out ,pitch_out , 0 , 0);
	
}
/**
  * @brief  云台任务
  * @param  void
  * @retval void
  * @attention  
  */
void Gimbal_task(void *pvParameters)
{
	Gimbal_Init(&gimbal_control);//云台初始化
	
	while(1)
	{		Gimbal_Mode_Set(&gimbal_control , &INPUTMOD);
//		Gimbal_Mode_Change(&gimbal_control);
		Gimbal_Motor_Control(&gimbal_control);
		
		vTaskDelay(1);  //系统延时
	}
}

/**
  * @brief  返回云台结构体指针
  * @param  void
  * @retval 
  * @attention  
  */
const Gimbal_Motor_t *get_gimbal_yaw_control_t_point(void)
{
  return &gimbal_control.gimbal_yaw_motor;
}

