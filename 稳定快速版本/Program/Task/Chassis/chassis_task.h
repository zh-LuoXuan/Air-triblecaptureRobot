#ifndef __CHASSIS_TASH_H
#define __CHASSIS_TASH_H

#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RemoteControl.h"

/*************************************��ʼ��PID����**************************************/

//��ʼ��3508 �ٶȻ� PID�����Լ� PID���������������
#define  CHASSIS_SPEED_PID_KP		20.0f	
#define  CHASSIS_SPEED_PID_KI		0.0f	
#define  CHASSIS_SPEED_PID_KD		0.0f	
#define  CHASSIS_SPEED_PID_MAX_OUT	16000.0f
#define  CHASSIS_SPEED_PID_MAX_IOUT 1000.0f

//���� PID�����Լ� PID���������������
#if ISGIMBALNULL 
#define  CHASSIS_FOLLOW_ANGLE_PID_KP		1.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_KI		0.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_KD		0.0f	
#define  CHASSIS_FOLLOW_ANGLE_PID_MAX_OUT	1000.0f
#define  CHASSIS_FOLLOW_ANGLE_PID_MAX_IOUT	1000.0f
#endif


/*************************************�޷�**************************************/

#define CHASSIS_SPEED_LIMIT	 4000.0f

#define CHASSIS_RC_MAX_SPEED_X   2000.0f
#define CHASSIS_RC_MAX_SPEED_Y   2000.0f
#define CHASSIS_RC_MAX_SPEED_W   80.0f


/*************************************��������**************************************/

#define RC_VW_RESPONSE  3.0f  //ң����������Ӧ
#define RC_VX_RESPONSE  10.0f  //ң����������Ӧ
#define RC_VY_RESPONSE  10.0f  //ң����������Ӧ

/*************************************�Ƿ�װ��̨**************************************/

#define ISGIMBALNULL 0

/*************************************������ز���**************************************/

#define WHEELBASE 						 400  //ǰ����ࣨmm��
#define WHEELTRACK             360  //�����־ࣨmm��
#define RADIAN_COEF        		 57.3f //����ϵ��
#define PERIMETER              478  //�����ܳ���mm��
#define CHASSIS_DECELE_RATIO   (1.0f/19.0f)  //������ٱ�

typedef enum 
{
	ALL_STOP = 0,
	REMOTE_CONTROL
}CHASSIS_State_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 SpeedRef;
	fp32 GiveCurrent; 
	fp32 SpeedFdb; 
	PidTypeDef chassis_speed_pid;
} Chassis_Motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_rc_ctrl;		//ң������
	
	CHASSIS_State_e control_mode;
	Chassis_Motor_t chassis_motor[4];
  
	PidTypeDef follow_gimbal_pid;
	
	fp32 chassis_vx;		
	fp32 chassis_vy;		
	fp32 chassis_vw;
	
	#if ISGIMBALNULL 
		fp32 rotate_x_offset;
		fp32 rotate_y_offset;
	#endif
 
} Chassis_Control_t;

void task_Chsssis_Create(void);
#endif

