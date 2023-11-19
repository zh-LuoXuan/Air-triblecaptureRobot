#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

#include "stm32f4xx.h"  
#include "gpio.h" 
#include "CAN_Receive.h"
#include "pid.h"


#define  TOP_ANGLE_PID_KP		15.0f	
#define  TOP_ANGLE_PID_KI		0.0f	
#define  TOP_ANGLE_PID_KD		0.0f	
#define  TOP_ANGLE_PID_MAX_OUT	16000.0f
#define  TOP_ANGLE_PID_MAX_IOUT 1000.0f

#define  TOP_SPEED_PID_KP		14.0f	
#define  TOP_SPEED_PID_KI		0.0f	
#define  TOP_SPEED_PID_KD		0.0f	
#define  TOP_SPEED_PID_MAX_OUT	16000.0f
#define  TOP_SPEED_PID_MAX_IOUT 1000.0f


#define MOTOR_REDUCTION_RATIO   (19.0f)

#define L_POSITION   (-1537.0f * MOTOR_REDUCTION_RATIO)
#define M_POSITION   (0.0f * MOTOR_REDUCTION_RATIO)
#define R_POSITION   (1537.0f * MOTOR_REDUCTION_RATIO)

#define TOPPOSITION_NUM  3

#define DELAY_TIME_1MS 		1
#define DELAY_TIME_5MS 		5
#define DELAY_TIME_10MS 	10
#define DELAY_TIME_100MS 	100
#define DELAY_TIME_150MS 	150
#define DELAY_TIME_200MS 	200
#define DELAY_TIME_300MS 	300
#define DELAY_TIME_450MS 	450
#define DELAY_TIME_500MS 	500
#define DELAY_TIME_750MS	750
#define DELAY_TIME_1000MS 1000
#define DELAY_TIME_1500MS 1500

enum 
{
	Left = 0,
	Middle,
	Right
};
typedef enum
{
	ALLSTOP = 0,
	CONTINUE_THR_FIR,
	CONTINUE_THR_SEC,
	ONEBYONE_FIR,
	ONEBYONE_SEC
}topMode_e;

typedef enum
{
	STOP = 0,
	THR_FIR,
	THR_SEC,
	ONEBY_FIR,
	ONEBY_SEC
}BehavState_e;

typedef struct 
{
	const motor_measure_t* top_motorPoint;
	const RC_ctrl_t* RC_Point;
	topMode_e topMode;
	topMode_e lastTopMode;
	BehavState_e behavState;
	fp32 topSpdRef;
	fp32 topSpdFdb;
	fp32 topAglRef;
	fp32 topAglFdb;
	fp32 giveCurrent;
	fp32 Center;
	fp32 appoint[TOPPOSITION_NUM];
	PidTypeDef topSpeedPID;
	PidTypeDef topAnglePID;
}topControl_t;

void task_control_Create(void);

#endif
