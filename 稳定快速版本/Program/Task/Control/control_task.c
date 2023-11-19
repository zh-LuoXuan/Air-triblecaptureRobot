#include "control_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "RemoteControl.h"
#include "time7.h"
#include "user_lib.h"


/*==============================================================*/
#define CONTROL_TASK_PRIO 30
#define CONTROL_STK_SIZE 512
TaskHandle_t ControlTask_Handler;
void Control_task(void *pvParameters);

/*==============================================================*/

void task_control_Create(void)
{
	xTaskCreate((TaskFunction_t)Control_task,
                (const char *)"Control_task",
                (uint16_t)CONTROL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CONTROL_TASK_PRIO,
                (TaskHandle_t *)&ControlTask_Handler);
}
/*==============================================================*/

fp32 top_spd_pid[3] = {TOP_SPEED_PID_KP , TOP_SPEED_PID_KI , TOP_SPEED_PID_KD};
fp32 top_agl_pid[3] = {TOP_ANGLE_PID_KP , TOP_ANGLE_PID_KI , TOP_ANGLE_PID_KD};
topControl_t top_Control;
fp32 topLocation[TOPPOSITION_NUM] = {L_POSITION, M_POSITION, R_POSITION};

extern u32 time7_count;

static void top_Init(topControl_t* topInit);
static void top_ModeSet(topControl_t* topMode);
static void top_BehavState_Set(topControl_t* topState);
static void top_Output(topControl_t* topOut);
static void top_Allstop(topControl_t* allStop);
static void top_conTinue_sec(topControl_t* topSec);
static void top_conTinue_fir(topControl_t* topFir);
static void top_oneByone_sec(topControl_t* oneBysec);
static void top_oneByone_fir(topControl_t* oneByfir);
static void storeHouse_round(void);
static void top_round1(void);
static void top_round2(void);


void Control_task(void *pvParameters)
{
	top_Init(&top_Control);
	while(1)
	{
		top_ModeSet(&top_Control);
		top_BehavState_Set(&top_Control);
		top_Output(&top_Control);
		vTaskDelay(1);
	}
}


static void top_Init(topControl_t* topInit)
{
	if(topInit == NULL)
	{
		return;
	}
	static uint16_t i = 0;
	
	PID_Init(&topInit->topAnglePID, PID_POSITION, top_agl_pid, TOP_ANGLE_PID_MAX_OUT, TOP_ANGLE_PID_MAX_IOUT);
	PID_Init(&topInit->topSpeedPID, PID_POSITION, top_spd_pid, TOP_SPEED_PID_MAX_OUT, TOP_SPEED_PID_MAX_IOUT);
	topInit->top_motorPoint = get_Top_Motor_Measure_Point();
	topInit->RC_Point = get_remote_control_point();
	topInit->topAglFdb = 0;
	topInit->topAglRef = 0;
	topInit->topSpdFdb = 0;
	topInit->topSpdRef = 0;
	topInit->giveCurrent = 0;
	
	vTaskDelay(DELAY_TIME_200MS);
	
	topInit->Center = topInit->top_motorPoint->real_ecd;

	for(i = 0; i < TOPPOSITION_NUM; i++)
	{
		topInit->appoint[i] = topInit->Center + topLocation[i];
	}
	
}


static void top_ModeSet(topControl_t* topMode)
{
	if(topMode == NULL)
	{
		return;
	}
	topMode->lastTopMode = topMode->topMode;
	
	if(switch_is_down(topMode->RC_Point->rc.s[ModeChannel_L]))
	{
		topMode->topMode = ALLSTOP;
	}
	else if((switch_is_mid(topMode->RC_Point->rc.s[ModeChannel_L])) && (switch_is_mid(topMode->RC_Point->rc.s[ModeChannel_R])))
	{
		topMode->topMode = ONEBYONE_FIR;
	}
	else if((switch_is_up(topMode->RC_Point->rc.s[ModeChannel_L])) && (switch_is_mid(topMode->RC_Point->rc.s[ModeChannel_R])))
	{
		topMode->topMode = ONEBYONE_SEC;
	}
	else if((switch_is_mid(topMode->RC_Point->rc.s[ModeChannel_L])) && (switch_is_up(topMode->RC_Point->rc.s[ModeChannel_R])))
	{
		topMode->topMode = CONTINUE_THR_FIR;
	}
	else if((switch_is_up(topMode->RC_Point->rc.s[ModeChannel_L])) && (switch_is_up(topMode->RC_Point->rc.s[ModeChannel_R])))
	{
		topMode->topMode = CONTINUE_THR_SEC;
	}
	else
	{
		return;
	}
}

static void top_BehavState_Set(topControl_t* topState)
{
	if(topState == NULL)
	{
		return;
	}
	if(topState->lastTopMode != topState->topMode)
	{
		top_Allstop(topState);
	}
	switch((uint16_t)topState->topMode)
	{
		case ALLSTOP:
		{
			topState->behavState = STOP;
			top_Allstop(topState);
			break;			
		}
		case ONEBYONE_FIR:
		{
			topState->behavState = ONEBY_FIR;
			if(RC_Control_CH4 > 500)
			{
				top_oneByone_fir(topState);
			}
			else if(RC_Control_CH4 < -500)
			{
				storeHouse_round();
			}
			break;			
		}
		case ONEBYONE_SEC:
		{
			topState->behavState = ONEBY_SEC;
			if(RC_Control_CH4 > 500)
			{
				top_oneByone_sec(topState);	
			}
			else if(RC_Control_CH4 < -500)
			{
				storeHouse_round();
			}				
			break;			
		}
		case CONTINUE_THR_FIR:
		{
			topState->behavState = THR_FIR;
			if(RC_Control_CH4 > 500)
			{
				top_conTinue_fir(topState);
			}
			else if(RC_Control_CH4 < -500)
			{
				storeHouse_round();
			}
			break;			
		}
		case CONTINUE_THR_SEC:
		{
			topState->behavState = THR_SEC;
			if(RC_Control_CH4 > 500)
			{
				top_conTinue_sec(topState);
			}
			else if(RC_Control_CH4 < -500)
			{
				storeHouse_round();
			}
			break;			
		}
		default:
			break;
	}
}

static void top_Output(topControl_t* topOut)
{
	if(topOut == NULL)
	{
		return;
	}
	PID_Calc(&topOut->topAnglePID, topOut->top_motorPoint->real_ecd, topOut->topAglRef);
	PID_Calc(&topOut->topSpeedPID, topOut->top_motorPoint->speed_rpm, topOut->topAnglePID.out);
	CAN1_CMD_TOP(topOut->topSpeedPID.out);
}

static void top_move(fp32 loCation)
{
	fp32 Appoint = 0;
	TIM_Cmd(TIM7, ENABLE);
	while(time7_count < 1000)
	{
		top_Control.topAglRef = RAMP_float( loCation, top_Control.top_motorPoint->real_ecd, 500);
		top_Output(&top_Control);
		vTaskDelay(DELAY_TIME_1MS);//ÏµÍ³ÑÓÊ±
	}	
	TIM_Cmd(TIM7, DISABLE);
	time7_count = 0;
}

static void top_round1(void)
{
	TURN_ON;
	vTaskDelay(100);
	CLAMP_ON;
	vTaskDelay(200);
	TURN_OFF;
	vTaskDelay(10);
}

static void top_round2(void)
{
	vTaskDelay(50);
	CLAMP_OFF;
	vTaskDelay(100);
	BULLET_BOX_ON;
	vTaskDelay(200);
	BULLET_BOX_OFF;
	vTaskDelay(10);
}

static void storeHouse_round(void)
{
	BULLET_STOREHOUSE_TOGGLE;
}
	
static void top_Allstop(topControl_t* allStop)
{
	if(allStop == NULL)
	{
		return;
	}
	CAN1_CMD_TOP(0);
	ALLAIR_OFF;
}

static void top_oneByone_fir(topControl_t* oneByfir)
{
	if(oneByfir == NULL)
	{
		return;
	}
	static uint16_t flag = 0;
	if(flag == 0)
		{
			INSTITUTIONAL_ON;
			vTaskDelay(100);
			top_move(oneByfir->appoint[Left]);
			vTaskDelay(50);
		}
	else if(flag == 1)
		{		
			top_round1();
			top_move(oneByfir->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
		}
	else if(flag == 2)
		{		
			top_round1();
			top_move(oneByfir->appoint[Right]);
			vTaskDelay(50);
			top_round2();
		}
	else if(flag == 3)
		{
			top_round1();
			top_move(oneByfir->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
			INSTITUTIONAL_OFF;
			vTaskDelay(100);
		}
	flag++;
	if(flag >= 4)
			flag = 0;
}
static void top_oneByone_sec(topControl_t* oneBysec)
{
	if(oneBysec == NULL)
	{
		return;
	}
	static uint16_t flag = 0;
	if(flag == 0)
		{
			INSTITUTIONAL_ON;
			vTaskDelay(100);
			STICK_ON;
			vTaskDelay(100);
			top_move(oneBysec->appoint[Left]);
			vTaskDelay(50);
		}
	else if(flag == 1)
		{		
			top_round1();
			top_move(oneBysec->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
		}
	else if(flag == 2)
		{		
			top_round1();
			top_move(oneBysec->appoint[Right]);
			vTaskDelay(50);
			top_round2();
		}
	else if(flag == 3)
		{
			top_round1();
			top_move(oneBysec->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
			STICK_OFF;
			vTaskDelay(100);
			INSTITUTIONAL_OFF;
			vTaskDelay(100);
		}
	flag++;
	if(flag >= 4)
			flag = 0;
}
static void top_conTinue_fir(topControl_t* topFir)
{
	if(topFir == NULL)
	{
		return;
	}
	static uint16_t flag = 0;
	{
		if(flag == 0)
		{
			INSTITUTIONAL_ON;
			vTaskDelay(100);
			top_move(topFir->appoint[Left]);
			vTaskDelay(50);
			top_round1();
			top_move(topFir->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
			top_round1();
			top_move(topFir->appoint[Right]);
			vTaskDelay(50);
			top_round2();
			top_round1();
			top_move(topFir->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
			INSTITUTIONAL_OFF;
			vTaskDelay(100);
		}
	}
}

static void top_conTinue_sec(topControl_t* topSec)
{
	if(topSec == NULL)
	{
		return;
	}
	static uint16_t flag = 0;
	if(flag == 0)
		{
			INSTITUTIONAL_ON;
			vTaskDelay(100);
			STICK_ON;
			vTaskDelay(50);
			top_move(topSec->appoint[Left]);
			vTaskDelay(50);
			top_round1();
			top_move(topSec->appoint[Middle]);
			vTaskDelay(50);
			top_round2();
			top_round1();
			top_move(topSec->appoint[Right]);
			vTaskDelay(50);
			top_round2();
			top_round1();
			top_move(topSec->appoint[Middle]);
			vTaskDelay(50);
			STICK_OFF;
			vTaskDelay(150);
			INSTITUTIONAL_OFF;
			vTaskDelay(100);
		}
}
