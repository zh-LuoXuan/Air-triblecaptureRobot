#ifndef __MINIPC_H
#define __MINIPC_H

#include "sys.h"
#include "usart.h"

//PC数据转换成期望结构，提取合成有用数据
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
	uint16_t yaw_limit;
	uint16_t outpost_HP;
    uint32_t time;      //上位机发过来的时间标签
    float distance;
    uint32_t CpuTime;   //下位机CPU时间标签
    float PcPitch;      //上位机发过来的pitch轴数据
    float PcYaw;       //上位机发过来的yaw轴数据
	unsigned char shoot_flag;//射击指令
	unsigned char frequency_adj;//射频调整
} PC_Data;

//原结构体
typedef __packed struct 
{
	unsigned char head;
	float angle_pitch;
	float angle_yaw;
//	float distance;
	unsigned char shoot_flag;
	unsigned char frequency_adj;
	unsigned char end;
}PC_Ctrl_t;

//上位机数据转换共用体
typedef union 
{
	PC_Ctrl_t PcDate;
	unsigned char PcDataArray[sizeof(PC_Ctrl_t)];
}PC_Ctrl_Union_t;


//通过指针方式获取原始数据
const PC_Data *get_PC_Data_Point(void);
void minipc_Get_Message(DataRevice *Buffer);

#endif

