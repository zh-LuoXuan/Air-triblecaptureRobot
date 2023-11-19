#ifndef __USER_LIB_H
#define __USER_LIB_H

#include "sys.h"
#include "stdint.h"


typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num;       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
//快速开方
//extern fp32 invSqrt1(fp32 num);


//范围映射
fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax);
int int_map(int a, int amin, int amax, int bmin, int bmax);

//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int16死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
int Constrain(int amt, int low, int high);
float Constrain_float(float amt, float low, float high);
int16_t Constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t Constrain_int32_t(int32_t amt, int32_t low, int32_t high);
int16_t Limit_int16_t(int16_t amt, int16_t limit);
//斜坡函数
float RAMP_float( float final, float now, float ramp );
float RampInc_float( float *buffer, float now, float ramp );
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );

//一阶低通滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,  fp32 num);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
void abs_limit(fp32 *num, fp32 Limit);
//判断符号位
fp32 sign(fp32 value);
//得到相对角度
fp32 get_relative_pos(fp32 raw_ecd, fp32 center_offset);

#endif

