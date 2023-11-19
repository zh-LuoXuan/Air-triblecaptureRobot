#include "user_lib.h"
#include "Filter_Control.h"
#include "arm_math.h"

/**
  * @brief			范围映射
  * @author			RW
  * @param[in]		原始数值
  * @param[in]		原始最小值
  * @param[in]		原始最大值
  * @param[in]		映射后的最小值
  * @param[in]		映射后的最大值
  * @retval			映射后的数值
  */
fp32 fp32_map(fp32 a, fp32 amin, fp32 amax, fp32 bmin, fp32 bmax)
{
	fp32 adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	return (fp32)(bdel * ((fp32)(a-amin) / adel))+bmin;
}
int int_map(int a, int amin, int amax, int bmin, int bmax)
{
	int adel = amax - amin, bdel = bmax - bmin;
	if(a > amax)a = amax;
	if(a < amin)a = amin;
	return (bdel * ((float)(a-amin) / adel))+bmin;
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }

    return Value;
}

//int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }

    return Value;
}

/**
  * @brief  限幅函数
  * @param  当前输入，下限，上限
  * @retval 当前输出
  * @attention  
  */
int Constrain(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

float Constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t Constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int32_t Constrain_int32_t(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

int16_t Limit_int16_t(int16_t amt, int16_t limit)
{
	if (amt > limit)
			return limit;
	else if(amt < -limit)
		  return (-limit);
	else
	    return amt;
}
/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
	if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						 now    += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						 now    += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}

float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec )
{
	float  factor = 0;
	factor = 0.05 * sqrt( 0.15 * (*time) );  //计算速度斜坡,time累加到296.3斜坡就完成
	if (status == 1)//按键被按下
	{
		if (factor < 1)//防止time太大
		{
			*time += inc;
		}
	}
	else		//按键松开
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}
	factor = constrain( factor, 0, 1 );//注意一定是float类型限幅
	return factor;  //注意方向
}

/**
  * @brief       一阶低通滤波初始化
  * @author      RM
  * @param[in]   一阶低通滤波结构体
  * @param[in]   间隔的时间，单位 s
  * @param[in]   滤波参数
  * @retval      返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,  fp32 num)
{
    first_order_filter_type->frame_period = frame_period;   //滤波的时间间隔
    first_order_filter_type->num = num;    //  滤波参数
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief       一阶低通滤波计算
  * @author      RM
  * @param[in]   一阶低通滤波结构体
  * @param[in]   间隔的时间，单位 s
  * @retval      返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num / (first_order_filter_type->num + first_order_filter_type->frame_period)
	* first_order_filter_type->out
	+ first_order_filter_type->frame_period / (first_order_filter_type->num + first_order_filter_type->frame_period)
	* first_order_filter_type->input;
}
// chu=  num/(timer + num )*out  +  timer/(num+timer) * int
//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

/**
  * @brief  得到相对角度
  * @param  
  * @retval 
  * @attention  
  *             
*/
fp32 get_relative_pos(fp32 raw_ecd, fp32 center_offset)
{
	fp32 tmp = 0;
	if (center_offset >= 4096)
	{
		if (raw_ecd > center_offset - 4096)
			tmp = raw_ecd - center_offset;
		else
		  tmp = raw_ecd + 8192 - center_offset;
	}
	else
	{
		if (raw_ecd > center_offset + 4096)
		  tmp = raw_ecd - 8192 - center_offset;
		else
		  tmp = raw_ecd - center_offset;
	}
	return tmp;
}
