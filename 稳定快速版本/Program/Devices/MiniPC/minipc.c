#include "minipc.h"
#include "usart.h"
#include "string.h"

#include "IMUTask.h"
#include "MPU_TIME_Init.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

PC_Data GimbalRaw;
PC_Ctrl_Union_t PcData;

extern QueueHandle_t TxCOM6;
extern QueueHandle_t RxCOM6;

//通过指针方式获取原始数据
const PC_Data *get_PC_Data_Point(void)
{
    return &GimbalRaw;
}

//上位机数据解码
void MinipcDatePrcess(PC_Ctrl_t *pData)
{
    if(pData==NULL) 
    {
        return ;
    }
	GimbalRaw.PcPitch					= pData->angle_pitch;
	GimbalRaw.PcYaw						= pData->angle_yaw;
	GimbalRaw.pitch_angle_dynamic_ref	= -pData->angle_pitch + Angular_Handler.Pitch;
	GimbalRaw.yaw_angle_dynamic_ref		= -pData->angle_yaw + Angular_Handler.YAW; //右负左正
	GimbalRaw.shoot_flag				= pData->shoot_flag;
	GimbalRaw.frequency_adj				= pData->frequency_adj;

}

void minipc_Get_Message(DataRevice *Buffer)
{
	memcpy(PcData.PcDataArray,Buffer->buffer,MINIPC_FRAME_LENGTH);
	MinipcDatePrcess(&(PcData.PcDate));
}

