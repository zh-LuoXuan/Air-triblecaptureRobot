#ifndef CANTASK_H
#define CANTASK_H
#include "sys.h"

#include "RemoteControl.h"


#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif


#define  RATE_BUF_SIZE 5
#define  ECD_RATE  (8192.f/360.f)

/* CAN send and receive ID */
typedef enum
{
	  CAN_M3508_SET_ID = 0x700,
	
	
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_CHASSIS_M1_ID = 0x201,
    CAN_CHASSIS_M2_ID = 0x202,
    CAN_CHASSIS_M3_ID = 0x203,
    CAN_CHASSIS_M4_ID = 0x204,
	
		CAN_TOP_ID = 0x1FF,
		CAN_TOP_M1_ID = 0x205,
	  
} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    fp32 ecd;
    int16_t speed_rpm;
    int16_t given_current;
    fp32  all_ecd;   //��������ֵ(��ֵ)
    fp32  real_ecd;
		int32_t  count;
    uint8_t temperate;
    fp32 last_ecd;
} motor_measure_t;



typedef struct
{
    int32_t diff;
    int32_t round_cnt;
    int32_t ecd_raw_rate;
    int32_t rate_buf[RATE_BUF_SIZE]; 	//buf��for filter
    uint8_t buf_count;					//�˲�����buf��
    int32_t filter_rate;				//�ٶ�
} Encoder_process_t;

//CAN1���͵����������
void CAN1_CMD_CHASSIS(int16_t data1 , int16_t data2 , int16_t data3 , int16_t data4);
//CAN1���͵����������
void CAN1_CMD_TOP(int16_t data);
//CAN1�򸱰巢��ң��������
void CAN_CMD_RC(RC_ctrl_t* RcData);
//�򸱰巢��IO����
void CAN_CMD_IO(u8 data);
//CAN1�򸱰巢�ͼ�������
void CAN_CMD_KEYMOUSE(RC_ctrl_t* RcData);

//����top���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Top_Motor_Measure_Point(void);
//����chassis���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);


#endif
