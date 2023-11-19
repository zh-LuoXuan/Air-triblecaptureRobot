#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f4xx.h"                  // Device header

#define		JUDGE_21		21			//21年裁判系统
#define		JUDGE_VERSION	JUDGE_21

#if JUDGE_VERSION==JUDGE_21

//帧头详细定义
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

/* data send (forward) */
/* data receive */
#define BLUE  0
#define RED   1


/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  11    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1     比赛结果数据         			比赛结束后发送      
	ID: 0x0003  Byte:  28    机器人血量数据   				1Hz发送
	ID: 0x0004  Byte:  3 	 飞镖发射状态，飞镖发射后发送
	ID: 0x0005  Byte:  11 	 人工智能挑战赛加成与惩罚区状态  1Hz 周期发送，发送范围：所有机器人

	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0104  Byte:  2    裁判警告数据，警告发生后发送 
	ID: 0X0105  Byte:  1    飞镖发射口倒计时			1Hz 周期发送 

	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0208  Byte:  2    子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz 周期发送
	ID: 0x0209  Byte:  4    机器人 RFID 状态，1Hz 周期发送
	
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	ID: 0x0302  Byte:  n    自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
	ID: 0x0303	Byte:  15	客户端小地图交互数据，触发发送
	ID: 0x0304	Byte:  12	键盘、鼠标信息，通过图传串口发送
	
*/

//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_HP       			= 0x0003,//机器人血量数据
	ID_game_dart_stauts				= 0x0004,//飞镖发射状态
	ID_game_ICRA_buff_t				= 0x0005,//人工智能挑战赛加成与惩罚区状态
	
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
	
	
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据
	
	
	ID_ROBORT_COM_DATA				= 0X0301,//10HZ
} CmdID;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct
{
	
	uint8_t game_type : 4;//0-3 bit：比赛类型
	uint8_t game_progress : 4;//4-7 bit：当前比赛阶段
	uint16_t stage_remain_time;//当前阶段剩余时间，单位 s
	uint64_t SyncTimeStamp;//机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效

} ext_game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  2    机器人血量数据 */
typedef __packed struct
{
	uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
	uint16_t red_2_robot_HP; //红 2 工程机器人血量
	uint16_t red_3_robot_HP; //红 3 步兵机器人血量
	uint16_t red_4_robot_HP; //红 4 步兵机器人血量
	uint16_t red_5_robot_HP; //红 5 步兵机器人血量
	uint16_t red_7_robot_HP; //红 7 哨兵机器人血量
	uint16_t red_outpost_HP; //红方前哨战血量
	uint16_t red_base_HP; 	 //红方基地血量
	uint16_t blue_1_robot_HP; //蓝 1 英雄机器人血量
	uint16_t blue_2_robot_HP; //蓝 2 工程机器人血量
	uint16_t blue_3_robot_HP; //蓝 3 步兵机器人血量
	uint16_t blue_4_robot_HP; //蓝 4 步兵机器人血量
	uint16_t blue_5_robot_HP; //蓝 5 步兵机器人血量
	uint16_t blue_7_robot_HP; //蓝 7 哨兵机器人血量
	uint16_t blue_outpost_HP; //蓝方前哨站血量
	uint16_t blue_base_HP;	  //蓝方基地血量
} ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;
//人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3; 
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct
{
	uint8_t supply_projectile_id; //补给站口 ID：
	uint8_t supply_robot_id; //补弹机器人 ID：
	uint8_t supply_projectile_step; //出弹口开闭状态：
	uint8_t supply_projectile_num;//补弹数量
} ext_supply_projectile_action_t;

/* ID: 0X0103  Byte:  2    场地补给站预约子弹数据 请求补给站补弹数据，由参赛队发送，上限 10Hz。*/
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 

/* ID: 0X0104  Byte: 2 		裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送 */
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

/*ID: 0X0105  Byte: 1		飞镖发射口倒计时：cmd_id (0x0105)*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;//冷却
	uint16_t shooter_id1_17mm_cooling_limit;//热量上限
	uint16_t shooter_id1_17mm_speed_limit;//上限速度
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct
{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct
{
	uint8_t power_rune_buff;
//bit 0 机器人血量补血状态 
//bit 1：枪口热量冷却加速
//bit 2：机器人防御加成
//bit 3：机器人攻击加成
}ext_buff_t;

/* ID: 0x0205  Byte:  2    空中机器人能量状态数据 */
typedef __packed struct
{
	uint8_t attack_time;
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
	/*
	  0x0 装甲伤害扣血；
	  0x1 模块掉线扣血；
    0x2 超射速扣血；
	  0x3 超枪口热量扣血；
	  0x4 超底盘功率扣血；
	  0x5 装甲撞击扣血
	*/
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID: 0x0208	发送频率：10Hz 子弹剩余发射数 周期发送，所有机器人发送*/
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm; 
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* ID: 0x0209	机器人 RFID 状态*/
typedef __packed struct
{
	uint32_t rfid_status;
/*
bit 0：基地增益点 RFID 状态；
bit 1：高地增益点 RFID 状态；
bit 2：能量机关激活点 RFID 状态；
bit 3：飞坡增益点 RFID 状态；
bit 4：前哨岗增益点 RFID 状态；
bit 5：资源岛增益点 RFID 状态；
bit 6：补血点增益点 RFID 状态；
bit 7：工程机器人补血卡 RFID 状态；
*/
} ext_rfid_status_t;

/* ID: 0x020A	飞镖机器人客户端指令数据*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 
	
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/
//交互数据接收信息：0x0301
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

typedef struct 
{
	uint8_t	 Flag;			//帧头位数
	uint16_t data_len;		//数据长度
	uint16_t data_cnt;		//自加位置
	uint8_t	 data;
}Judge_FLAG;

typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;
  float data1;
  float data2;
  float data3;
	u8    mask;
} client_show_data_t;

typedef __packed struct
{
	xFrameHeader		FrameHeader;
	CmdID			    CmdID;

  __packed union
	{ 
		ext_game_status_t 			game_information;
		ext_robot_hurt_t  			blood_changed_data;
		ext_shoot_data_t       		real_shoot_data;
		ext_power_heat_data_t   	real_powerheat_data;
		ext_event_data_t      		rfid_data;
		ext_game_result_t      		game_result_data;
		ext_buff_t         			get_buff_data;
		ext_game_robot_pos_t		gameRobotPos;
		client_show_data_t  		client_show_data;
	}Data;
	uint16_t		CRC16;	//所有数据CRC校验
}Dateframe_t;//数据帧

/* 交互数据接收信息：0x0301  */

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

/*-------------1. 交互数据接收信息： 0x0301。 发送频率：上限 10Hz---------
字节偏移量 大小 说明 备注
0 2 数据段的内容ID
2 2 发送者的ID   需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
4 2 接收者的ID  需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
6 x 内容数据段x 最大为 113
----*/

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;
//客户端删除图形 机器人间通信：0x0301
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;
//图形数据
typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
} graphic_data_struct_t;

//客户端绘制一个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
//客户端绘制两个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//客户端绘制五个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
//客户端绘制字符 机器人间通信
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;
//客户端绘制七个图形 机器人间通信
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
//客户端下发信息
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} ext_robot_commands_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
/* data send (forward) */
/* data receive */


#define INFANTRY 1
#define HERO 2
#define ENGINEER 3
#define SENTINEL 4
typedef struct
{
	int16_t Voltage;
	int16_t Current;
	int16_t Supply_Num;
	int16_t ShooterHeat_17mm;
	int16_t ShooterHeat_42mm;
	int16_t Power;
	int16_t PowerBuffer;
	uint8_t level;
	uint8_t hurt_type;
}JudgementType;

typedef struct
{
	uint8_t 	graph_operate_type;
	uint8_t 	graph_type;
	uint8_t 	graph_name[5];
	uint8_t 	graph_color;
	uint8_t		graph_line_width;
	uint16_t graph_start_x;
	uint16_t graph_start_y;
	uint16_t graph_radius;
	uint16_t graph_dst_x;
	uint16_t graph_dst_y;
	uint8_t text_lengh;
	uint8_t text[30];
}Graph_Data_Type;

typedef struct
{
	int16_t 		ShootLevel;
	int16_t 		SuperCapacitorComment;
	float 					bullet_can_shoot;
	uint8_t 		State_Mask;
	uint8_t 		SuperCapacitorState;
	Graph_Data_Type Graph_Data ;
}SendToJudgementDataType;

//机器人交互信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;


#endif


void  judgement_data_handler(uint8_t *p_frame);
void determine_ID(void);
#endif
