#include "CAN2_SEND.h"

uint8_t js_SEND_all[33]; // 4x8=32
uint8_t place_SEND_all[8]; 
encoder_send_t encoder_sent;


bool send_to_C;			 // DR16_遥控器_是否发送给C板
bool break_is_raedy = 1; //刹车已经装备好了

bool in_END;
bool in_MID;

/*
机器人状态数据，10Hz 周期发送                              27(15)
实时功率热量数据，50Hz 周期发送                            16(14)
伤害状态数据，伤害发生后发送                               1
实时射击数据，子弹发射后发送                               6
子弹剩余发送数，空中机器人以及哨兵机器人发送，1Hz 周期发送   2
*/
bool send_to_C_JS_SHOOT;	//裁判系统_发射数据_是否发送给C板
bool send_to_C_JS_HURT;		//裁判系统_伤害数据_是否发送给C板
bool send_to_C_JS_STATUS;	//裁判系统_状态数据_是否发送给C板
bool send_to_C_JS_HEAT = 0; //裁判系统_状态数据_是否发送给C板
bool send_to_C_JS_ROBOTHP = 0;
bool send_to_C_JS_GAMESTATUS = 0;

bool send_to_C_IN_END = 0; //是否 处于轨道尽头_状态数据_是否发送给C板 100ms发一次
bool send_to_C_IN_MID = 0; // 处于轨道中间_状态数据_是否发送给C板  1秒一次

bool send_to_C_B0_FLAG = 0;

bool send_to_C_SPEED_CHANGE = 0; // 处于轨道中间_状态数据_是否发送给C板  1秒一次

void SPEED_CHANGE_SEND_control()
{
	//	for(int i=0;i<7;i++ )//0到6位有效,一共7位
	//{
	//	js_SEND_all[i]=ext_shoot_data.data.dataBuff[i];
	//}
	CAN_SendData(&hcan2, CAN_ID_STD, SPEED_CHANGE_SEND_ID, &js_SEND_all[0]);
}

void JS_send_SHOOT_control()
{
	for (int i = 0; i < 7; i++) // 0到6位有效,一共7位
	{
		js_SEND_all[i] = ext_shoot_data.data.dataBuff[i];
	}
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_SHOOT_ID, &js_SEND_all[0]);
}
void JS_send_HURT_control()
{

	js_SEND_all[0] = ext_robot_hurt.data.dataBuff[0];

	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_HURT_ID, &js_SEND_all[0]);
}

void JS_send_STATUS_control()
{

	for (int i = 0; i < 27; i++) // 0到26位有效,一共27位
	{
		js_SEND_all[i] = ext_game_robot_state.data.dataBuff[i];
	}
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_STATUS_ID_ONE, &js_SEND_all[0]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_STATUS_ID_TWO, &js_SEND_all[8]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_STATUS_ID_THREE, &js_SEND_all[16]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_STATUS_ID_FOUR, &js_SEND_all[24]);
	osDelay(1);
}

void JS_send_HEAT_control()
{

	for (int i = 0; i < 18; i++) // 0到17位有效,一共18位
	{
		js_SEND_all[i] = ext_power_heat_data.data.dataBuff[i];
	}
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_HEAT_ID_ONE, &js_SEND_all[0]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_HEAT_ID_TWO, &js_SEND_all[8]);
}

void JS_send_robotHP_control()
{
	for(int i = 0; i < 32 ; i++)
	{
		js_SEND_all[i] = ext_game_robot_HP.data.dataBuff[i];
	}
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_ROBOTHP_ID_ONE, &js_SEND_all[0]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_ROBOTHP_ID_TOW, &js_SEND_all[8]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_ROBOTHP_ID_THREE, &js_SEND_all[16]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_ROBOTHP_ID_FOUR, &js_SEND_all[24]);
	osDelay(1);
		
}

void JS_send_gamestatus_control()
{
	for(int i = 0;i < 11; i++)
	{
		js_SEND_all[i] = ext_game_status.data.dataBuff[i];
	}
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_GAMESTATUS_ONE, &js_SEND_all[0]);
	osDelay(1);
	CAN_SendData(&hcan2, CAN_ID_STD, JS_SEND_GAMESTATUS_TWO, &js_SEND_all[8]);
	osDelay(1);
	
}

void PLACE_send_control()
{

	if (in_END == 1) //不在轨道中间段,在末端
	{
		//先右后左
			if(Sensor_R.RawData.DIST < 80)
			{
				if(0)//Sensor_R.RawData.APM > 1000)
				{
					place_SEND_all[0] = 0;
					place_SEND_all[7] = 1;
				}
				else
				{
					if(Chassis_Encoder.totalLine < (Chassis.Route_limit.right + 15000))
					{
						place_SEND_all[0] = 0;
						place_SEND_all[7] = 1;
					}
				}
			}
			else if(Sensor_L.RawData.DIST < 80)
			{
				if(0)//Sensor_L.RawData.APM > 1000)
				{
					place_SEND_all[0] = 1;
					place_SEND_all[7] = 0;
				}
				else
				{
					if(Chassis_Encoder.totalLine > (Chassis.Route_limit.left - 15000))
					{
						place_SEND_all[0] = 1;
						place_SEND_all[7] = 0;
					}
				}
			}
	}
	else //在轨道中间段
	{
		place_SEND_all[0] = 0;
		place_SEND_all[7] = 0;
	}

	place_SEND_all[1] = B0_flag;
	place_SEND_all[2] = 0;

	if (in_MID == 1) //在轨道中间段
	{
		place_SEND_all[3] = 1;
		place_SEND_all[4] = 1;
	}
	else //不在轨道中间段
	{
		place_SEND_all[3] = 0;
		place_SEND_all[4] = 0;
	}

	place_SEND_all[5] = 0;
	place_SEND_all[6] = B0_flag;

	CAN_SendData(&hcan2, CAN_ID_STD, PLACE_SEND_ID, &place_SEND_all[0]);
}

void Encoder_Send_control(void)
{
	static int encoder_send_time = 0;
	
	encoder_sent.encoder_SEND_all[0] = Chassis.encoder->totalLine;
	encoder_sent.encoder_SEND_all[1] = Chassis.encoder->totalLine >> 8;
	encoder_sent.encoder_SEND_all[2] = Chassis.encoder->totalLine >> 16;
	encoder_sent.encoder_SEND_all[3] = Chassis.encoder->totalLine >> 24;
	
	if(encoder_send_time % 100 == 0)
	{
		CAN_SendData(&hcan2, CAN_ID_STD, ENCODER_SEND_ID, encoder_sent.encoder_SEND_all);
	}
}

//B0位置标识
void disturbB0_send_control(void)
{
	if(B0_flag == 1)
	{
//		place_SEND_all[1] = 1;
//		place_SEND_all[6] = 1;
//	}
//	else
//	{
//		place_SEND_all[1] = 0;
//		place_SEND_all[6] = 0;
	}
}

uint8_t DR16_SEND_all[24]; // 3x8=24

void DR16_send_master_control()
{

	for (int i = 0; i < 18; i++) // 0到17位有效,一共18位
	{
		DR16_SEND_all[i] = DR16_RxBUFF[i];
	}

	DR16_send_part_one();
	osDelay(1);
	DR16_send_part_two();
	osDelay(1);
	DR16_send_part_three();
}

void DR16_send_part_one()
{
	CAN_SendData(&hcan2, CAN_ID_STD, DR16_SEND_PART_ONE, &DR16_SEND_all[0]);
}
void DR16_send_part_two()
{
	CAN_SendData(&hcan2, CAN_ID_STD, DR16_SEND_PART_TWO, &DR16_SEND_all[8]);
}
void DR16_send_part_three()
{

	CAN_SendData(&hcan2, CAN_ID_STD, DR16_SEND_PART_THREE, &DR16_SEND_all[16]);
}
