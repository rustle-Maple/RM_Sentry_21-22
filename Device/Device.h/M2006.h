#ifndef M2006_H
#define M2006_H

#include "main.h"
#include "user_CAN.h"
#include "PID_Position.h"

#define M2006_SENDID 0x200		 //CAN发送給 M3508的电调 ID
#define M2006_READID_START 0x201 //CAN接收到 M3508的电调 开始ID
#define M2006_READID_END 0x204	 //CAN接收到 M3508的电调 结束ID

#define M2006_Max_Integral 10000
#define M2006_Max_Output 10000

typedef struct
{
	uint16_t realAngle; //对回来的机械角度
	int16_t realSpeed;	//读回来的速度

	uint16_t lastAngle; //上次的角度

	P_PID_t S_PID_t; //该M2006对应的PID
	P_PID_t A_PID_t;

	float OutputCurrent; //M2006输出电流

	int32_t targetAngle; //目标角度
	int16_t targetSpeed; //目标速度

	int16_t turnCount;	//转过的圈数
	int32_t totalAngle; //转过的总角度

	int32_t last_totalAngle; //上次的总角度
	
	uint16_t OffLine_Detection; //离线检测
	uint8_t OffLine_Status;		//离线标志位

}M2006_t;

extern M2006_t M2006s[4];

void M2006_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

#endif
