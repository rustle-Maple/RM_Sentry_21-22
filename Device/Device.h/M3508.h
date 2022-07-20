#ifndef M3508_H
#define M3508_H

#include "main.h"
#include "user_CAN.h"
#include "PID_Increment.h"
#include "PID_Position.h"


#define M3508_SENDID 0x200		 //CAN发送給 M3508的电调 ID
#define M3508_READID_START 0x201 //CAN接收到 M3508的电调 开始ID
#define M3508_READID_END 0x204	 //CAN接收到 M3508的电调 结束ID

#define M3508s1_SENDID 0x200 //摩擦轮 CAN发送给 M3508的电调ID
#define M3508s1_START 0x201  //
#define M3508s1_END 0x204	  //

#define M3508_Max_Integral 16384
#define M3508_Max_Output 16384

typedef struct
{
	uint16_t realAngle;	 //对回来的机械角度
	int16_t realSpeed;	 //读回来的速度
	int16_t realCurrent; //读回来的实际电流
	uint8_t temperture;	 //读回来的电机温度

	I_PID_t I_PID;		   //增量式pid
	P_PID_t P_PID;

	float OutputCurrent; //M3508输出电流

	uint16_t last_angle;
	uint16_t targetAngle; //目标角度
	float targetSpeed;  //目标速度

	int16_t turnCount;	//转过的圈数
	float totalAngle; //转过的总角度
	
	uint16_t OffLine_Detection; //离线检测
	uint8_t OffLine_Status;		//离线标志位

} M3508_t;

extern M3508_t M3508s[4];

typedef struct
{
	uint16_t realAngle;	 //对回来的机械角度
	int16_t realSpeed;	 //读回来的速度
	int16_t realCurrent; //读回来的实际电流
	uint8_t temperture;	 //读回来的电机温度

	I_PID_t I_PID; //该M3508对应的PID

	float OutputCurrent; //M3508输出电流

	uint16_t targetAngle; //目标角度
	int16_t targetSpeed;  //目标速度

	
	int16_t turnCount;	//转过的圈数
	int32_t totalAngle; //转过的总角度
	
	uint16_t OffLine_Detection; //离线检测
	uint8_t OffLine_Status;		//离线标志位


}M3508s1_t;

extern M3508s1_t M3508s1[4];

void M3508_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M3508_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

void M3508s1_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M3508s1_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

#endif
