#ifndef GM6020_H
#define GM6020_H

#include "main.h"
#include "user_CAN.h"
#include "PID_Position.h"
#include "PID_Increment.h"

#pragma anon_unions 

#define GM6020_SENDID 0x1FF
#define GM6020_READID_START 0x205
#define GM6020_READID_END 0x208

typedef struct
{
	uint16_t readAngle;
	int16_t readSpeed;
	float freadSpeed;
	int16_t realCurrent;
	uint8_t readTemperture;

	int32_t targetAngle;
	int16_t targetSpeed;

	int16_t lastAngle;

	int32_t turnCount;
	float totalAngle;

	P_PID_t P_AnglePID;
	P_PID_t P_SpeedPID;

	int16_t PAoutVoltage;
	int16_t PSoutVoltage;
	
	uint8_t OffLine_Detection;				//离线检测
	uint8_t OffLine_Status;					//离线标志位
				
	
} GM6020_t;

extern GM6020_t GM6020s[4];

void GM6020_Yaw_SetVoltage(int16_t Vid1,int16_t Vid2,int16_t Vid3,int16_t Vid4);
void GM6020_SetVoltage(int16_t Vid1, int16_t Vid2, int16_t Vid3, int16_t Vid4);

void GM6020_Pitch_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);
void GM6020_Yaw_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

#endif
