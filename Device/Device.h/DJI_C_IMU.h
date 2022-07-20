#ifndef DJI_C_IMU_H
#define DJI_C_IMU_H

#include "main.h"
#include "can.h"
#include "user_CAN.h"
#include "User_typedef.h"
#include "PID_Position.h"
#pragma anon_unions

#define DJI_C_Angle 0x195
#define DJI_C_Gyro 0x165

#define Angle_turn_Radian 57.295779513082320876798154814105f

//处理过数据
typedef struct
{
	float yaw;
	float last_yaw;
	int32_t yaw_turnCounts;
	float total_yaw;
	float pitch;
	float last_pitch;
	int32_t pitch_turnCounts;
	float total_pitch;
	float Gyro_z;
	float Gyro_y;

	P_PID_t yaw_pid;
	P_PID_t gyro_z_pid;
	P_PID_t pitch_pid;
	P_PID_t gyro_y_pid;

} DJIC_IMU_t;
extern DJIC_IMU_t DJIC_IMU;

extern Frame_rate_t Euler_Framerate;
extern Frame_rate_t Gyro_Framerate;

//DJI_C 陀螺仪角度接收联合体
typedef union
{
	struct
	{
		float yaw;
		float pitch;
	};
	uint8_t BraodData[8];
} DJI_C_Euler_u;
extern DJI_C_Euler_u DJI_C_Euler_Receive;
//DJI_C 陀螺仪角速度接收联合体
typedef union
{
	struct
	{
		float Gyro_z;
		float Gyro_y;
	};
	uint8_t BraodData[8];
} DJI_C_Gyro_u;
extern DJI_C_Gyro_u DJI_C_Gyro_Receive;

void DJI_C_Euler_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);
void DJI_C_Gyro_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);
void Updata_Hand_Euler_Gyro_Data(void);

#endif
