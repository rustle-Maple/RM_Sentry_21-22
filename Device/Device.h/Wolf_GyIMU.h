#ifndef WOLF_GYIMU_H
#define WOLF_GYIMU_H

#include "main.h"
#include "string.h"
#include "User_can.h"
#include "User_math.h"
#include "PID_Position.h"
#include "PID_Increment.h"
#include "User_typedef.h"
#include "Robots_Detection.h"


#pragma anon_unions

//#define GY_IMU_PACKAGE_LENGTH 	18
#define GY_IMU_READID	       0x414  //can2的云台陀螺仪

typedef union {
	struct {
		uint16_t yaw;
		int16_t gyro_z;
		int16_t pitch;
		int16_t gyro_x;
	};
	uint8_t dataBuff[8];

	
}bno055_data_u;
extern bno055_data_u bno055_data[2];


/* 陀螺仪 */
typedef struct {
		float x;                 //浮点数pitch轴的方向向量
		float y;                 //浮点数Y轴方向向量
		float z;                 //浮点数yaw轴的转动速度
}Vector_t;

typedef struct {
	float Roll;                 //ROLL轴方向，当前的角度值
	float Pitch;                //PITCH轴方向
	float Yaw;                  //YAW轴方向
}Eular_t;

typedef struct {
	
	Vector_t Gyro;              //陀螺仪速度值！！          
	Eular_t  Eular;         	 //欧拉角数据     
	
	Vector_t last_Gyro;					//上次陀螺仪的速度值
	Eular_t last_Eular;				//上次欧拉角的数据
	
	Vector_t target_Gyro;				//目标陀螺仪速度值
	Eular_t target_Eular;				//目标欧拉角的数据
	
	Eular_t total_Eular;				//总的欧拉角的数据
	
	int16_t  turnCount;					//计数圈数
	
	P_PID_t P_PID;
	I_PID_t I_PID;

	P_PID_t Yaw_Speed;
	
	int16_t outVoltage;					//由于实际上控制的还是云台电机的电压值
	
		uint32_t FPS;
	
}GY_IMU_t;

extern GY_IMU_t IMU_Cloud;		 

void GY6050_getCloundInfo(CAN_Rx_TypeDef RxMessage);

#endif

