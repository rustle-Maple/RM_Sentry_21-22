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
#define GY_IMU_READID	       0x414  //can2����̨������

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


/* ������ */
typedef struct {
		float x;                 //������pitch��ķ�������
		float y;                 //������Y�᷽������
		float z;                 //������yaw���ת���ٶ�
}Vector_t;

typedef struct {
	float Roll;                 //ROLL�᷽�򣬵�ǰ�ĽǶ�ֵ
	float Pitch;                //PITCH�᷽��
	float Yaw;                  //YAW�᷽��
}Eular_t;

typedef struct {
	
	Vector_t Gyro;              //�������ٶ�ֵ����          
	Eular_t  Eular;         	 //ŷ��������     
	
	Vector_t last_Gyro;					//�ϴ������ǵ��ٶ�ֵ
	Eular_t last_Eular;				//�ϴ�ŷ���ǵ�����
	
	Vector_t target_Gyro;				//Ŀ���������ٶ�ֵ
	Eular_t target_Eular;				//Ŀ��ŷ���ǵ�����
	
	Eular_t total_Eular;				//�ܵ�ŷ���ǵ�����
	
	int16_t  turnCount;					//����Ȧ��
	
	P_PID_t P_PID;
	I_PID_t I_PID;

	P_PID_t Yaw_Speed;
	
	int16_t outVoltage;					//����ʵ���Ͽ��ƵĻ�����̨����ĵ�ѹֵ
	
		uint32_t FPS;
	
}GY_IMU_t;

extern GY_IMU_t IMU_Cloud;		 

void GY6050_getCloundInfo(CAN_Rx_TypeDef RxMessage);

#endif

