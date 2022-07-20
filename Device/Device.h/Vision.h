#ifndef VISION_H
#define VISION_H

#include "main.h"
#include "PID_Position.h"
#include "PID_Increment.h"
#include <stdio.h>
#include "CRC.h"
#include "usart.h"
#include "Robots_Detection.h"
#include "User_typedef.h"
#pragma anon_unions

#include "DJI_C_IMU.h"


#define Vision_BuffSize (13 + 2) //�Ӿ��������ݻ���������
extern uint8_t Vision_DataBuff[Vision_BuffSize];

#define VisionPage_Width 1280
#define VisionPage_Height 800

typedef struct
{
	float X;
	float Y;

} XY_t;

typedef struct
{
	struct
	{
		union
		{
			struct
			{
				char Start_Tag; //֡ͷ

				uint8_t Armour; //�Ƿ�ʶ��װ�װ�
				uint8_t Beat; //�Ƿ񹥻����ų�����2�ţ�

				uint8_t Yaw_Dir;		//Yaw�᷽��
				uint8_t Yaw_Angle_Low;	//Yaw��ǶȵͰ�λ
				uint8_t Yaw_Angle_High; //Yaw��Ƕȸ߰�λ

				uint8_t Pitch_Dir;		  //Pitch�᷽��
				uint8_t Pitch_Angle_Low;  //Pitch��ǶȵĵͰ�λ
				uint8_t Pitch_Angle_High; //Pitch��Ƕȵĸ߰�λ

				uint8_t Depth_Low;	//��ȵĵͰ�λ
				uint8_t Depth_High; //��ȵĸ߰�λ

				uint8_t crc; //CRCУ��λ

				char End_Tag; //֡β
			};
			uint8_t VisionRawData[13];
		};

		float Yaw_Angle;				//Yaw��ĽǶ�
		float Pitch_Angle;				//Pitch��ĽǶ�
		float Depth;					//���
	
	} RawData; //�Ӿ���Э�� ����һ֡�����ݽṹ��

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//���ݴ����־λ
	uint8_t DataUpdate_Flag;				//���ݸ��±�־λ
	
	//�Ӿ��Ľ�������֡��
	WorldTime_RxTypedef Vision_WorldTimes;
	uint32_t FPS;							//֡��

} VisionData_t;

//�Ӿ��������ݽṹ��
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//������YAW�ǶȲ�
			float PitchAngle_Error; //������Pitch�ǶȲ�
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;

	int Gyro_z;			  //�����Ǽ��ٶ�С�������λ
	uint8_t Gyro_z_Hight; //�����Ǽ��ٶ�С�������λ�߰�λ
	uint8_t Gyro_z_low;	  //�����Ǽ��ٶ�С�������λ�Ͱ�λ

} VisionSend_Cloud_t;


extern VisionData_t VisionData;
extern VisionSend_Cloud_t Vision_Cloud;

void Vision_DataReceive(uint8_t *data);
void Vision_Handler(UART_HandleTypeDef *huart);

void Updata_SendVision_Stata(void);
void Vision_ID_Type_Init(void);
void Update_Vision_SendData(void);

#endif
