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


#define Vision_BuffSize (13 + 2) //视觉接收数据缓冲区长度
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
				char Start_Tag; //帧头

				uint8_t Armour; //是否识别到装甲板
				uint8_t Beat; //是否攻击（排除工程2号）

				uint8_t Yaw_Dir;		//Yaw轴方向
				uint8_t Yaw_Angle_Low;	//Yaw轴角度低八位
				uint8_t Yaw_Angle_High; //Yaw轴角度高八位

				uint8_t Pitch_Dir;		  //Pitch轴方向
				uint8_t Pitch_Angle_Low;  //Pitch轴角度的低八位
				uint8_t Pitch_Angle_High; //Pitch轴角度的高八位

				uint8_t Depth_Low;	//深度的低八位
				uint8_t Depth_High; //深度的高八位

				uint8_t crc; //CRC校验位

				char End_Tag; //帧尾
			};
			uint8_t VisionRawData[13];
		};

		float Yaw_Angle;				//Yaw轴的角度
		float Pitch_Angle;				//Pitch轴的角度
		float Depth;					//深度
	
	} RawData; //视觉的协议 接收一帧的数据结构体

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//数据错误标志位
	uint8_t DataUpdate_Flag;				//数据更新标志位
	
	//视觉的接收数据帧率
	WorldTime_RxTypedef Vision_WorldTimes;
	uint32_t FPS;							//帧率

} VisionData_t;

//视觉发送数据结构体
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//陀螺仪YAW角度差
			float PitchAngle_Error; //陀螺仪Pitch角度差
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;

	int Gyro_z;			  //陀螺仪加速度小数点后两位
	uint8_t Gyro_z_Hight; //陀螺仪加速度小数点后两位高八位
	uint8_t Gyro_z_low;	  //陀螺仪加速度小数点后两位低八位

} VisionSend_Cloud_t;


extern VisionData_t VisionData;
extern VisionSend_Cloud_t Vision_Cloud;

void Vision_DataReceive(uint8_t *data);
void Vision_Handler(UART_HandleTypeDef *huart);

void Updata_SendVision_Stata(void);
void Vision_ID_Type_Init(void);
void Update_Vision_SendData(void);

#endif
