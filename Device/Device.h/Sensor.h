#ifndef SENSOR_H
#define SENSOR_H

#include <stdio.h>
#include "usart.h"
#include "CRC.h" 
#include "User_typedef.h"

#pragma anon_unions


#define SENSOR_BuffSize (9 + 2) //传感器接收数据缓冲区长度
extern uint8_t SENSOR_L_DataBuff[SENSOR_BuffSize];
extern uint8_t SENSOR_R_DataBuff[SENSOR_BuffSize];

typedef struct
{
	struct
	{
		union
		{
			struct
			{
				uint8_t Start_Tag_one; //帧头
				uint8_t Start_Tag_two; //帧头
				
				uint8_t DIST_L; //帧头
				uint8_t DIST_H; //帧头
				
				uint8_t APM_L; //帧头
				uint8_t APM_H; //帧头

				uint8_t TEMP_L; //帧头
				uint8_t TEMP_H; //帧头
				
				uint8_t ZHECK_SUM; //帧尾


			};
			uint8_t SENSOR_RawData[9];
		};

		int DIST;				//距离
		int APM;				//置信度
		int TEMP;					//温度
	
	} RawData; //传感器的协议 接收一帧的数据结构体

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//数据错误标志位
	uint8_t DataUpdate_Flag;				//数据更新标志位
	
	int32_t Last_Dist;
	
} sensor_t;

void Sensor_R_DataReceive(uint8_t *data);
void SENSOR_Handler(UART_HandleTypeDef *huart);

void Sensor_L_DataReceive(uint8_t *data);
void SENSOL_Handler(UART_HandleTypeDef *huart);

extern sensor_t Sensor_L;
extern sensor_t Sensor_R;

extern Frame_rate_t Sensor_R_Framerate;
extern Frame_rate_t Sensor_L_Framerate;

#endif


