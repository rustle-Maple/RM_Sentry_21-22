#ifndef SENSOR_H
#define SENSOR_H

#include <stdio.h>
#include "usart.h"
#include "CRC.h" 
#include "User_typedef.h"

#pragma anon_unions


#define SENSOR_BuffSize (9 + 2) //�������������ݻ���������
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
				uint8_t Start_Tag_one; //֡ͷ
				uint8_t Start_Tag_two; //֡ͷ
				
				uint8_t DIST_L; //֡ͷ
				uint8_t DIST_H; //֡ͷ
				
				uint8_t APM_L; //֡ͷ
				uint8_t APM_H; //֡ͷ

				uint8_t TEMP_L; //֡ͷ
				uint8_t TEMP_H; //֡ͷ
				
				uint8_t ZHECK_SUM; //֡β


			};
			uint8_t SENSOR_RawData[9];
		};

		int DIST;				//����
		int APM;				//���Ŷ�
		int TEMP;					//�¶�
	
	} RawData; //��������Э�� ����һ֡�����ݽṹ��

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//���ݴ����־λ
	uint8_t DataUpdate_Flag;				//���ݸ��±�־λ
	
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


