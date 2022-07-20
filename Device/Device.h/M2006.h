#ifndef M2006_H
#define M2006_H

#include "main.h"
#include "user_CAN.h"
#include "PID_Position.h"

#define M2006_SENDID 0x200		 //CAN���ͽo M3508�ĵ�� ID
#define M2006_READID_START 0x201 //CAN���յ� M3508�ĵ�� ��ʼID
#define M2006_READID_END 0x204	 //CAN���յ� M3508�ĵ�� ����ID

#define M2006_Max_Integral 10000
#define M2006_Max_Output 10000

typedef struct
{
	uint16_t realAngle; //�Ի����Ļ�е�Ƕ�
	int16_t realSpeed;	//���������ٶ�

	uint16_t lastAngle; //�ϴεĽǶ�

	P_PID_t S_PID_t; //��M2006��Ӧ��PID
	P_PID_t A_PID_t;

	float OutputCurrent; //M2006�������

	int32_t targetAngle; //Ŀ��Ƕ�
	int16_t targetSpeed; //Ŀ���ٶ�

	int16_t turnCount;	//ת����Ȧ��
	int32_t totalAngle; //ת�����ܽǶ�

	int32_t last_totalAngle; //�ϴε��ܽǶ�
	
	uint16_t OffLine_Detection; //���߼��
	uint8_t OffLine_Status;		//���߱�־λ

}M2006_t;

extern M2006_t M2006s[4];

void M2006_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

#endif
