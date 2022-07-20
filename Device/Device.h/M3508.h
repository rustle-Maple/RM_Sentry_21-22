#ifndef M3508_H
#define M3508_H

#include "main.h"
#include "user_CAN.h"
#include "PID_Increment.h"
#include "PID_Position.h"


#define M3508_SENDID 0x200		 //CAN���ͽo M3508�ĵ�� ID
#define M3508_READID_START 0x201 //CAN���յ� M3508�ĵ�� ��ʼID
#define M3508_READID_END 0x204	 //CAN���յ� M3508�ĵ�� ����ID

#define M3508s1_SENDID 0x200 //Ħ���� CAN���͸� M3508�ĵ��ID
#define M3508s1_START 0x201  //
#define M3508s1_END 0x204	  //

#define M3508_Max_Integral 16384
#define M3508_Max_Output 16384

typedef struct
{
	uint16_t realAngle;	 //�Ի����Ļ�е�Ƕ�
	int16_t realSpeed;	 //���������ٶ�
	int16_t realCurrent; //��������ʵ�ʵ���
	uint8_t temperture;	 //�������ĵ���¶�

	I_PID_t I_PID;		   //����ʽpid
	P_PID_t P_PID;

	float OutputCurrent; //M3508�������

	uint16_t last_angle;
	uint16_t targetAngle; //Ŀ��Ƕ�
	float targetSpeed;  //Ŀ���ٶ�

	int16_t turnCount;	//ת����Ȧ��
	float totalAngle; //ת�����ܽǶ�
	
	uint16_t OffLine_Detection; //���߼��
	uint8_t OffLine_Status;		//���߱�־λ

} M3508_t;

extern M3508_t M3508s[4];

typedef struct
{
	uint16_t realAngle;	 //�Ի����Ļ�е�Ƕ�
	int16_t realSpeed;	 //���������ٶ�
	int16_t realCurrent; //��������ʵ�ʵ���
	uint8_t temperture;	 //�������ĵ���¶�

	I_PID_t I_PID; //��M3508��Ӧ��PID

	float OutputCurrent; //M3508�������

	uint16_t targetAngle; //Ŀ��Ƕ�
	int16_t targetSpeed;  //Ŀ���ٶ�

	
	int16_t turnCount;	//ת����Ȧ��
	int32_t totalAngle; //ת�����ܽǶ�
	
	uint16_t OffLine_Detection; //���߼��
	uint8_t OffLine_Status;		//���߱�־λ


}M3508s1_t;

extern M3508s1_t M3508s1[4];

void M3508_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M3508_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

void M3508s1_setCurrent(int16_t Iid1, int16_t Iid2, int16_t Iid3, int16_t Iid4);

void M3508s1_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

#endif
