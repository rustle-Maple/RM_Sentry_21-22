#ifndef __CAN2_SEND_H
#define __CAN2_SEND_H

#include "main.h"
#include "user_CAN.h"
#include "RM_JudgeSystem.h"
#include "DR16.h"
#include "stdbool.h"

typedef struct
{
	union
	{
		struct
		{
			int32_t totolline;
		};
		uint8_t encoder_SEND_all[4];
	};
}encoder_send_t;
extern encoder_send_t encoder_sent;

#define ENCODER_SEND_ID 0x152

#define JS_SEND_GAMESTATUS_ONE 0x150
#define JS_SEND_GAMESTATUS_TWO 0x151

#define JS_SEND_ROBOTHP_ID_ONE 0x146
#define JS_SEND_ROBOTHP_ID_TOW 0x147
#define JS_SEND_ROBOTHP_ID_THREE 0x148
#define JS_SEND_ROBOTHP_ID_FOUR 0x149

#define SPEED_CHANGE_SEND_ID 0x145

#define PLACE_SEND_ID 0x144

#define JS_SEND_HEAT_ID_ONE 0x142
#define JS_SEND_HEAT_ID_TWO 0x143

#define JS_SEND_STATUS_ID_ONE 0x138
#define JS_SEND_STATUS_ID_TWO 0x139
#define JS_SEND_STATUS_ID_THREE 0x140
#define JS_SEND_STATUS_ID_FOUR 0x141

#define JS_SEND_HURT_ID 0x137

#define JS_SEND_SHOOT_ID 0x136

#define DR16_SEND_PART_ONE 0x135
#define DR16_SEND_PART_TWO 0x134
#define DR16_SEND_PART_THREE 0x133



extern bool send_to_C;      // DR16_ң����_�Ƿ��͸�C��
extern bool break_is_raedy; //ɲ���Ѿ�װ������

extern bool in_END;
extern bool in_MID;

/*
������״̬���ݣ�10Hz ���ڷ���                              27(15)
ʵʱ�����������ݣ�50Hz ���ڷ���                            16(14)
�˺�״̬���ݣ��˺���������                               1
ʵʱ������ݣ��ӵ��������                               6
�ӵ�ʣ�෢���������л������Լ��ڱ������˷��ͣ�1Hz ���ڷ���   2
*/
extern bool send_to_C_JS_SHOOT;  //����ϵͳ_��������_�Ƿ��͸�C��
extern bool send_to_C_JS_HURT;   //����ϵͳ_�˺�����_�Ƿ��͸�C��
extern bool send_to_C_JS_STATUS; //����ϵͳ_״̬����_�Ƿ��͸�C��
extern bool send_to_C_JS_HEAT;   //����ϵͳ_״̬����_�Ƿ��͸�C��
extern bool send_to_C_JS_ROBOTHP;
extern bool send_to_C_JS_GAMESTATUS;

extern bool send_to_C_IN_END; //�Ƿ� ���ڹ����ͷ_״̬����_�Ƿ��͸�C�� 100ms��һ��
extern bool send_to_C_IN_MID; // ���ڹ���м�_״̬����_�Ƿ��͸�C��  1��һ��

extern bool send_to_C_B0_FLAG; 

extern bool send_to_C_SPEED_CHANGE; // ���ڹ���м�_״̬����_�Ƿ��͸�C��  1��һ��



void JS_send_SHOOT_control(void);
void JS_send_HURT_control(void);
void JS_send_STATUS_control(void);
void JS_send_HEAT_control(void);
void JS_send_robotHP_control(void);
void PLACE_send_control(void);
void disturbB0_send_control(void);
void Encoder_Send_control(void);
void SPEED_CHANGE_SEND_control(void);
void JS_send_gamestatus_control(void);
void DR16_send_master_control(void); //�ܿ���

void DR16_send_part_one(void);
void DR16_send_part_two(void);
void DR16_send_part_three(void);

#endif
