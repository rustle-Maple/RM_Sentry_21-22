#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#include "main.h"
#include "M3508.h"
#include "PID_Increment.h"
#include "Robots_Control.h"
#include "User_typedef.h"

#define Shoot_HOOK_FUN {\
	Shoot_Init,\
	Shoot_Control,\
}

typedef struct 
{
	//������������ָ��
	M3508s1_t* Shoot1;
	M3508s1_t* Shoot2;
	//����б�½ṹ��
	Ramp_Struct	ramp_Speed; 
	//��ʱ�ٶȱ���
	float temp_Speed;
	//����
	float high_Speed;
	//����
	float low_Speed;
}Shoot_t;
extern Shoot_t Shoot;

typedef struct 
{
	void (*Shoot_Init)(void);
	void (*Shoot_Control)(void);
}Shoot_FUN_t;
extern Shoot_FUN_t Shoot_FUN;

void Shoot_Init(void);
void Shoot_Control(void);

#endif

