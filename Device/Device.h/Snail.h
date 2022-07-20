#ifndef SNAIL_H
#define SNAIL_H

#include "main.h"
#include "tim.h"

//extern uint16_t a ;

typedef struct
{
	//����ֵ
	uint16_t Accelerator_Value;	
	
	//���� �г� ��ֵ
	uint16_t Strock_Value;					//�г̵����ֵһ��Ϊ660
	
	uint16_t Final_OutputSpeed;			//���յ�����ٶ�
	
}Snail_t;
extern Snail_t Snail;

extern uint16_t Strock_Value;
extern uint16_t Accelerator_Value;

void Snail_Init(void);





#endif

