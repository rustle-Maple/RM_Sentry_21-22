#ifndef SNAIL_H
#define SNAIL_H

#include "main.h"
#include "tim.h"

//extern uint16_t a ;

typedef struct
{
	//油门值
	uint16_t Accelerator_Value;	
	
	//设置 行程 的值
	uint16_t Strock_Value;					//行程的最大值一般为660
	
	uint16_t Final_OutputSpeed;			//最终的输出速度
	
}Snail_t;
extern Snail_t Snail;

extern uint16_t Strock_Value;
extern uint16_t Accelerator_Value;

void Snail_Init(void);





#endif

