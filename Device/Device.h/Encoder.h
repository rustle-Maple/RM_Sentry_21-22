#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "main.h"
#include "PID_Position.h"
#include "Robots_Control.h"

#define OneLoop_LineNumber (1024*4)

typedef struct
{
			
	//直接用线数的
	int16_t realValue_AB;
	
	int32_t TargerLine;
	
	int16_t lastValue_AB;
	
	int32_t Counts;
	int32_t totalLine;
	int32_t last_totalLine;

	P_PID_t P_PID;
	
	int32_t Update_times;
	int32_t FPS_lastline;
	uint8_t Offline_flag;
	
}Encoder_t;

extern Encoder_t Chassis_Encoder;

void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab);

#endif


