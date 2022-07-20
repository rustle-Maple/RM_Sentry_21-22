#ifndef _ANO_H 
#define _ANO_H 

#include "user_common.h"


void ANO_Send_Data_V1(int16_t Temp_Target1,int16_t Temp_Now1,int16_t Temp_Target2,int16_t Temp_Now2);
//void ANO_Send_Data_V2(int16_t Temp_Target1,int16_t Temp_Now1,int16_t Temp_Target2,int16_t Temp_Now2,int16_t Temp_Target3,int16_t Temp_Now3,int16_t Temp_Target4,int16_t Temp_Now4);
void ANO_Send_Data_V2(int16_t Temp_Target1,int16_t Temp_Now1,int16_t Temp_Target2,int16_t Temp_Now2);
void ANO_Send_Data_V3(int16_t Temp_Target1,int16_t Temp_Now1,int16_t Temp_Target2,int16_t Temp_Now2);
void ANO_Send_Data_V4(void);
//void ANO_Send_Data_Init(int16_t Target1,int16_t Target2,int16_t Target3,int16_t Target4,\
//	int16_t Target5,int16_t Target6,int16_t Target7,int16_t Target8);
void ANO_Send_Data_Init(int16_t Target1,int16_t Target2,int16_t Target3,int16_t Target4,\
		int32_t Target5,int32_t Target6,int32_t Target7,int32_t Target8);
#endif 



