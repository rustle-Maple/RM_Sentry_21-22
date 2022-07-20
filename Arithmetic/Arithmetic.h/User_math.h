#ifndef USER_MATH_H
#define USER_MATH_H

#include "main.h"

//取绝对值
#define abs(x) ((x)>0?(x):-(x))

//斜坡结构体
typedef struct
{
	float Target_Value;					//斜坡的目标值
	float Current_Value;				//斜坡的当前值
	float Rate;							//斜坡的增量
	float Absolute_Max;					//极限值（多个极限值可以更加安全一点点）
	
}Ramp_Struct;

void Absolute_Value_Limit(float* value,float limit_max_value);

void Value_Limit(float* value,float* min,float* max);
float Ramp_Function(Ramp_Struct* data);

#endif



