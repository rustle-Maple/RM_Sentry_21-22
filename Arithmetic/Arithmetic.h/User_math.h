#ifndef USER_MATH_H
#define USER_MATH_H

#include "main.h"

//ȡ����ֵ
#define abs(x) ((x)>0?(x):-(x))

//б�½ṹ��
typedef struct
{
	float Target_Value;					//б�µ�Ŀ��ֵ
	float Current_Value;				//б�µĵ�ǰֵ
	float Rate;							//б�µ�����
	float Absolute_Max;					//����ֵ���������ֵ���Ը��Ӱ�ȫһ��㣩
	
}Ramp_Struct;

void Absolute_Value_Limit(float* value,float limit_max_value);

void Value_Limit(float* value,float* min,float* max);
float Ramp_Function(Ramp_Struct* data);

#endif



