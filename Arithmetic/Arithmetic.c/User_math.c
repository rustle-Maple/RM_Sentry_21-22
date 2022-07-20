#include "User_math.h"

//����ֵ����
//����1����Ҫ����ֵ�ı���ָ��
//����2�����Ƶļ���ֵ
void Absolute_Value_Limit(float* value,float limit_max_value)
{
	if(*value > limit_max_value)
	{
		*value = limit_max_value;
	}
	else if(*value < -limit_max_value)
	{
		*value = -limit_max_value;
	}
}

void Value_Limit(float* value,float* min,float* max)
{
	if(*value <= *min)
	{
		*value = *min;
	}
	else if(*value >= *max)
	{
		*value = *max;
	}
}

float Ramp_Function(Ramp_Struct* data)
{
	
	if(data->Current_Value > data->Target_Value)
	{
		data->Current_Value -= data->Rate;
	}
	else if(data->Current_Value < data->Target_Value)
	{
		data->Current_Value += data->Rate;
	}
	else if(data->Current_Value == data->Target_Value)
	{
		data->Current_Value += 0;
	}

	//�����ֵ�޷������Ը��Ӱ�ȫһ���
	Absolute_Value_Limit(&data->Current_Value,data->Absolute_Max);

	return  data->Current_Value;
}



