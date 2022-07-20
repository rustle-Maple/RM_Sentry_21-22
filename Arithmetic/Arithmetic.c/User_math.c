#include "User_math.h"

//绝对值限制
//参数1：需要限制值的变量指针
//参数2：限制的极限值
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

	//多个极值限幅，可以更加安全一点点
	Absolute_Value_Limit(&data->Current_Value,data->Absolute_Max);

	return  data->Current_Value;
}



