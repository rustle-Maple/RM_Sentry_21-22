#include "Filter_fuction.h"

//IIR低通滤波，输入与输出不能是同一个变量
//参数1：输入数据 要达到的目标值
//参数2：输出数据 
//参数3：低通滤波衰减因子 should be between 0 to 1.
void Filter_IIRLPF(float *in,float *out, float LpfAttFactor)
{
	*out = *out + LpfAttFactor*(*in - *out); 
}

