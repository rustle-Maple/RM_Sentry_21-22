#include "Filter_fuction.h"

//IIR��ͨ�˲������������������ͬһ������
//����1���������� Ҫ�ﵽ��Ŀ��ֵ
//����2��������� 
//����3����ͨ�˲�˥������ should be between 0 to 1.
void Filter_IIRLPF(float *in,float *out, float LpfAttFactor)
{
	*out = *out + LpfAttFactor*(*in - *out); 
}

