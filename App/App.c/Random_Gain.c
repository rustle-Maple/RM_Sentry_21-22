#include "Random_Gain.h"

//RNG ��Ӳ������������� ���ṩ��32λ�������
//�������������֮��ļ����40��PLL48CLKʱ���ź�����
//����������������������������SR�Ĵ�����ȥ��������״̬
//���䷢�����󣬻����Բ����ж�
//������Ҫ�����ſ���ȥ��ȡ���������Ҫ���������Ӳ���
//ȡһ����Χ�ڵ����������ͨ������rangeȡ�෨
//range(min,max) X % (max-min+1)+min ����range(min,max)�ڵ������
//��ȡһ����Χ�ڵ������
uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;
	//�������״̬λ�����жϣ���Ϊ�ڻ�ȡ������ĺ����У�����ʹ��м��
	//	if(__HAL_RNG_GET_FLAG(&hrng,RNG_FLAG_DRDY) == SET)
	//	{}
	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}
