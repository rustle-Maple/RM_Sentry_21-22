#include "PEswitch.h"

PSwitch_t PSwitch_FLAG;


//��ȡ��翪�ص�ֵ

void Get_PSwitch_FLAG(void)
{
	//���������ҹ��λ����ôװ��ֻҪ��������������λ�þͿ���
	//����һ�㣬zjunʦ�ֵĴ��������������룬�������Ǹ�������
	//�ȿ��˹��˵���飬��˵
	PSwitch_FLAG.PSwitch_R = HAL_GPIO_ReadPin(PEswitch_1_GPIO_Port,PEswitch_1_Pin);
	PSwitch_FLAG.PSwitch_L = HAL_GPIO_ReadPin(PEswitch_2_GPIO_Port,PEswitch_2_Pin);
	
}


