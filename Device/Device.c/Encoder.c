#include "Encoder.h"

Encoder_t Chassis_Encoder;

//��ȡ��������ֵ
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	//�ۼƸ���ʱ��
	Chassis_Encoder->Update_times ++;
	
	//�õ�AB���ֵ
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	//���㴦��
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	//���߼��
	if(Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
	{
		if(Chassis_Encoder->Update_times > 5000)
		{
			if(Chassis_Encoder->lastValue_AB == Chassis_Encoder->realValue_AB)
			{
				Chassis_Encoder->Offline_flag = 1;				//�Ƿ���һ����һ�����߾���λ�Ѿ�����ȷ��
				Chassis_Encoder->Update_times = 0;
			}
			else
			{
//				Chassis_Encoder->Offline_flag = 0;
				Chassis_Encoder->Update_times = 0;
			}
		}
	}
	else				//��֪��Ϊ�Σ������ӣ�һ�ϵ�ͻ�ֱ��ִ��if������
	{
		Chassis_Encoder->Update_times = 0;
	}
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
		
}




