#include "Encoder.h"

Encoder_t Chassis_Encoder;

//获取编码器的值
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	//累计更新时间
	Chassis_Encoder->Update_times ++;
	
	//得到AB相的值
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	//过零处理
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	//离线检查
	if(Robots_Control.Chassis_e == A_cs_Cruise || Robots_Control.Chassis_e == A_cs_Random)
	{
		if(Chassis_Encoder->Update_times > 5000)
		{
			if(Chassis_Encoder->lastValue_AB == Chassis_Encoder->realValue_AB)
			{
				Chassis_Encoder->Offline_flag = 1;				//是否是一出现一次离线就限位已经不正确了
				Chassis_Encoder->Update_times = 0;
			}
			else
			{
//				Chassis_Encoder->Offline_flag = 0;
				Chassis_Encoder->Update_times = 0;
			}
		}
	}
	else				//不知道为何，若不加，一上电就会直接执行if中内容
	{
		Chassis_Encoder->Update_times = 0;
	}
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
		
}




