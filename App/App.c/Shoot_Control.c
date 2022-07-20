#include "Shoot_Control.h"

const float SHOOT_LOW_SPEED = 3500.0f;
const float SHOOT_HIGH_SPEED = 6700.0f;
Shoot_t Shoot;

//���亯��ָ��ҹ�
Shoot_FUN_t Shoot_FUN = Shoot_HOOK_FUN; 

//����ĳ�ʼ��
void Shoot_Init(void)
{
	//ȷ��������
	Shoot.Shoot1 = &M3508s1[0];
	Shoot.Shoot2 = &M3508s1[1];
	//��ʼʱĿ���ٶ�Ϊ0
	Shoot.Shoot1->targetSpeed = 0;
	Shoot.Shoot2->targetSpeed = 0;
	//��ʱ�ٶ�ҲΪ0 
	Shoot.temp_Speed = 0;
	//��ʼ��б�µĵ�ǰֵ������ֵ���Լ�б�µ��޷�ֵ
	Shoot.ramp_Speed.Current_Value = 0;
	Shoot.ramp_Speed.Rate = 5.0f;
	Shoot.ramp_Speed.Absolute_Max = 7000.0f;
	//��ʼ��pid����
	I_PID_FUN.I_PID_Parameter_Init(&Shoot.Shoot1->I_PID,20.0f,2.5f,45.0f,500.0f,0.0f,0.0f,0.85f,7000.0f,-7000.0f,16000.0f,-16000.0f);		//����ֵ16000��
	I_PID_FUN.I_PID_Parameter_Init(&Shoot.Shoot2->I_PID,20.0f,2.5f,45.0f,500.0f,0.0f,0.0f,0.85f,7000.0f,-7000.0f,16000.0f,-16000.0f);
	//��ʼ���ߵ���ģʽ���ٶ�ֵ
	Shoot.low_Speed = SHOOT_LOW_SPEED;
	Shoot.high_Speed = SHOOT_HIGH_SPEED;
}
//���亯��
void Shoot_Control(void)
{
	//�����˰չ���Ħ����ʧ��
	if ( Robots_Control.Attack_e == ak_Disable)
	{	
		//Ŀ��ֵ��0
		Shoot.Shoot1->targetSpeed = 0;
		Shoot.Shoot2->targetSpeed = 0;
		//������
		Shoot.Shoot1->OutputCurrent = 0;
		Shoot.Shoot2->OutputCurrent = 0;
		//������ʧ�ܣ���ֱ�������Ϊ0����б�µĵ�ǰ�ٶȺ�Ħ���ֵ�Ŀ���ٶȶ�������ʧ��ǰһ�̵�ֵ
		Shoot.ramp_Speed.Current_Value = 0; //Ҫ��б�µ�ǰֵ��0�������´�ʹ��ʱ�޷�б������
		//���pid�Ĺ���ֵ
		I_PID_FUN.I_PID_Parameter_Clear(&Shoot.Shoot1->I_PID);
		I_PID_FUN.I_PID_Parameter_Clear(&Shoot.Shoot2->I_PID);
		return;
	}
	else if (Robots_Control.Attack_e == cease_Fire)
	{
		Shoot.temp_Speed = 0;
	}
	//�˵�
	else if (Robots_Control.Attack_e == R_ak_Sag_Nooverheat)
	{
		Shoot.temp_Speed = Shoot.low_Speed;
	}
	//����
	else
	{
		Shoot.temp_Speed = Shoot.high_Speed;
	}

	Shoot.ramp_Speed.Target_Value = Shoot.temp_Speed;
	Shoot.ramp_Speed.Current_Value = Ramp_Function(&Shoot.ramp_Speed);

	Shoot.Shoot1->targetSpeed = -Shoot.ramp_Speed.Current_Value;
	Shoot.Shoot2->targetSpeed = Shoot.ramp_Speed.Current_Value;

	Shoot.Shoot1->OutputCurrent = I_PID_FUN.I_PID_Regulation(&Shoot.Shoot1->I_PID,Shoot.Shoot1->targetSpeed,Shoot.Shoot1->realSpeed);
	Shoot.Shoot2->OutputCurrent = I_PID_FUN.I_PID_Regulation(&Shoot.Shoot2->I_PID,Shoot.Shoot2->targetSpeed,Shoot.Shoot2->realSpeed);

}

