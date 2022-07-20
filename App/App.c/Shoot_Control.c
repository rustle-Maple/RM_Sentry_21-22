#include "Shoot_Control.h"

const float SHOOT_LOW_SPEED = 3500.0f;
const float SHOOT_HIGH_SPEED = 6700.0f;
Shoot_t Shoot;

//发射函数指针挂钩
Shoot_FUN_t Shoot_FUN = Shoot_HOOK_FUN; 

//发射的初始化
void Shoot_Init(void)
{
	//确定发射电机
	Shoot.Shoot1 = &M3508s1[0];
	Shoot.Shoot2 = &M3508s1[1];
	//初始时目标速度为0
	Shoot.Shoot1->targetSpeed = 0;
	Shoot.Shoot2->targetSpeed = 0;
	//临时速度也为0 
	Shoot.temp_Speed = 0;
	//初始化斜坡的当前值，比率值，以及斜坡的限幅值
	Shoot.ramp_Speed.Current_Value = 0;
	Shoot.ramp_Speed.Rate = 5.0f;
	Shoot.ramp_Speed.Absolute_Max = 7000.0f;
	//初始化pid参数
	I_PID_FUN.I_PID_Parameter_Init(&Shoot.Shoot1->I_PID,20.0f,2.5f,45.0f,500.0f,0.0f,0.0f,0.85f,7000.0f,-7000.0f,16000.0f,-16000.0f);		//极限值16000多
	I_PID_FUN.I_PID_Parameter_Init(&Shoot.Shoot2->I_PID,20.0f,2.5f,45.0f,500.0f,0.0f,0.0f,0.85f,7000.0f,-7000.0f,16000.0f,-16000.0f);
	//初始化高低速模式的速度值
	Shoot.low_Speed = SHOOT_LOW_SPEED;
	Shoot.high_Speed = SHOOT_HIGH_SPEED;
}
//发射函数
void Shoot_Control(void)
{
	//机器人罢工、摩擦轮失能
	if ( Robots_Control.Attack_e == ak_Disable)
	{	
		//目标值置0
		Shoot.Shoot1->targetSpeed = 0;
		Shoot.Shoot2->targetSpeed = 0;
		//电机输出
		Shoot.Shoot1->OutputCurrent = 0;
		Shoot.Shoot2->OutputCurrent = 0;
		//由于是失能，是直接让输出为0，而斜坡的当前速度和摩擦轮的目标速度都保留着失能前一刻的值
		Shoot.ramp_Speed.Current_Value = 0; //要将斜坡当前值置0，否则下次使能时无法斜坡增速
		//清除pid的过程值
		I_PID_FUN.I_PID_Parameter_Clear(&Shoot.Shoot1->I_PID);
		I_PID_FUN.I_PID_Parameter_Clear(&Shoot.Shoot2->I_PID);
		return;
	}
	else if (Robots_Control.Attack_e == cease_Fire)
	{
		Shoot.temp_Speed = 0;
	}
	//退弹
	else if (Robots_Control.Attack_e == R_ak_Sag_Nooverheat)
	{
		Shoot.temp_Speed = Shoot.low_Speed;
	}
	//拉满
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

