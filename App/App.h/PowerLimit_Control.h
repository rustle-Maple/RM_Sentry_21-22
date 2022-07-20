#ifndef POWERLIMIT_CONTROL_H
#define POWERLIMIT_CONTROL_H

#include "main.h"
#include "RM_JudgeSystem.h"
#include "User_math.h"

#define PowerLimit_Way 0

#if PowerLimit_Way == !0
#define Chassis_Open_LimitPowerBuf 190.0f			//开启功率限制的缓冲能量
#define Chassis_Danger_PowerBuf 50.0f					//底盘的缓冲能量的危险值
#define Chassis_Max_Power 30.0f								//底盘的最大功率
#define Chassis_Voltage 24.0f

//根据裁判系统的UART帧率去设置
#define Judge_Chassis_Times 0.3f 							//裁判系统与底盘的通讯周期30ms
#endif

typedef struct
{

    float SumCurrent_IN;              //输入的电流总和
    float SumCurrent_OUT;             //最后计算出可输出电流总和

		float PowerRatio_Denominator;			//功率比率的分母值
    float PowerRatio;                 //功率的比率

    float PowerBuffer;                //实时的缓冲能量
		
		float Max_Power;									//可用的实时可用的最大功率 
	
		uint16_t Power_DelayCNT;						//更新数据计时

}PowerLimit_t;
extern PowerLimit_t Chassis_PowerLimit;

void PowerLimit_Calculate(PowerLimit_t* powerlimit);
void PowerLimit_Processing(PowerLimit_t* powerlimit,float* WheelCurrent,int16_t amout);

#endif

