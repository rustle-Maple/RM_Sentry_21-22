#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include "main.h"
#include "Kalman.h"
#include "Vision.h"
#include "PID_Position.h"
#include "User_math.h"
#include "User_typedef.h"

#define Bullet_Velocity 28.0f

//角速度结构体
typedef struct
{
	int delay_cnt;		 //延时的时间
	float Position_Last; //上时刻的位置
	float Time_Last;	 //上时刻的时间
	float Omiga;		 //角速度
} Omiga_data_t;
//角加速度结构体
typedef struct
{
	int delay_cnt;	  //延时的时间
	float Omiga_Last; //上时刻的位置
	float Time_Last;  //上时刻的时间
	float A_Omiga;	  //角速度
} A_Omiga_data_t;
//视觉原未滤波的数据
typedef struct
{
	
	//Yaw轴的角度/角速度/角加速度
	float Yaw_Angle;
	float Yaw_Omiga;
	float Yaw_A_Omiga;

	float Yaw_Last_Angle;
	//yaw轴的角度变化量
	float Yaw_Delta_Angle;

	//Pitch轴
	float Pitch_Angle;
	float Pitch_Omiga;
	float Pitch_A_Omiga;

	float Pitch_Last_Angle;
	float Pitch_Delta_Angle;

	//深度
	float Depth;

	float Last_Depth;

	//卡尔曼速度或加速度的时钟节拍变量(即使每一帧数据成功更新所有需要的时间)
	TickType_t WorldTimes;

	//Yaw
	//角速度结构体
	Omiga_data_t Yaw_Omiga_data;
	//角加速度结构体
	A_Omiga_data_t Yaw_A_Omiga_data;
	//Pitch
	Omiga_data_t Pitch_Omiga_data;
	A_Omiga_data_t Pitch_A_Omiga_data;

} Vision_RawData_t;

//滤波处理后的数据
typedef struct
{
	float Raw_yaw;
	float Raw_pitch;
	
	//Yaw轴
	float *Yaw_Datas;
	float Yaw_Angle;
	float Yaw_Omiga;
	//Pitch轴
	float *Pitch_Datas;
	float Pitch_Angle;
	float Pitch_Omiga;
	//深度
	float Depth;
	//kalman滤波的时间
	uint32_t Kalman_Delay;
} Vision_FilterData_t;

//补偿数据
typedef struct
{
	//开启补偿的时间
	uint16_t Kalman_Delay_Open;
	//补偿开启的最小速度
	float Yaw_Omiga_Min;
	//补偿开启的最大速度
	float Yaw_Omiga_Max;
	float Pitch_Omiga_Min;
	float Pitch_Omiga_Max;

	//补偿系数
	float Velocity_Factor;
	float Gravity_Factor;
	float A_Velocity_Factor;
	float A_Gravity_Factor;

	//补偿最终值
	float Velocity_result;
	float Gravity_result;
	float Velocity_Min; //速度补偿的最小值
	float Velocity_Max;
	float Gravity_Min; //重力补偿的最小值
	float Gravity_Max;

	//斜坡
	Ramp_Struct Velocity_Ramp;
	Ramp_Struct Gravity_Ramp;

	//（理想）子弹运行时间
	float Bullet_Times;
	//角度的变化量极限值
	float Yaw_DeltaAngle_Max;
	float Pitch_DeltaAngle_Max;

} Compensate_Data_t;
extern Compensate_Data_t IMU_Compensate_Data;
extern Compensate_Data_t EM_Compensate_Data;

//视觉处理数据
typedef struct
{
	//视觉原始数据
	Vision_RawData_t Vision_RawData;
	//视觉滤波后数据
	Vision_FilterData_t Vision_FilterData;
	//补偿数据
	Compensate_Data_t *Compensate_Data;
} VisionData_Hand_t;

//最终用于云台控制数据
typedef struct
{
	//最终用于云台电机PID运算的角度增量
	float Yaw_FinalAngle;
	float Pitch_FinalAngle;
	
	struct IMU_pid_t
	{
		P_PID_t Yaw_A_pid;
		P_PID_t Yaw_S_pid;
		P_PID_t Pitch_A_pid;
		P_PID_t Pitch_S_pid;
	} IMU_pid_t;
	struct EM_pid_t
	{
		P_PID_t Yaw_A_pid;
		P_PID_t Yaw_S_pid;
		P_PID_t Pitch_A_pid;
		P_PID_t Pitch_S_pid;
	} EM_pid_t;

	P_PID_t *Yaw_A_pid;
	P_PID_t *Yaw_S_pid;
	P_PID_t *Pitch_A_pid;
	P_PID_t *Pitch_S_pid;
} Vision_Ctrl_t;
//视觉控制结构体

extern VisionData_Hand_t VisionData_Hand;
extern Vision_Ctrl_t Vision_Ctrl;

float Calculate_Omiga(Omiga_data_t *Omiga_data, float Position, uint32_t Times);
float Calculate_A_Omiga(A_Omiga_data_t *A_Omiga_data, float Omiga, uint32_t Times);
void VisionData_Hand_VA_Calculate(void);
void Updata_VisionData_Hand(void);
void VisionData_Hand_Smooth_Processing(void);
void Clear_VisionData_Processing(void);
void Vision_Control_Init(void);
void Vision_Control_Cloud(void);

#endif
