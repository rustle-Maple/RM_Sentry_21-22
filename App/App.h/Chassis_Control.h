#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "math.h"

#include "main.h"
#include "User_typedef.h"
#include "Robots_Control.h"
#include "PID_Position.h"
#include "PID_Increment.h"
#include "M3508.h"
#include "Encoder.h"
#include "PEswitch.h"
#include "Random_Gain.h"
#include "User_math.h"
#include "Limit_switch.h"
#include "Vision_Control.h"
#include "DJI_C_IMU.h"
#include "PEswitch.h"
#include "RM_JudgeSystem.h"
#include "PowerLimit_Control.h"

#define Debug_Location_pid 0
#define Pures_Debug 0

#define Chassis_HOOK_FUN {\
	EM_Purse_Init,\
	Updata_Chassis_Sensor,\
	Route_limit_Processing,\
	Outpost_survive_Processing,\
	Cruise_Processing,\
	Index_Parameter_Calculate,\
	Index_VariableSpeed,\
	Random_Processing,\
	Frenzy_Processing,\
	Optimally_Attack_Distance,\
	Pursue_Processing,\
	MecanumCalculate,\
	Chassis_Init,\
	Chassis_Control,\
}

#define Chassis_MaxSpeed_Wheel 8000.0f
#define Chassis_MaxSpeed_X 8000.0f
#define Chassis_MaxSpeed_Z 8000.0f

extern uint8_t B0_flag;

//底盘位置（是根据位置差来计算的）结构体
typedef struct
{
    float reference; //参考位置
    float real; //实时位置
    float target; //目标位置 （不要用线数，线数是int类型，用于pid计算，用float更好，所以直接算出出距离）
    P_PID_t* P_PID; //位置式pid
} Location_t;
typedef struct 
{
	float targetXRaw;		//底盘x轴原始数据
	float targetZRaw;		//底盘z轴原始数据
	float LpfAttFactor;		//底盘滤波系数
	float targetXLPF;		//底盘x轴滤波后数据
	float targetZLPF;		//底盘z轴滤波后数据
    float Calcu_Speed[1];   //胶轮解算后的速度
    float temp_Speed;       //临时速度变量
    I_PID_t* I_PID;         //增量式pid
    P_PID_t* P_PID;         //位置式pid
}Velocity_t;
//行程限制
typedef struct 
{
    uint8_t flag; //行程限制成功标志位
    float left; //左极限
    float right; //右极限
		uint8_t right_flag;
		uint8_t left_flag;
		uint32_t  time;
}Route_limit_t;
//指数模型变速运动结构体
typedef struct 
{
	uint16_t Lower_Limit;	//下限值
	uint16_t Upper_Limit;	//上限值
	uint16_t Increment_times;	//增量时间
	float Accelerate_base;	//加速底数
	float Decelerate_base;	//减速底数
	int16_t times;				//运行时间
	float abs_Vel;				//速度绝对值
}Index_VarSpe_t;
//随机运动结构体
typedef struct 
{
	  uint16_t Dir_number; //方向随机数
		uint16_t Time_number;	//时间随机数
		uint16_t Mode_number;	//运行模式随机数
		
		uint16_t Dir_times;  //方向时间
		uint16_t Time;	//总运行时间
		
		int8_t Dir;		 //方向
		float percent;
	
		uint8_t Break_flag;	//刹车标志位
	/*【21赛季】
    uint32_t number; //随机数
    uint16_t sampling; //扫描时间
	*/
}Random_t;
typedef struct
{
	uint8_t Right_flag;
	uint8_t Left_flag;
	uint16_t Right_times;
	uint16_t Left_times;
}CDisable_t;
//暴走运动结构体
typedef struct
{
    uint8_t just_flag; //刚进入狂暴状态
    Ramp_Struct ramp_speed;
}Frenzy_t;
//追击运动结构体
typedef struct
{
    float *real_angle; //当前角度
    float refer_angle; //参考角度
    float delta_angle; //增量角度
    float unit_conversion; //单位转换
    float angle; //用于cos计算的角度
    float delta_distance; //增量距离
}Purse_t;
//extern Purse_t IMU_Purse; //陀螺仪的追击变量 (发现好像直接用机械角度去算角度偏差值就行，不用陀螺仪了)
extern Purse_t EM_Purse; //电机的追击变量
//底盘结构体
typedef struct
{
    M3508_t* EM;
    Encoder_t* encoder;
    Velocity_t Velocity;
    Location_t Location;
    Route_limit_t Route_limit;
		Index_VarSpe_t Index_VarSpe;
    Random_t Random;
		CDisable_t CDisable;
    Frenzy_t Frenzy;
    Purse_t* Purse;
		float Power_Currents[4];						//功率限制的电流值
} Chassis_t;
extern Chassis_t Chassis;

typedef struct
{
	void (*EM_Purse_Init)(void);
	void (*Updata_Chassis_Sensor)(void);
	void (*Route_limit_Processing)(void);
	void (*Outpost_survive_Processing)(void);
	void (*Cruise_Processing)(void);
	void (*Index_Parameter_Calculate)(void);
	void (*Index_VariableSpeed)(void);
	void (*Random_Processing)(void);
	void (*Frenzy_Processing)(void);
	void (*Optimally_Attack_Distance)(float real_Angle, float real_depth, float *b);
	void (*Pursue_Processing)(void);
	void (*MecanumCalculate)(float Vx, float V0mega, float *speed);
	void (*Chassis_Init)(void);
	void (*Chassis_Control)(void);
}Chassis_FUN_t;
extern Chassis_FUN_t Chassis_FUN;

void EM_Purse_Init(void);
void Updata_Chassis_Sensor(void);
void Route_limit_Processing(void);
void Outpost_survive_Processing(void);
void Cruise_Processing(void);
void Index_Parameter_Calculate(void);
void Index_VariableSpeed(void);
void Random_Processing(void);
void Prevent_Cloud_Breakdown(void);
void exclude_B0_disturb(void);
void Frenzy_Processing(void);
void Optimally_Attack_Distance(float real_Angle, float real_depth, float *b);
void Pursue_Processing(void);
void MecanumCalculate(float Vx, float V0mega, float *speed);
void Chassis_Init(void);
void Chassis_Control(void);


#endif
