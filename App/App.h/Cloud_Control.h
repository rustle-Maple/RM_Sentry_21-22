#ifndef CLOUD_CONTROL_H
#define CLOUD_CONTROL_H

#include "User_typedef.h"
#include "main.h"
#include "DR16.h"
#include "GM6020.h"
#include "Filter_fuction.h"
#include "Math.h"
#include "Vision.h"
#include "Robots_Control.h"
#include "Wolf_GyIMU.h"
#include "PID_Position.h"
#include "RM_JudgeSystem.h"
#include "Vision_Control.h"
#include "DJI_C_IMU.h"

#define Cloud_Debug_PID 0

#define CLOUD_HOOK_FUN {\
	Cloud_IMUlimit_Calculate,\
	IMU_Scan_Init,\
	EM_Scan_Init,\
	Cloud_PID_Init,\
	Radian_Gain,\
	Scan_Processing,\
	Cloud_IMU_Init,\
	Cloud_EM_Init,\
	Cloud_Emergency_Processing,\
	Cloud_Init,\
	Cloud_Control\
}
//云台的控制模式
typedef enum
{
	Nothing = 0,
	IMU = 1,
	EM
} control_way_e;
//限幅结构体
typedef struct
{
	float low;
	float high;
	float centre;
	float left;
	float right;
} Limit_t;
extern Limit_t IMU_limit;
extern Limit_t EM_limit;
//扫描结构体
typedef struct
{
	float *total;			//现在处(机械)角度值
	float *low;				//下极限
	float *high;			//上极限
	float *centre;			//中心值
	float radian;			//弧度值（自变量）
	int8_t dir_pitch;		//方向
	float unit_incre_picth; //单位增量

	float unit_incre_yaw;
	int8_t dir_yaw;
	int32_t *count_yaw; //过零的圈数
	float *left;		//左极限
	float *right;		//右极限

	float unit_value; //（机械）角度单位圈数值

} Scan_t;
extern Scan_t IMU_scan;
extern Scan_t EM_scan;

typedef struct
{
	//控制云台的方式
	control_way_e real_way;
	control_way_e last_way;
	uint8_t way_change_flag;

	float targetYawRaw;	  //云台Yaw轴原始数据
	float targetPitchRaw; //底盘Yaw轴滤波后数据
	float LpfAttFactor;	  //云台滤波系数
	float targetYawLPF;	  //云台Yaw轴滤波后数据
	float targetPitchLPF; //云台Pitch轴滤波后数据

	//测量值(防炸)
	float* yaw_total;
	float* pitch_total;
	float* gyro_z;
	float* gyro_y;

	//输出
	float yaw_output;
	float pitch_output;

	//云台电机指针
	GM6020_t *Yaw;
	GM6020_t *Pitch;
	//云台陀螺仪指针
	DJIC_IMU_t *IMU;
	//限幅的结构体指针
	Limit_t *limit;
	//扫描的结构体指针
	Scan_t *scan;
	//PID
	P_PID_t *Yaw_Angle_pid;
	P_PID_t *Yaw_Speed_pid;
	P_PID_t *Pitch_Angle_pid;
	P_PID_t *Pitch_Speed_pid;

} Cloud_t;
extern Cloud_t Cloud;
typedef struct 
{
	//获取陀螺仪的上下限幅值
	void (*Cloud_IMUlimit_Calculate)(void);
	//IMU扫描初始化函数指针
	void (*IMU_Scan_Init)(void);
	//EM扫描初始化函数指针
	void (*EM_Scan_Init)(void);
	//PID初始化
	void (*Cloud_PID_Init)(void);
	//获取弧度函数指针
	void (*Radian_Gain)(void);
	//扫描执行函数指针
	void (*Scan_Processing)(void);
	//IMU控制下的初始化函数指针
	void (*Cloud_IMU_Init)(void);
	//EM控制下的初始化函数指针
	void (*Cloud_EM_Init)(void);
	//云台应急处理
	void (*Cloud_Emergency_Processing)(void);
	//Cloud初始化
	void (*Cloud_Init)(void);
	//Cloud执行
	void (*Cloud_Control)(void);
}Cloud_FUN_t;
extern Cloud_FUN_t Cloud_FUN;

#endif
