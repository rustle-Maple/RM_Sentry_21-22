
//其实我觉得这个typedef 主要是用于用户自定义的 rtos的任务的各个参数
//而各模块的用户自定义的变量/宏，其实放在各模块的.h文件中，更加有逻辑，清晰

#ifndef USER_TYPEDEF_H
#define USER_TYPEDEF_H

#include "main.h"

#define Down_Cloud_Enable 1
#define Up_Cloud_Enable 0

#define Judge_Enable 1

#define pai 3.1415926535897932384626433832795f

#define Coded_Contact_Angle 22.755555555555555555555555555556f      //单位角度对应的码盘值
#define Coded_Contact_Radian 1303.7972938088065906186957895476f     //单位弧度对应的码盘值
#define Angle_Contact_Radian 57.295779513082320876798154814105f     //单位弧度对应的角度
#define EM_Unit 8192.0f
#define Angle_Unit 360.0f
#define Radian_Unit 6.283185307179586476925286766559f



extern uint32_t WorldTimes;
//世界时钟结构体
typedef struct 
{
    uint32_t WorldTime;             //世界时钟
    uint32_t Last_WorldTime;        //上一刻时钟
}WorldTime_RxTypedef;
//帧率计算
//帧率结构体
typedef struct
{
	WorldTime_RxTypedef times;
	uint32_t FPS;
	uint32_t Offline_Detec;		//离线检查
	uint8_t Offline_Flag;			//离线标志位
} Frame_rate_t;

void Robots_Detection_processing(void);
void Get_FPS(WorldTime_RxTypedef *time,uint32_t* FPS);


#endif


