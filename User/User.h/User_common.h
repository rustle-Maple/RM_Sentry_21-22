
//主要包含中断控制函数中需要用到的头文件，
//由于是整体控制函数，要用的头文件也较多，所以包含起来较美观
//其他模块APP控制函数，其实无需用到这么多个头文件，那就不要去引用这次文件，造成没有必要的导入

#ifndef USER_COMMON_H
#define USER_COMMON_H

#include "user_CAN.h"
#include "M3508.h"
#include "GM6020.h"
#include "DR16.h"
#include "Debug_DataScope.h"
#include "Vision.h"
#include "user_UART.h"
#include "Chassis_Control.h"
#include "Cloud_Control.h"
#include "Vision_Control.h"
#include "M2006.h"
#include "Shoot_Control.h"
#include "RM_JudgeSystem.h"
#include "ANO.h"
#include "tim.h"
#include "PID_Increment.h"
#include "Vision_Control.h"
#include "DJI_C_IMU.h"
#include "Broad_communicate.h"
#include "Attack_Control.h"
#include "Sensor.h"


#endif


