#ifndef HANDLE_H
#define HANDLE_H

#include "FreeRTOS.h"
#include "queue.h"

//CAN数据队列句柄
extern QueueHandle_t CAN1_Queue;
extern QueueHandle_t CAN2_Queue;

//控制任务的参数
//堆栈大小
#define RobotCtrl_Size 512
//优先级
#define RobotCtrl_Priority osPriorityRealtime
//任务句柄
extern TaskHandle_t RobotCtrl_Handle;
//控制任务入口
extern void Robot_Control(void const * argument);

#endif


