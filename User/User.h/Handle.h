#ifndef HANDLE_H
#define HANDLE_H

#include "FreeRTOS.h"
#include "queue.h"

//CAN���ݶ��о��
extern QueueHandle_t CAN1_Queue;
extern QueueHandle_t CAN2_Queue;

//��������Ĳ���
//��ջ��С
#define RobotCtrl_Size 512
//���ȼ�
#define RobotCtrl_Priority osPriorityRealtime
//������
extern TaskHandle_t RobotCtrl_Handle;
//�����������
extern void Robot_Control(void const * argument);

#endif


