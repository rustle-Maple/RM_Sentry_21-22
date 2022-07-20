
//��ʵ�Ҿ������typedef ��Ҫ�������û��Զ���� rtos������ĸ�������
//����ģ����û��Զ���ı���/�꣬��ʵ���ڸ�ģ���.h�ļ��У��������߼�������

#ifndef USER_TYPEDEF_H
#define USER_TYPEDEF_H

#include "main.h"

#define Down_Cloud_Enable 1
#define Up_Cloud_Enable 0

#define Judge_Enable 1

#define pai 3.1415926535897932384626433832795f

#define Coded_Contact_Angle 22.755555555555555555555555555556f      //��λ�Ƕȶ�Ӧ������ֵ
#define Coded_Contact_Radian 1303.7972938088065906186957895476f     //��λ���ȶ�Ӧ������ֵ
#define Angle_Contact_Radian 57.295779513082320876798154814105f     //��λ���ȶ�Ӧ�ĽǶ�
#define EM_Unit 8192.0f
#define Angle_Unit 360.0f
#define Radian_Unit 6.283185307179586476925286766559f



extern uint32_t WorldTimes;
//����ʱ�ӽṹ��
typedef struct 
{
    uint32_t WorldTime;             //����ʱ��
    uint32_t Last_WorldTime;        //��һ��ʱ��
}WorldTime_RxTypedef;
//֡�ʼ���
//֡�ʽṹ��
typedef struct
{
	WorldTime_RxTypedef times;
	uint32_t FPS;
	uint32_t Offline_Detec;		//���߼��
	uint8_t Offline_Flag;			//���߱�־λ
} Frame_rate_t;

void Robots_Detection_processing(void);
void Get_FPS(WorldTime_RxTypedef *time,uint32_t* FPS);


#endif


