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
//��̨�Ŀ���ģʽ
typedef enum
{
	Nothing = 0,
	IMU = 1,
	EM
} control_way_e;
//�޷��ṹ��
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
//ɨ��ṹ��
typedef struct
{
	float *total;			//���ڴ�(��е)�Ƕ�ֵ
	float *low;				//�¼���
	float *high;			//�ϼ���
	float *centre;			//����ֵ
	float radian;			//����ֵ���Ա�����
	int8_t dir_pitch;		//����
	float unit_incre_picth; //��λ����

	float unit_incre_yaw;
	int8_t dir_yaw;
	int32_t *count_yaw; //�����Ȧ��
	float *left;		//����
	float *right;		//�Ҽ���

	float unit_value; //����е���Ƕȵ�λȦ��ֵ

} Scan_t;
extern Scan_t IMU_scan;
extern Scan_t EM_scan;

typedef struct
{
	//������̨�ķ�ʽ
	control_way_e real_way;
	control_way_e last_way;
	uint8_t way_change_flag;

	float targetYawRaw;	  //��̨Yaw��ԭʼ����
	float targetPitchRaw; //����Yaw���˲�������
	float LpfAttFactor;	  //��̨�˲�ϵ��
	float targetYawLPF;	  //��̨Yaw���˲�������
	float targetPitchLPF; //��̨Pitch���˲�������

	//����ֵ(��ը)
	float* yaw_total;
	float* pitch_total;
	float* gyro_z;
	float* gyro_y;

	//���
	float yaw_output;
	float pitch_output;

	//��̨���ָ��
	GM6020_t *Yaw;
	GM6020_t *Pitch;
	//��̨������ָ��
	DJIC_IMU_t *IMU;
	//�޷��Ľṹ��ָ��
	Limit_t *limit;
	//ɨ��Ľṹ��ָ��
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
	//��ȡ�����ǵ������޷�ֵ
	void (*Cloud_IMUlimit_Calculate)(void);
	//IMUɨ���ʼ������ָ��
	void (*IMU_Scan_Init)(void);
	//EMɨ���ʼ������ָ��
	void (*EM_Scan_Init)(void);
	//PID��ʼ��
	void (*Cloud_PID_Init)(void);
	//��ȡ���Ⱥ���ָ��
	void (*Radian_Gain)(void);
	//ɨ��ִ�к���ָ��
	void (*Scan_Processing)(void);
	//IMU�����µĳ�ʼ������ָ��
	void (*Cloud_IMU_Init)(void);
	//EM�����µĳ�ʼ������ָ��
	void (*Cloud_EM_Init)(void);
	//��̨Ӧ������
	void (*Cloud_Emergency_Processing)(void);
	//Cloud��ʼ��
	void (*Cloud_Init)(void);
	//Cloudִ��
	void (*Cloud_Control)(void);
}Cloud_FUN_t;
extern Cloud_FUN_t Cloud_FUN;

#endif
