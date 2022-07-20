#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include "main.h"
#include "Kalman.h"
#include "Vision.h"
#include "PID_Position.h"
#include "User_math.h"
#include "User_typedef.h"

#define Bullet_Velocity 28.0f

//���ٶȽṹ��
typedef struct
{
	int delay_cnt;		 //��ʱ��ʱ��
	float Position_Last; //��ʱ�̵�λ��
	float Time_Last;	 //��ʱ�̵�ʱ��
	float Omiga;		 //���ٶ�
} Omiga_data_t;
//�Ǽ��ٶȽṹ��
typedef struct
{
	int delay_cnt;	  //��ʱ��ʱ��
	float Omiga_Last; //��ʱ�̵�λ��
	float Time_Last;  //��ʱ�̵�ʱ��
	float A_Omiga;	  //���ٶ�
} A_Omiga_data_t;
//�Ӿ�ԭδ�˲�������
typedef struct
{
	
	//Yaw��ĽǶ�/���ٶ�/�Ǽ��ٶ�
	float Yaw_Angle;
	float Yaw_Omiga;
	float Yaw_A_Omiga;

	float Yaw_Last_Angle;
	//yaw��ĽǶȱ仯��
	float Yaw_Delta_Angle;

	//Pitch��
	float Pitch_Angle;
	float Pitch_Omiga;
	float Pitch_A_Omiga;

	float Pitch_Last_Angle;
	float Pitch_Delta_Angle;

	//���
	float Depth;

	float Last_Depth;

	//�������ٶȻ���ٶȵ�ʱ�ӽ��ı���(��ʹÿһ֡���ݳɹ�����������Ҫ��ʱ��)
	TickType_t WorldTimes;

	//Yaw
	//���ٶȽṹ��
	Omiga_data_t Yaw_Omiga_data;
	//�Ǽ��ٶȽṹ��
	A_Omiga_data_t Yaw_A_Omiga_data;
	//Pitch
	Omiga_data_t Pitch_Omiga_data;
	A_Omiga_data_t Pitch_A_Omiga_data;

} Vision_RawData_t;

//�˲�����������
typedef struct
{
	float Raw_yaw;
	float Raw_pitch;
	
	//Yaw��
	float *Yaw_Datas;
	float Yaw_Angle;
	float Yaw_Omiga;
	//Pitch��
	float *Pitch_Datas;
	float Pitch_Angle;
	float Pitch_Omiga;
	//���
	float Depth;
	//kalman�˲���ʱ��
	uint32_t Kalman_Delay;
} Vision_FilterData_t;

//��������
typedef struct
{
	//����������ʱ��
	uint16_t Kalman_Delay_Open;
	//������������С�ٶ�
	float Yaw_Omiga_Min;
	//��������������ٶ�
	float Yaw_Omiga_Max;
	float Pitch_Omiga_Min;
	float Pitch_Omiga_Max;

	//����ϵ��
	float Velocity_Factor;
	float Gravity_Factor;
	float A_Velocity_Factor;
	float A_Gravity_Factor;

	//��������ֵ
	float Velocity_result;
	float Gravity_result;
	float Velocity_Min; //�ٶȲ�������Сֵ
	float Velocity_Max;
	float Gravity_Min; //������������Сֵ
	float Gravity_Max;

	//б��
	Ramp_Struct Velocity_Ramp;
	Ramp_Struct Gravity_Ramp;

	//�����룩�ӵ�����ʱ��
	float Bullet_Times;
	//�Ƕȵı仯������ֵ
	float Yaw_DeltaAngle_Max;
	float Pitch_DeltaAngle_Max;

} Compensate_Data_t;
extern Compensate_Data_t IMU_Compensate_Data;
extern Compensate_Data_t EM_Compensate_Data;

//�Ӿ���������
typedef struct
{
	//�Ӿ�ԭʼ����
	Vision_RawData_t Vision_RawData;
	//�Ӿ��˲�������
	Vision_FilterData_t Vision_FilterData;
	//��������
	Compensate_Data_t *Compensate_Data;
} VisionData_Hand_t;

//����������̨��������
typedef struct
{
	//����������̨���PID����ĽǶ�����
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
//�Ӿ����ƽṹ��

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
