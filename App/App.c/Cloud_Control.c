#include "Cloud_Control.h"

Cloud_t Cloud;

//�޷��ṹ��
const float EM_LIMIT_LOW = 6420.0f;
const float EM_LIMIT_High = 7110.0f;
Limit_t IMU_limit;
Limit_t EM_limit = {EM_LIMIT_LOW, EM_LIMIT_High, (float)(EM_LIMIT_LOW + EM_LIMIT_High) / 2.0f};
//ɨ��ṹ��
Scan_t IMU_scan;
Scan_t EM_scan;

void Cloud_IMUlimit_Calculate(void);
void IMU_Scan_Init(void);
void EM_Scan_Init(void);
void Cloud_PID_Init(void);
void Radian_Gain(void);
void Scan_Processing(void);
void Cloud_IMU_Init(void);
void Cloud_EM_Init(void);
void Cloud_Init(void);
void Cloud_Emergency_Processing(void);
void Cloud_Control(void);

//�ҹ�
Cloud_FUN_t Cloud_FUN = CLOUD_HOOK_FUN;

//IMU�޷��ļ���
void Cloud_IMUlimit_Calculate(void)
{
	/* ǰ�� EM �� IMU ͬ�򣬷�����ŵøı� */
	//����EM�޷���
	float delta_low, delta_high;
	delta_low = (float)fabs(Cloud.Pitch->totalAngle - EM_limit.low) / Coded_Contact_Angle;
	delta_high = (float)fabs(Cloud.Pitch->totalAngle - EM_limit.high) / Coded_Contact_Angle;
	//����EM�޷���
	if (Cloud.Pitch->totalAngle < EM_limit.low)
	{
		delta_low = 0.0f;
	}
	if (Cloud.Pitch->totalAngle > EM_limit.high)
	{
		delta_high = 0.0f;
	}
	//�õ�IMU�޷�
	IMU_limit.low = Cloud.IMU->total_pitch - delta_low;
	IMU_limit.high = Cloud.IMU->total_pitch + delta_high;
	IMU_limit.centre = (float)(IMU_limit.low + IMU_limit.high) / 2.0f;
	IMU_limit.left = 360.0f;
	IMU_limit.right = 0.0f;
}
//IMUɨ��ṹ���ʼ��
void IMU_Scan_Init(void)
{
	IMU_scan.total = &DJIC_IMU.total_pitch;
	IMU_scan.low = &IMU_limit.low;
	IMU_scan.high = &IMU_limit.high;
	IMU_scan.centre = &IMU_limit.centre;
	IMU_scan.radian = 0.0f;
	IMU_scan.dir_pitch = 1;
	IMU_scan.unit_incre_picth = (float)1.0f / Angle_Contact_Radian; //���Ա�����
	IMU_scan.unit_incre_yaw = 1.0f;
	IMU_scan.dir_yaw = 1;
	IMU_scan.count_yaw = &DJIC_IMU.yaw_turnCounts;
	IMU_scan.left = &IMU_limit.left;
	IMU_scan.right = &IMU_limit.right;
	IMU_scan.unit_value = Angle_Unit;
}
//EMɨ��ṹ���ʼ��
void EM_Scan_Init(void)
{
	EM_scan.total = &Cloud.Pitch->totalAngle;
	EM_scan.low = &EM_limit.low;
	EM_scan.high = &EM_limit.high;
	EM_scan.centre = &EM_limit.centre;
	EM_scan.radian = 0.0f;
	EM_scan.dir_pitch = 1;
	EM_scan.unit_incre_picth = (float)1.0f / Angle_Contact_Radian; //���Ա�����
	EM_scan.unit_incre_yaw = 1.0f;
	EM_scan.dir_yaw = 1;
	EM_scan.count_yaw = &Cloud.Yaw->turnCount;
	EM_scan.left = &EM_limit.left;
	EM_scan.right = &EM_limit.right;
	EM_scan.unit_value = EM_Unit;
}

//Pitch��ɨ�裬��ȡ����ֵ
//Yaw��Ҫͨ������sin�������˶�ģ��ȥ���У�Ҳ���ǲ���
void Radian_Gain(void)
{
	/*
	ֻҪ������ɨ��ģʽ�£���Ҫ���ж���̨Pitch�ỡ�ȵ�ɨ�裬��Ȼ�л�ʱ��ը��
	����һֱɨ�裬ֻҪ�˳�ɨ��״̬ʱ����Ҫɨ�赱ǰ�Ļ��ȣ�
	һ������ɨ��ģʽ����ͨ�����ƻ��ȵ�+/-�����ﵽ���Ƶ���Ļ�е�Ƕȵģ�
	����ɨ��ģʽ�£���һֱɨ�赱ǰ���ȣ��ᵼ�»��ȵ�+/-��ֵ ��ɨ��ɵ�ǰ�Ļ��ȵ�ֵ �����滻
	*/

	//��������ɨ��״̬ʱ����Ҫʱ�̻�ȡ����
	if (Robots_Control.Cloud_e != A_cd_Scan)
	{
		//��radian���ȶ�Ӧ�����������Ϊy
		double y;
		int32_t total;
		//����ɨ��ʱ��Pitch������������λ֮�������˶�
		//���Թ���һ���˶�ģ�ͣ�
		//�� totalAngel Ϊ��������Զ�Ӧ�Ļ��� radian Ϊ�Ա�����
		//����һ�����Һ���ģ�ͣ�totalAngle = Asin(b��) + c

		//���ڴ��˶�ģ�͵�ϵ���Ǹ������¼����Ƶ�������
		//����Ϊ�˷�ֹ���ת������������˶�ϵ��
		//�ȸ����޸���
		if (*(Cloud.scan->total) < *(Cloud.scan->low)) //����Pitch���
		{
			total = *(Cloud.scan->low);
		}
		else if (*(Cloud.scan->total) > *(Cloud.scan->high)) //����Pitch���
		{
			total = *(Cloud.scan->high);
		}
		else
		{
			total = *(Cloud.scan->total);
		}
		//����Ҫ�õ�����totalAngle��Ӧ�Ļ���radian������Ҫ������ȥȡ������arcsin
		//������� A = ��/�¼��� - ���� = Cloud_Pitch_High - Cloud_Pitch_Centre
		//b Ϊ��Ƶ�ʣ�ʵ����Ҳֻ��һ�������任���ѣ����˶��Ŀ�����Ϊ�˼��㷽�������Ϊ1
		//C Ϊ���� ������radianΪ0�ǣ���Ӧ��totalAngle����Ϊ c = Cloud_Pitch_Centre
		//�Ƶ����̣�ֽ�Ͻ��
		y = (double)(total - *(Cloud.scan->centre)) / abs(*(Cloud.scan->high) - *(Cloud.scan->centre));
		//�����Ҽ��ɵû���radian
		Cloud.scan->radian = asin(y);
	}
}
//ɨ�麯����ִ��
void Scan_Processing(void)
{
	float P_Kp = 0.4f, Y_Kp = 0.2f;

	//�ǵ����Ҫ�����ģ����ֺ�ɨ��ʱ�ķ���һ�£���Ȼ������л�ʱͻȻ��ը��
	Cloud.targetPitchRaw = (abs(*(Cloud.scan->high) - *(Cloud.scan->centre))) * sin(Cloud.scan->radian) + *(Cloud.scan->centre);
	//�Ա���ÿ������1�㣬ע���Ա����ġ� �� ������ġ� �ǲ�ͬ��
	Cloud.scan->radian += Cloud.scan->dir_pitch * Cloud.scan->unit_incre_picth * P_Kp;
	//Ϊ�˷�ֹCloud.Scan_Pitch_Radian���������������Cloud.Scan_Pitch_Dir�ķ������б�Ҫ��һ��
	//��Ȼ���÷����Ա���radianһֱ�ӣ�Ҳ����ʵ�������totalAngel �˶��켣Ϊ sin
	//��ʵ����������Ҳû��ʲô���⣬��ΪֻҪ�㲻����ɨ��ģʽ���Ա���radian�ͻᱻ����
	if (Cloud.scan->radian > Cloud.scan->unit_incre_picth * Angle_Unit * 5) //��ͬ������ת10Ȧ����ı䷽��
	{
		Cloud.scan->dir_pitch = -1;
	}
	else if (Cloud.scan->radian < (-Cloud.scan->unit_incre_picth) * Angle_Unit * 5)
	{
		Cloud.scan->dir_pitch = 1;
	}
	//��Yaw������sin�˶�ģʽ���Ǿ�ֱ�ӿ���targetAngle����һ������ֵ����
	Cloud.targetYawRaw += Cloud.scan->unit_incre_yaw * Cloud.scan->dir_yaw * Y_Kp;
	//��Yawɨ��ʱ�����Է�ֹһֱ��һ������ת�����������
	if (Cloud.targetYawRaw - (*(Cloud.scan->count_yaw) * Cloud.scan->unit_value) > *(Cloud.scan->left))
	{
		Cloud.scan->dir_yaw = -1;
	}
	else if (Cloud.targetYawRaw - (*(Cloud.scan->count_yaw) * Cloud.scan->unit_value) < *(Cloud.scan->right))
	{
		Cloud.scan->dir_yaw = 1;
	}
}
void Cloud_PID_Init(void)
{
	/* ע������������ȷ��kp/ki/kd���� ע��𱻵ֿ�����ƭ�� */
	//IMU pid��ʼ��
	//yaw : angle gyro
	//�ǶȻ����ٶȻ������������෴
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.yaw_pid, -12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	//�ٶȻ��������������������ͬ
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.gyro_z_pid, 800.0f, 10.5f, 1200.0f, 80.0f, 0.0f, 0.0f, 0.85f, 5000.0f, -5000.0, 28888.0f, -28888.0f);
	//pitch: angle gyro
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.pitch_pid, 25.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 300.0f, -300.0f);
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.gyro_y_pid, 340.0f, 3.2f, 0.0f, 300.0f, 0.0f, 0.0f, 0.85f, 3000.0f, -3000.0, 28888.0f, -28888.0f);
	//EM
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Yaw->P_AnglePID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Yaw->P_SpeedPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 10000.0f, -10000.0, 28888.0f, -28888.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Pitch->P_AnglePID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Pitch->P_SpeedPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 10000.0f, -10000.0, 28888.0f, -28888.0f);
	//�Ӿ�
	/* ... */
}
//IMU�����µ�cloud�ĳ�ʼ��
void Cloud_IMU_Init(void)
{
	//����̨����λ��ɨ��ֵ �� ΪIMU��
	Cloud.limit = &IMU_limit;
	Cloud.scan = &IMU_scan;
	//PID��ȷ��
	Cloud.Yaw_Angle_pid = &DJIC_IMU.yaw_pid;
	Cloud.Yaw_Speed_pid = &DJIC_IMU.gyro_z_pid;
	Cloud.Pitch_Angle_pid = &DJIC_IMU.pitch_pid;
	Cloud.Pitch_Speed_pid = &DJIC_IMU.gyro_y_pid;
	//��ʼ����ǰ�Ƕ�
	Cloud.yaw_total = &Cloud.IMU->total_yaw;
	Cloud.pitch_total = &Cloud.IMU->total_pitch;
	//��ʼ��Ŀ��ֵ��ֹը��
	Cloud.targetYawRaw = *Cloud.yaw_total;
	Cloud.targetPitchRaw = *Cloud.pitch_total;
	//��ʼ����ǰ�ٶ�ֵ
	Cloud.gyro_z = &Cloud.IMU->Gyro_z;
	Cloud.gyro_y = &Cloud.IMU->Gyro_y;
}
//EM������CLoud�ĳ�ʼ��
void Cloud_EM_Init(void)
{
	Cloud.limit = &EM_limit;
	Cloud.scan = &EM_scan;
	Cloud.Yaw_Angle_pid = &Cloud.Yaw->P_AnglePID;
	Cloud.Yaw_Speed_pid = &Cloud.Yaw->P_SpeedPID;
	Cloud.Pitch_Angle_pid = &Cloud.Pitch->P_AnglePID;
	Cloud.Pitch_Speed_pid = &Cloud.Pitch->P_SpeedPID;
	Cloud.yaw_total = &Cloud.Yaw->totalAngle;
	Cloud.pitch_total = &Cloud.Pitch->totalAngle;
	Cloud.targetYawRaw = *Cloud.yaw_total;
	Cloud.targetPitchRaw = *Cloud.pitch_total;
	Cloud.gyro_z = (float *)&Cloud.Yaw->freadSpeed; //ע��ǿ������ת��
	Cloud.gyro_y = (float *)&Cloud.Pitch->freadSpeed;
}
//��̨Ӧ������
void Cloud_Emergency_Processing(void)
{
	if (Cloud.way_change_flag == 1)
	{
		//�������ʱ�̵�PID����
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch��
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Speed_pid);
		//�ٳ�ʼ��
		if (Cloud.real_way == IMU)
		{
			Cloud_FUN.Cloud_IMU_Init();
		}
		else if (Cloud.real_way == EM)
		{
			Cloud_FUN.Cloud_EM_Init();
		}
		//������֮���ٽ��任��־λ��0
		Cloud.way_change_flag = 0;
	}
}
//��̨Yaw��Pitch�ĳ�ʼ��
//������������̨�ķ�ʽ
void Cloud_Init(void)
{
	//Yaw/Pitch���˲�ϵ����ʼ��
	Cloud.LpfAttFactor = 1.0f;
	//ȷ����̨������ĵ��
	Cloud.Yaw = &GM6020s[0];
	Cloud.Pitch = &GM6020s[1];
	//ȷ����̨��IMU
	Cloud.IMU = &DJIC_IMU;
	//IMU��λ�ĳ�ʼ��
	Cloud_FUN.Cloud_IMUlimit_Calculate();
	//IMUɨ��ĳ�ʼ��
	Cloud_FUN.IMU_Scan_Init();
	//EMɨ��ĳ�ʼ��
	Cloud_FUN.EM_Scan_Init();
	//PID��ʼ��
	Cloud_FUN.Cloud_PID_Init();
	//���Cloud��ɶ����
	Cloud_FUN.Cloud_Emergency_Processing();
	//�ж���̨��ɶ���ƣ�IMU �� EM
	if (Cloud.real_way == IMU)
	{
		Cloud_FUN.Cloud_IMU_Init();
	}
	else if (Cloud.real_way == EM)
	{
		Cloud_FUN.Cloud_EM_Init();
	}
}
//Debug_PID
float Yaw_Debugpid = 0;
float Pitch_Debugpid = 0;
void Cloud_Control(void)
{
	//(���������һЩ���ݣ���־��λ���£����ǵ÷��ڿ��ƿ��������У������ӿ��Ա���������ݸ��������е�ʱ��Ϳ��������ʱ��ƥ�䣬���������Ƶ����ݾͲ����)
	//���ж�һ�´��ں��ֿ��Ʒ�ʽ
	Cloud_FUN.Cloud_Emergency_Processing();
	//�޷�����
	Cloud_FUN.Cloud_IMUlimit_Calculate();
	//ɨ���ȡ���ȸ���
	Cloud_FUN.Radian_Gain();

	if (Robots_Control.Cloud_e == cd_Disable)
	{
		//��ը
		Cloud.targetYawRaw = *Cloud.yaw_total;
		Cloud.targetPitchRaw = *Cloud.pitch_total;
		//���Ϊ0
		Cloud.yaw_output = 0;
		Cloud.pitch_output = 0;
		//���pid
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch��
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Speed_pid);

		return;
	}
	//ң�س���
	else if (Robots_Control.Cloud_e == R_cd_Common)
	{
		//������̨���ȵ���
		//��̨Ҫ���ǽǶȣ�������Ҫ���ٶ�
		//��ң��ʱ����̨��Ҫ���ֵ�ǰ�ĽǶȵģ���������Ҫ�ٶ�Ϊ0��
		//������̨��Ҫ += ������ֻ�� =
		if (Cloud.real_way == EM)
		{
			//Yaw��
			Cloud.targetYawRaw += DR16.ch0 * 0.05f;
			//Pitch��
			Cloud.targetPitchRaw += DR16.ch1 * 0.01f; //����DR16������660����Pitchֻ��660-1880֮���ƶ�������ϵ����Сһ��
		}
		else if (Cloud.last_way == IMU)
		{
			Cloud.targetYawRaw += DR16.ch0 * 0.5f * 1.0f / Coded_Contact_Radian;
			Cloud.targetPitchRaw += DR16.ch1 * 0.3f * 1.0f / Coded_Contact_Radian;
		}

#if Cloud_Debug_PID == 1
		Cloud.targetYawRaw = Yaw_Debugpid;
		Cloud.targetPitchRaw = Pitch_Debugpid;
#endif
	}
	//ɨ��
	else if (Robots_Control.Cloud_e == A_cd_Scan)
	{
		Cloud_FUN.Scan_Processing();
	}
	//����
	else if (Robots_Control.Cloud_e == cd_AutoAim)
	{
		Vision_Control_Cloud();
	}

	if (Robots_Control.Cloud_e != cd_AutoAim)
	{
		//��ʶ��װ�װ�����������˲��Ĺ�����(ʵ���Ͼ����൱�����³�ʼ��һ�¿���������)�����о����˲�ʱ��,
		//�Լ�������б�µĵ�ǰֵ...
		Clear_VisionData_Processing();
	}

	//ֻ��Pitch�����Ҫ�޷������޷��ļ���ֵ��totalֵ��������targetֵ
	if (Cloud.targetPitchRaw < Cloud.limit->low)
	{
		Cloud.targetPitchRaw = Cloud.limit->low;
	}
	else if (Cloud.targetPitchRaw > Cloud.limit->high)
	{
		Cloud.targetPitchRaw = Cloud.limit->high;
	}

//	if (Cloud.targetYawRaw >= 310.0f)
//	{
//		Cloud.targetYawRaw = 310.0f;
//	}
//	else if (Cloud.targetYawRaw <= 180.0f)
//	{
//		Cloud.targetYawRaw = 180.0f;
//	}

	//�˲�,�õ�Ŀ��
	//Yaw��
	Filter_IIRLPF(&Cloud.targetYawRaw, &Cloud.targetYawLPF, Cloud.LpfAttFactor);
	//Picth��
	Filter_IIRLPF(&Cloud.targetPitchRaw, &Cloud.targetPitchLPF, Cloud.LpfAttFactor);

	if (Robots_Control.Cloud_e == cd_AutoAim)
	{
		//���������pid
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch��
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Speed_pid);

		Cloud.targetYawRaw = *Cloud.yaw_total;
		Cloud.targetPitchRaw = *Cloud.pitch_total;

		//yaw
		P_PID_FUN.P_PID_Regulation(Vision_Ctrl.Yaw_A_pid, Vision_Ctrl.Yaw_FinalAngle, *Cloud.yaw_total);
		P_PID_FUN.P_PID_Regulation(Vision_Ctrl.Yaw_S_pid, Vision_Ctrl.Yaw_A_pid->result, *Cloud.gyro_z);
		Cloud.yaw_output = Vision_Ctrl.Yaw_S_pid->result;
		//Pitch
		P_PID_FUN.P_PID_Regulation(Vision_Ctrl.Pitch_A_pid, Vision_Ctrl.Pitch_FinalAngle, *Cloud.pitch_total);
		P_PID_FUN.P_PID_Regulation(Vision_Ctrl.Pitch_S_pid, Vision_Ctrl.Pitch_A_pid->result, *Cloud.gyro_y);
		Cloud.pitch_output = Vision_Ctrl.Pitch_S_pid->result;
	}
	else
	{
		//��������pid
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_S_pid);
		//Pitch��
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_S_pid);

		//yaw��
		P_PID_FUN.P_PID_Regulation(Cloud.Yaw_Angle_pid, Cloud.targetYawLPF, *Cloud.yaw_total);
		Cloud.yaw_output = P_PID_FUN.P_PID_Regulation(Cloud.Yaw_Speed_pid, Cloud.Yaw_Angle_pid->result, *Cloud.gyro_z);
		//pitch
		P_PID_FUN.P_PID_Regulation(Cloud.Pitch_Angle_pid, Cloud.targetPitchLPF, *Cloud.pitch_total);
		Cloud.pitch_output = P_PID_FUN.P_PID_Regulation(Cloud.Pitch_Speed_pid, Cloud.Pitch_Angle_pid->result, *Cloud.gyro_y);
	}

	/*
	//����һ��ǰ����������������Ӱ��
	if(GM6020s[1].targetAngle > 3790)
	{
		//matlab��ϵ���������
		float a1 = 1884,b1 = 0.01347,c1 = 68.6;
		float y = a1 * sin(b1 * GM6020s[1].totalAngle + c1);
		GM6020s[1].PSoutVoltage = GM6020s[1].P_SpeedPID.result + y;
	}
	else if(GM6020s[1].targetAngle < 3660)
	{
		float a1_1 = 1974, b1_1 = 0.006564,c1_1 = 13.45;
		float y_1 =  a1_1 * sin(b1_1 * GM6020s[1].totalAngle + c1_1);
		GM6020s[1].PSoutVoltage = GM6020s[1].P_SpeedPID.result + y_1;
	}
	*/

	//	���õ�ѹֵ������������Ŀ��ƺ���д
	//	GM6020_SetVoltage(GM6020s[0].PSoutVoltage,GM6020s[1].PSoutVoltage,0,0);
}
