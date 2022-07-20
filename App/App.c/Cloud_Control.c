#include "Cloud_Control.h"

Cloud_t Cloud;

//限幅结构体
const float EM_LIMIT_LOW = 6420.0f;
const float EM_LIMIT_High = 7110.0f;
Limit_t IMU_limit;
Limit_t EM_limit = {EM_LIMIT_LOW, EM_LIMIT_High, (float)(EM_LIMIT_LOW + EM_LIMIT_High) / 2.0f};
//扫描结构体
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

//挂钩
Cloud_FUN_t Cloud_FUN = CLOUD_HOOK_FUN;

//IMU限幅的计算
void Cloud_IMUlimit_Calculate(void)
{
	/* 前提 EM 与 IMU 同向，否则符号得改变 */
	//计算EM限幅的
	float delta_low, delta_high;
	delta_low = (float)fabs(Cloud.Pitch->totalAngle - EM_limit.low) / Coded_Contact_Angle;
	delta_high = (float)fabs(Cloud.Pitch->totalAngle - EM_limit.high) / Coded_Contact_Angle;
	//超出EM限幅了
	if (Cloud.Pitch->totalAngle < EM_limit.low)
	{
		delta_low = 0.0f;
	}
	if (Cloud.Pitch->totalAngle > EM_limit.high)
	{
		delta_high = 0.0f;
	}
	//得到IMU限幅
	IMU_limit.low = Cloud.IMU->total_pitch - delta_low;
	IMU_limit.high = Cloud.IMU->total_pitch + delta_high;
	IMU_limit.centre = (float)(IMU_limit.low + IMU_limit.high) / 2.0f;
	IMU_limit.left = 360.0f;
	IMU_limit.right = 0.0f;
}
//IMU扫描结构体初始化
void IMU_Scan_Init(void)
{
	IMU_scan.total = &DJIC_IMU.total_pitch;
	IMU_scan.low = &IMU_limit.low;
	IMU_scan.high = &IMU_limit.high;
	IMU_scan.centre = &IMU_limit.centre;
	IMU_scan.radian = 0.0f;
	IMU_scan.dir_pitch = 1;
	IMU_scan.unit_incre_picth = (float)1.0f / Angle_Contact_Radian; //（自变量）
	IMU_scan.unit_incre_yaw = 1.0f;
	IMU_scan.dir_yaw = 1;
	IMU_scan.count_yaw = &DJIC_IMU.yaw_turnCounts;
	IMU_scan.left = &IMU_limit.left;
	IMU_scan.right = &IMU_limit.right;
	IMU_scan.unit_value = Angle_Unit;
}
//EM扫描结构体初始化
void EM_Scan_Init(void)
{
	EM_scan.total = &Cloud.Pitch->totalAngle;
	EM_scan.low = &EM_limit.low;
	EM_scan.high = &EM_limit.high;
	EM_scan.centre = &EM_limit.centre;
	EM_scan.radian = 0.0f;
	EM_scan.dir_pitch = 1;
	EM_scan.unit_incre_picth = (float)1.0f / Angle_Contact_Radian; //（自变量）
	EM_scan.unit_incre_yaw = 1.0f;
	EM_scan.dir_yaw = 1;
	EM_scan.count_yaw = &Cloud.Yaw->turnCount;
	EM_scan.left = &EM_limit.left;
	EM_scan.right = &EM_limit.right;
	EM_scan.unit_value = EM_Unit;
}

//Pitch轴扫描，获取弧度值
//Yaw轴要通过这种sin函数的运动模型去运行，也不是不行
void Radian_Gain(void)
{
	/*
	只要不处于扫描模式下，都要进行对云台Pitch轴弧度的扫描，不然切换时会炸鸡
	无需一直扫描，只要退出扫描状态时才需要扫描当前的弧度，
	一旦进入扫描模式，是通过控制弧度的+/-，来达到控制电机的机械角度的，
	若在扫描模式下，还一直扫描当前弧度，会导致弧度的+/-的值 被扫描成当前的弧度的值 所有替换
	*/

	//当不处于扫描状态时，就要时刻获取弧度
	if (Robots_Control.Cloud_e != A_cd_Scan)
	{
		//设radian弧度对应的因变量变量为y
		double y;
		int32_t total;
		//由于扫描时，Pitch轴是在上下限位之间往返运动
		//所以构件一个运动模型：
		//以 totalAngel 为因变量，以对应的弧度 radian 为自变量，
		//构建一个正弦函数模型：totalAngle = Asin(bθ) + c

		//由于此运动模型的系数是更具上下极限推导出来的
		//所以为了防止电机转过，而脱离此运动系数
		//先给其限个幅
		if (*(Cloud.scan->total) < *(Cloud.scan->low)) //低于Pitch最低
		{
			total = *(Cloud.scan->low);
		}
		else if (*(Cloud.scan->total) > *(Cloud.scan->high)) //高于Pitch最高
		{
			total = *(Cloud.scan->high);
		}
		else
		{
			total = *(Cloud.scan->total);
		}
		//由于要得到当下totalAngle对应的弧度radian，所以要对正弦去取反函数arcsin
		//由于振幅 A = 上/下极限 - 中心 = Cloud_Pitch_High - Cloud_Pitch_Centre
		//b 为角频率，实际上也只是一个伸缩变换而已，即运动的快慢，为了计算方便可以视为1
		//C 为初相 当弧度radian为0是，对应的totalAngle，即为 c = Cloud_Pitch_Centre
		//推导过程，纸上解决
		y = (double)(total - *(Cloud.scan->centre)) / abs(*(Cloud.scan->high) - *(Cloud.scan->centre));
		//反正弦即可得弧度radian
		Cloud.scan->radian = asin(y);
	}
}
//扫瞄函数的执行
void Scan_Processing(void)
{
	float P_Kp = 0.4f, Y_Kp = 0.2f;

	//记得振幅要是正的，保持和扫描时的符号一致，不然会出现切换时突然间炸鸡
	Cloud.targetPitchRaw = (abs(*(Cloud.scan->high) - *(Cloud.scan->centre))) * sin(Cloud.scan->radian) + *(Cloud.scan->centre);
	//自变量每次增加1°，注意自变量的° 与 因变量的° 是不同的
	Cloud.scan->radian += Cloud.scan->dir_pitch * Cloud.scan->unit_incre_picth * P_Kp;
	//为了防止Cloud.Scan_Pitch_Radian变量的溢出，所以Cloud.Scan_Pitch_Dir的方向还是有必要改一下
	//虽然不该方向，自变量radian一直加，也可以实现因变量totalAngel 运动轨迹为 sin
	//其实不给个换向也没有什么问题，因为只要你不处于扫描模式，自变量radian就会被重置
	if (Cloud.scan->radian > Cloud.scan->unit_incre_picth * Angle_Unit * 5) //绕同个方向转10圈，则改变方向
	{
		Cloud.scan->dir_pitch = -1;
	}
	else if (Cloud.scan->radian < (-Cloud.scan->unit_incre_picth) * Angle_Unit * 5)
	{
		Cloud.scan->dir_pitch = 1;
	}
	//若Yaw轴无用sin运动模式，那就直接控制targetAngle按照一定的数值叠加
	Cloud.targetYawRaw += Cloud.scan->unit_incre_yaw * Cloud.scan->dir_yaw * Y_Kp;
	//对Yaw扫描时即可以防止一直往一个方向转，而数据溢出
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
	/* 注意坐标正方向，确定kp/ki/kd正负 注意别被抵抗电流骗了 */
	//IMU pid初始化
	//yaw : angle gyro
	//角度环与速度环坐标正方向相反
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.yaw_pid, -12.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	//速度环与电流环坐标正方向相同
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.gyro_z_pid, 800.0f, 10.5f, 1200.0f, 80.0f, 0.0f, 0.0f, 0.85f, 5000.0f, -5000.0, 28888.0f, -28888.0f);
	//pitch: angle gyro
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.pitch_pid, 25.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 300.0f, -300.0f);
	P_PID_FUN.P_PID_Parameter_Init(&DJIC_IMU.gyro_y_pid, 340.0f, 3.2f, 0.0f, 300.0f, 0.0f, 0.0f, 0.85f, 3000.0f, -3000.0, 28888.0f, -28888.0f);
	//EM
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Yaw->P_AnglePID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Yaw->P_SpeedPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 10000.0f, -10000.0, 28888.0f, -28888.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Pitch->P_AnglePID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 0.0f, -0.0, 600.0f, -600.0f);
	P_PID_FUN.P_PID_Parameter_Init(&Cloud.Pitch->P_SpeedPID, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.85f, 10000.0f, -10000.0, 28888.0f, -28888.0f);
	//视觉
	/* ... */
}
//IMU控制下的cloud的初始化
void Cloud_IMU_Init(void)
{
	//将云台的限位、扫描值 赋 为IMU的
	Cloud.limit = &IMU_limit;
	Cloud.scan = &IMU_scan;
	//PID的确定
	Cloud.Yaw_Angle_pid = &DJIC_IMU.yaw_pid;
	Cloud.Yaw_Speed_pid = &DJIC_IMU.gyro_z_pid;
	Cloud.Pitch_Angle_pid = &DJIC_IMU.pitch_pid;
	Cloud.Pitch_Speed_pid = &DJIC_IMU.gyro_y_pid;
	//初始化当前角度
	Cloud.yaw_total = &Cloud.IMU->total_yaw;
	Cloud.pitch_total = &Cloud.IMU->total_pitch;
	//初始化目标值防止炸鸡
	Cloud.targetYawRaw = *Cloud.yaw_total;
	Cloud.targetPitchRaw = *Cloud.pitch_total;
	//初始化当前速度值
	Cloud.gyro_z = &Cloud.IMU->Gyro_z;
	Cloud.gyro_y = &Cloud.IMU->Gyro_y;
}
//EM控制下CLoud的初始化
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
	Cloud.gyro_z = (float *)&Cloud.Yaw->freadSpeed; //注意强制类型转化
	Cloud.gyro_y = (float *)&Cloud.Pitch->freadSpeed;
}
//云台应急处理
void Cloud_Emergency_Processing(void)
{
	if (Cloud.way_change_flag == 1)
	{
		//先清除上时刻的PID控制
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch轴
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Speed_pid);
		//再初始化
		if (Cloud.real_way == IMU)
		{
			Cloud_FUN.Cloud_IMU_Init();
		}
		else if (Cloud.real_way == EM)
		{
			Cloud_FUN.Cloud_EM_Init();
		}
		//处理完之后再将变换标志位置0
		Cloud.way_change_flag = 0;
	}
}
//云台Yaw和Pitch的初始化
//参数：控制云台的方式
void Cloud_Init(void)
{
	//Yaw/Pitch轴滤波系数初始化
	Cloud.LpfAttFactor = 1.0f;
	//确定云台各个轴的电机
	Cloud.Yaw = &GM6020s[0];
	Cloud.Pitch = &GM6020s[1];
	//确定云台的IMU
	Cloud.IMU = &DJIC_IMU;
	//IMU限位的初始化
	Cloud_FUN.Cloud_IMUlimit_Calculate();
	//IMU扫描的初始化
	Cloud_FUN.IMU_Scan_Init();
	//EM扫描的初始化
	Cloud_FUN.EM_Scan_Init();
	//PID初始化
	Cloud_FUN.Cloud_PID_Init();
	//检测Cloud由啥控制
	Cloud_FUN.Cloud_Emergency_Processing();
	//判断云台受啥控制：IMU 或 EM
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
	//(控制任务的一些数据（标志）位更新，还是得放在控制控制任务中，这样子可以避免放在数据更新任务中的时序和控制任务的时序不匹配，而用来控制的数据就不相符)
	//先判断一下处于何种控制方式
	Cloud_FUN.Cloud_Emergency_Processing();
	//限幅更新
	Cloud_FUN.Cloud_IMUlimit_Calculate();
	//扫描获取弧度跟新
	Cloud_FUN.Radian_Gain();

	if (Robots_Control.Cloud_e == cd_Disable)
	{
		//防炸
		Cloud.targetYawRaw = *Cloud.yaw_total;
		Cloud.targetPitchRaw = *Cloud.pitch_total;
		//输出为0
		Cloud.yaw_output = 0;
		Cloud.pitch_output = 0;
		//清除pid
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch轴
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Pitch_Speed_pid);

		return;
	}
	//遥控常规
	else if (Robots_Control.Cloud_e == R_cd_Common)
	{
		//由于云台不比底盘
		//云台要的是角度，而底盘要的速度
		//无遥控时：云台是要保持当前的角度的，而底盘是要速度为0的
		//所以云台需要 += ，底盘只需 =
		if (Cloud.real_way == EM)
		{
			//Yaw轴
			Cloud.targetYawRaw += DR16.ch0 * 0.05f;
			//Pitch轴
			Cloud.targetPitchRaw += DR16.ch1 * 0.01f; //由于DR16推满就660，而Pitch只在660-1880之间移动，所以系数放小一点
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
	//扫描
	else if (Robots_Control.Cloud_e == A_cd_Scan)
	{
		Cloud_FUN.Scan_Processing();
	}
	//自瞄
	else if (Robots_Control.Cloud_e == cd_AutoAim)
	{
		Vision_Control_Cloud();
	}

	if (Robots_Control.Cloud_e != cd_AutoAim)
	{
		//无识别到装甲板则清除所有滤波的过程量(实际上就是相当于重新初始化一下卡尔曼参数)，还有就是滤波时间,
		//以及补偿的斜坡的当前值...
		Clear_VisionData_Processing();
	}

	//只有Pitch轴才需要限幅，而限幅的极限值是total值，而不是target值
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

	//滤波,得到目标
	//Yaw轴
	Filter_IIRLPF(&Cloud.targetYawRaw, &Cloud.targetYawLPF, Cloud.LpfAttFactor);
	//Picth轴
	Filter_IIRLPF(&Cloud.targetPitchRaw, &Cloud.targetPitchLPF, Cloud.LpfAttFactor);

	if (Robots_Control.Cloud_e == cd_AutoAim)
	{
		//清除非自瞄pid
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Angle_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Cloud.Yaw_Speed_pid);
		//Pitch轴
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
		//清除自瞄的pid
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_S_pid);
		//Pitch轴
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_S_pid);

		//yaw轴
		P_PID_FUN.P_PID_Regulation(Cloud.Yaw_Angle_pid, Cloud.targetYawLPF, *Cloud.yaw_total);
		Cloud.yaw_output = P_PID_FUN.P_PID_Regulation(Cloud.Yaw_Speed_pid, Cloud.Yaw_Angle_pid->result, *Cloud.gyro_z);
		//pitch
		P_PID_FUN.P_PID_Regulation(Cloud.Pitch_Angle_pid, Cloud.targetPitchLPF, *Cloud.pitch_total);
		Cloud.pitch_output = P_PID_FUN.P_PID_Regulation(Cloud.Pitch_Speed_pid, Cloud.Pitch_Angle_pid->result, *Cloud.gyro_y);
	}

	/*
	//给多一个前馈补偿，抵消重力影响
	if(GM6020s[1].targetAngle > 3790)
	{
		//matlab拟合的重力补偿
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

	//	设置电压值，在外面整体的控制函数写
	//	GM6020_SetVoltage(GM6020s[0].PSoutVoltage,GM6020s[1].PSoutVoltage,0,0);
}
