#include "Vision_Control.h"

//视觉数据处理的结构体
VisionData_Hand_t VisionData_Hand;
//视觉控制结构体
Vision_Ctrl_t Vision_Ctrl;

//imu / em 补偿数值
Compensate_Data_t IMU_Compensate_Data;
Compensate_Data_t EM_Compensate_Data;

//Yaw轴二阶卡尔曼数组结构体
SecondOrder_Kalman_Init_t SK_Init_VisionData_Yaw = {
	//观测
	.H = {1.0f, 0.0f, 0.0f, 1.0f}, //状态观测转换矩阵,只要状态量和观测量的关系 观测量 = k*状态量 (k=1)
	//预测
	.A = {1.0f, delta_t, 0.0f, 1.0f}, //状态转换矩阵。通过运动模型得出。
	.B = {(float)delta_t * delta_t / 2.0f, delta_t},
	.Q = {1.0f, 0.0f, 0.0f, 1.0f}, //过程噪声协方差。此矩阵中数组的两个元素 是 调超参时确定的。
	//更新
	.R = {150.0f, 0.0f, 0.0f, 50.0f},		//观测噪声协方差。此矩阵中数组的两个元素 是 调超参时确定的。
	.LP_Optimal = {1.0f, 0.0f, 0.0f, 1.0f}, //此刻的最优估计值协方差。一般初始化时为1(即I:单位矩阵)
	.I = {1.0f, 0.0f, 0.0f, 1.0f}			//单位矩阵
};
//Yaw轴二阶卡尔曼矩阵结构体
SecondOrder_Kalman_t SK_VisionData_Yaw;

//Pitch轴
SecondOrder_Kalman_Init_t SK_Init_VisionData_Pitch = {
	//观测
	.H = {1.0f, 0.0f, 0.0f, 1.0f}, //状态观测转换矩阵,只要状态量和观测量的关系 观测量 = k*状态量 (k=1)
	//预测
	.A = {1.0f, delta_t, 0.0f, 1.0f}, //状态转换矩阵。通过运动模型得出。
	.B = {(float)delta_t * delta_t / 2.0f, delta_t},
	.Q = {1.0f, 0.0f, 0.0f, 1.0f}, //过程噪声协方差。此矩阵中数组的两个元素 是 调超参时确定的。
	//更新
	.R = {150.0f, 0.0f, 0.0f, 50.0f},		//观测噪声协方差。此矩阵中数组的两个元素 是 调超参时确定的。
	.LP_Optimal = {1.0f, 0.0f, 0.0f, 1.0f}, //此刻的最优估计值协方差。一般初始化时为1(即I:单位矩阵)
	.I = {1.0f, 0.0f, 0.0f, 1.0f}			//单位矩阵
};
SecondOrder_Kalman_t SK_VisionData_Pitch;

//深度一阶卡尔曼
FirstOrder_Kalman_t FK_VisionData_Depth;

//计算角速度
float Calculate_Omiga(Omiga_data_t *Omiga_data, float Position, uint32_t Times)
{
	//累计计算的时间，若时间超过了上位机传递给下位机的时间，则是传输失败，速度就为0
	Omiga_data->delay_cnt++;

	if (Times != Omiga_data->Time_Last)
	{
		//角速度
		Omiga_data->Omiga = (float)(Position - Omiga_data->Position_Last) / (Times - Omiga_data->Time_Last) * 2.0f; //目前还不知道为什么要*2.0f，感觉和瞬时速度有关
		//保存上刻的位置
		Omiga_data->Position_Last = Position;
		//保存上刻的时间
		Omiga_data->Time_Last = Times;
		//清除累计计算时间
		Omiga_data->delay_cnt = 0;
	}

	if (Omiga_data->delay_cnt > 200) //其实我觉得应该得看控制周期去改此值 若控制周期1ms，那么300->300ms，就没有什么问题
	{
		Omiga_data->Omiga = 0;
	}
	return Omiga_data->Omiga;
}
//计算角加速度
float Calculate_A_Omiga(A_Omiga_data_t *A_Omiga_data, float Omiga, uint32_t Times)
{
	//累计计算的时间，若时间超过了上位机传递给下位机的时间，则是传输失败，速度就为0
	A_Omiga_data->delay_cnt++;

	if (Times != A_Omiga_data->Time_Last)
	{
		//瞬时速度
		A_Omiga_data->A_Omiga = (float)(Omiga - A_Omiga_data->Omiga_Last) / (Times - A_Omiga_data->Time_Last);
		//保存上刻的位置
		A_Omiga_data->Omiga_Last = Omiga;
		//保存上刻的时间
		A_Omiga_data->Time_Last = Times;
		//清除累计计算时间
		A_Omiga_data->delay_cnt = 0;
	}

	if (A_Omiga_data->delay_cnt > 200) //其实我觉得应该得看控制周期去改此值 若控制周期1ms，那么300->300ms，就没有什么问题
	{
		A_Omiga_data->A_Omiga = 0;
	}
	return A_Omiga_data->A_Omiga;
}

//视觉无滤波数据的更新
void Updata_VisionData_Hand(void)
{

	//若视觉数据更新成功了
	if (VisionData.DataUpdate_Flag == 1)
	{

		//更新数据时的时钟节拍
		VisionData_Hand.Vision_RawData.WorldTimes = xTaskGetTickCount();

		//数据更新完成，数据更新标志位清0
		VisionData.DataUpdate_Flag = 0;

		//视觉深度：单位mm
		VisionData_Hand.Vision_RawData.Depth = VisionData.RawData.Depth;

		if (Cloud.real_way == IMU)
		{
			//Yaw轴：单位 角度° 转成 机械角度(码盘值)
			VisionData_Hand.Vision_RawData.Yaw_Angle = (float)VisionData.RawData.Yaw_Angle / 100.0f;
			//Pitch轴：
			VisionData_Hand.Vision_RawData.Pitch_Angle = (float)VisionData.RawData.Pitch_Angle / 100.0f;

			if (Cloud.last_way == EM)
			{
				VisionData_Hand.Vision_RawData.Yaw_Last_Angle *= (float)1.0f / Coded_Contact_Angle;
				VisionData_Hand.Vision_RawData.Pitch_Last_Angle *= (float)1.0f / Coded_Contact_Angle;
			}
		}
		else if (Cloud.real_way == EM)
		{
			//Yaw轴：单位 角度° 转成 机械角度(码盘值)
			VisionData_Hand.Vision_RawData.Yaw_Angle = (float)Coded_Contact_Angle * VisionData.RawData.Yaw_Angle / 100.0f;
			//Pitch轴：
			VisionData_Hand.Vision_RawData.Pitch_Angle = (float)Coded_Contact_Angle * VisionData.RawData.Pitch_Angle / 100.0f;

			if (Cloud.last_way == IMU)
			{
				VisionData_Hand.Vision_RawData.Yaw_Last_Angle *= (float)Coded_Contact_Angle;
				VisionData_Hand.Vision_RawData.Pitch_Last_Angle *= (float)Coded_Contact_Angle;
			}
		}
		//数据更新成功，但是却无识别到装甲板,保存上一刻的数据
		//即使接收到错误数据还是保存上一刻的数据
		//保证切换时更加丝滑
		if (VisionData.RawData.Depth == 0 || VisionData.RawData.Armour == 0 || VisionData.Offline_Flag == 1)
		{
			//角度保存为上一刻的值
			VisionData_Hand.Vision_RawData.Depth = 0.0f;
			VisionData_Hand.Vision_RawData.Yaw_Angle = 0.0f;
			VisionData_Hand.Vision_RawData.Pitch_Angle = 0.0f;

			//角度变化量清0
			VisionData_Hand.Vision_RawData.Yaw_Delta_Angle = 0.0f;
			VisionData_Hand.Vision_RawData.Pitch_Delta_Angle = 0.0f;

			return;
		}
		//计算一下角度变化量
		VisionData_Hand.Vision_RawData.Yaw_Delta_Angle = VisionData_Hand.Vision_RawData.Yaw_Angle - VisionData_Hand.Vision_RawData.Yaw_Last_Angle;
		VisionData_Hand.Vision_RawData.Pitch_Delta_Angle = VisionData_Hand.Vision_RawData.Pitch_Angle - VisionData_Hand.Vision_RawData.Pitch_Last_Angle;

		//保存上一时刻的值
		VisionData_Hand.Vision_RawData.Last_Depth = VisionData_Hand.Vision_RawData.Depth;
		VisionData_Hand.Vision_RawData.Yaw_Last_Angle = VisionData_Hand.Vision_RawData.Yaw_Angle;
		VisionData_Hand.Vision_RawData.Pitch_Last_Angle = VisionData_Hand.Vision_RawData.Pitch_Angle;
	}
}
//视觉数据的滤波处理计算
void VisionData_Hand_Smooth_Processing(void)
{
	/* 原本 IMU 和 EM 需要有各自的角速度和角加速度的结构体变量，
		不然切换的时候会有上时刻的 位置 和 角度 是原本 IMU 或 EM 的数据 
		会导致数据错乱。
		但是由于卡尔曼数据处理后，做速度补偿是延时开启的，所以经过较长时间的迭代计算之后，数据就正确了
		所以就不重新搞两个结构体了，减少工作量
		但切换时要将卡尔曼计算时间清0 
	*/
	//清0卡尔曼计算时间
	if (Cloud.last_way != Cloud.real_way)
	{
		VisionData_Hand.Vision_FilterData.Kalman_Delay = 0;
	}

	//陀螺仪的值 + 视觉的偏差值 = 目标值
	//不加陀螺仪数据，那么你的外环目标值就得为0，而你整个外环的控制是依靠摄像头的精度，
	//而不是用陀螺仪的的角度去算的
	VisionData_Hand.Vision_FilterData.Raw_yaw = *Cloud.yaw_total - VisionData_Hand.Vision_RawData.Yaw_Angle;
	VisionData_Hand.Vision_FilterData.Raw_pitch = *Cloud.pitch_total - VisionData_Hand.Vision_RawData.Pitch_Angle;

	//更新角速度
	VisionData_Hand.Vision_RawData.Yaw_Omiga = Calculate_Omiga(&VisionData_Hand.Vision_RawData.Yaw_Omiga_data, VisionData_Hand.Vision_FilterData.Raw_yaw, VisionData_Hand.Vision_RawData.WorldTimes);
	VisionData_Hand.Vision_RawData.Pitch_Omiga = Calculate_Omiga(&VisionData_Hand.Vision_RawData.Pitch_Omiga_data, VisionData_Hand.Vision_FilterData.Raw_pitch, VisionData_Hand.Vision_RawData.WorldTimes);
	//更新角加速度
	VisionData_Hand.Vision_RawData.Yaw_A_Omiga = Calculate_A_Omiga(&VisionData_Hand.Vision_RawData.Yaw_A_Omiga_data, VisionData_Hand.Vision_RawData.Yaw_Omiga, VisionData_Hand.Vision_RawData.WorldTimes);
	VisionData_Hand.Vision_RawData.Pitch_A_Omiga = Calculate_A_Omiga(&VisionData_Hand.Vision_RawData.Pitch_A_Omiga_data, VisionData_Hand.Vision_RawData.Pitch_Omiga, VisionData_Hand.Vision_RawData.WorldTimes);

	//卡尔曼滤波的初始化再视觉初始化中完成
	//depth数据进行一阶卡尔曼滤波
	VisionData_Hand.Vision_FilterData.Depth = FirstOrder_Kalman_Calucate(&FK_VisionData_Depth, VisionData_Hand.Vision_RawData.Depth);

	//二阶卡尔曼
	//Yaw轴
	VisionData_Hand.Vision_FilterData.Yaw_Datas = SencondOrder_Kalman_Calucate(&SK_VisionData_Yaw, VisionData_Hand.Vision_FilterData.Raw_yaw, VisionData_Hand.Vision_RawData.Yaw_Omiga, VisionData_Hand.Vision_RawData.Yaw_A_Omiga);
	VisionData_Hand.Vision_FilterData.Yaw_Angle = VisionData_Hand.Vision_FilterData.Yaw_Datas[0];
	VisionData_Hand.Vision_FilterData.Yaw_Omiga = VisionData_Hand.Vision_FilterData.Yaw_Datas[1];

	//Pitch轴
	VisionData_Hand.Vision_FilterData.Pitch_Datas = SencondOrder_Kalman_Calucate(&SK_VisionData_Pitch, VisionData_Hand.Vision_FilterData.Raw_pitch, VisionData_Hand.Vision_RawData.Pitch_Omiga, VisionData_Hand.Vision_RawData.Pitch_A_Omiga);
	VisionData_Hand.Vision_FilterData.Pitch_Angle = VisionData_Hand.Vision_FilterData.Pitch_Datas[0];
	VisionData_Hand.Vision_FilterData.Pitch_Omiga = VisionData_Hand.Vision_FilterData.Pitch_Datas[1];

	//卡尔曼数据处理时间累计
	VisionData_Hand.Vision_FilterData.Kalman_Delay++;
}
//IMU补偿的初始化
void IMU_Compensate_Init(void)
{
	//补偿开启的速度最小/大值
	IMU_Compensate_Data.Yaw_Omiga_Min = 0;//0.002275;
	IMU_Compensate_Data.Yaw_Omiga_Max = 0;//2.0f;
	IMU_Compensate_Data.Pitch_Omiga_Min = 0.0f;
	IMU_Compensate_Data.Pitch_Omiga_Max = 0.0f;

	//角度变化量的极限值
	IMU_Compensate_Data.Yaw_DeltaAngle_Max = 0;//60.0f;
	IMU_Compensate_Data.Pitch_DeltaAngle_Max = 0.0f;

	//补偿系数 速度/角加速度
	IMU_Compensate_Data.Velocity_Factor = 0;//30000.0f;
	IMU_Compensate_Data.Gravity_Factor = 0.0f;
	IMU_Compensate_Data.A_Velocity_Factor = 0.0f;
	IMU_Compensate_Data.A_Gravity_Factor = 0.0f;

	//补偿输出限幅
	IMU_Compensate_Data.Velocity_Min = 0;//-5.0f;
	IMU_Compensate_Data.Velocity_Max = 0;//5.0f;
	IMU_Compensate_Data.Gravity_Min = 0.0f;
	IMU_Compensate_Data.Gravity_Max = 0.0f;

	//斜坡初始化
	IMU_Compensate_Data.Velocity_Ramp.Rate = 0;//0.05;
	IMU_Compensate_Data.Velocity_Ramp.Absolute_Max = //5.0f;
	IMU_Compensate_Data.Gravity_Ramp.Rate = 0.0f;
	IMU_Compensate_Data.Gravity_Ramp.Absolute_Max = 0.0f;

	//预测补偿开启的时间阈值
	IMU_Compensate_Data.Kalman_Delay_Open = 200.0f;
}
//EM补偿的初始化
void EM_Compensate_Init(void)
{
	//补偿开启的速度最小/大值
	EM_Compensate_Data.Yaw_Omiga_Min = 0.0f;
	EM_Compensate_Data.Yaw_Omiga_Max = 0.0f;
	EM_Compensate_Data.Pitch_Omiga_Min = 0.0f;
	EM_Compensate_Data.Pitch_Omiga_Max = 0.0f;

	//角度变化量的极限值
	EM_Compensate_Data.Yaw_DeltaAngle_Max = 0.0f;
	EM_Compensate_Data.Pitch_DeltaAngle_Max = 0.0f;

	//补偿系数 速度/角加速度
	EM_Compensate_Data.Velocity_Factor = 0.0f;
	EM_Compensate_Data.Gravity_Factor = 0.0f;
	EM_Compensate_Data.A_Velocity_Factor = 0.0f;
	EM_Compensate_Data.A_Gravity_Factor = 0.0f;

	//补偿输出限幅
	EM_Compensate_Data.Velocity_Min = 0.0f;
	EM_Compensate_Data.Velocity_Max = 0.0f;
	EM_Compensate_Data.Gravity_Min = 0.0f;
	EM_Compensate_Data.Gravity_Max = 0.0f;

	//斜坡初始化
	EM_Compensate_Data.Velocity_Ramp.Rate = 0.0;
	EM_Compensate_Data.Velocity_Ramp.Absolute_Max = 0.0f;
	EM_Compensate_Data.Gravity_Ramp.Rate = 0.0f;
	EM_Compensate_Data.Gravity_Ramp.Absolute_Max = 0.0f;

	//预测补偿开启的时间阈值
	EM_Compensate_Data.Kalman_Delay_Open = 200.0f;
}
//视觉pid的初始化
void Vision_PID_Init(void)
{
	//PID初始化
	//IMU
	//Yaw轴
	P_PID_Parameter_Init(&Vision_Ctrl.IMU_pid_t.Yaw_A_pid, -8.0f/*-12.0f*/, 0.0f, -40.5f, 0.0f, 0.0f, 0.0f, 0.75f, 0.0f, -0.0f, 600.0f, -600.0f);
	P_PID_Parameter_Init(&Vision_Ctrl.IMU_pid_t.Yaw_S_pid, 750/*800.0f*/, 7.5f, 1200.0f, 80.0f, 0.0f, 0.0f, 0.0f, 5000.0f, -5000.0f, 28888.0f, -28888.0f);
	//Pitch轴
	P_PID_Parameter_Init(&Vision_Ctrl.IMU_pid_t.Pitch_A_pid, 21.0f/*25.0f*/, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 300.0f, -300.0f);
	P_PID_Parameter_Init(&Vision_Ctrl.IMU_pid_t.Pitch_S_pid, 300.0f/*340.0f*/, 2.6/*3.2f*/, 0.0f, 300.0f, 0.0f, 0.0f, 0.0f, 3000.0f, -3000.0f, 28888.0f, -28888.0f);
	//EM
	P_PID_Parameter_Init(&Vision_Ctrl.EM_pid_t.Yaw_A_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 150.0f, -150.0f);
	P_PID_Parameter_Init(&Vision_Ctrl.EM_pid_t.Yaw_S_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, -10000.0f, 28888.0f, -28888.0f);
	P_PID_Parameter_Init(&Vision_Ctrl.EM_pid_t.Pitch_A_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -0.0f, 150.0f, -150.0f);
	P_PID_Parameter_Init(&Vision_Ctrl.EM_pid_t.Pitch_S_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f, -10000.0f, 28888.0f, -28888.0f);
}
void Vision_Control_Init(void)
{
	//一、二介卡尔曼的初始化
	//深度
	FirstOrder_Kalman_Parameter_Init(&FK_VisionData_Depth, 400, 1);
	//Yaw轴
	SencondOrder_Kalman_Parameter_Init(&SK_Init_VisionData_Yaw, &SK_VisionData_Yaw);
	//Pitch轴
	SencondOrder_Kalman_Parameter_Init(&SK_Init_VisionData_Pitch, &SK_VisionData_Pitch);

	//IMU补偿初始化
	IMU_Compensate_Init();
	//EM补偿初始化
	EM_Compensate_Init();

	//PID初始化
	Vision_PID_Init();
	
	if (Cloud.real_way == IMU)
	{
		//补偿指针指向IMU补偿
		VisionData_Hand.Compensate_Data = &IMU_Compensate_Data;
		//PID指针指向IMUpid
		Vision_Ctrl.Yaw_A_pid = &Vision_Ctrl.IMU_pid_t.Yaw_A_pid;
		Vision_Ctrl.Yaw_S_pid = &Vision_Ctrl.IMU_pid_t.Yaw_S_pid;
		Vision_Ctrl.Pitch_A_pid = &Vision_Ctrl.IMU_pid_t.Pitch_A_pid;
		Vision_Ctrl.Pitch_S_pid = &Vision_Ctrl.IMU_pid_t.Pitch_S_pid;
	}
	else if (Cloud.real_way == EM)
	{
		VisionData_Hand.Compensate_Data = &EM_Compensate_Data;
		//PID指针指向IMUpid
		Vision_Ctrl.Yaw_A_pid = &Vision_Ctrl.EM_pid_t.Yaw_A_pid;
		Vision_Ctrl.Yaw_S_pid = &Vision_Ctrl.EM_pid_t.Yaw_S_pid;
		Vision_Ctrl.Pitch_A_pid = &Vision_Ctrl.EM_pid_t.Pitch_A_pid;
		Vision_Ctrl.Pitch_S_pid = &Vision_Ctrl.EM_pid_t.Pitch_S_pid;
	}
}
//视觉控制的应急处理
void Vision_Emergency_Processing(void)
{
	if (Cloud.way_change_flag == 1)
	{
		//清除卡尔曼过程量 和 补偿过程量
		Clear_VisionData_Processing();
		//清除pid
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Yaw_S_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_A_pid);
		P_PID_FUN.P_PID_Parameter_Clear(Vision_Ctrl.Pitch_S_pid);

		//重新指向补偿量
		if (Cloud.real_way == IMU)
		{
			//补偿指针指向IMU补偿
			VisionData_Hand.Compensate_Data = &IMU_Compensate_Data;
			//PID指针指向IMUpid
			Vision_Ctrl.Yaw_A_pid = &Vision_Ctrl.IMU_pid_t.Yaw_A_pid;
			Vision_Ctrl.Yaw_S_pid = &Vision_Ctrl.IMU_pid_t.Yaw_S_pid;
			Vision_Ctrl.Pitch_A_pid = &Vision_Ctrl.IMU_pid_t.Pitch_A_pid;
			Vision_Ctrl.Pitch_S_pid = &Vision_Ctrl.IMU_pid_t.Pitch_S_pid;
		}
		else if (Cloud.real_way == EM)
		{
			VisionData_Hand.Compensate_Data = &EM_Compensate_Data;
			//PID指针指向IMUpid
			Vision_Ctrl.Yaw_A_pid = &Vision_Ctrl.EM_pid_t.Yaw_A_pid;
			Vision_Ctrl.Yaw_S_pid = &Vision_Ctrl.EM_pid_t.Yaw_S_pid;
			Vision_Ctrl.Pitch_A_pid = &Vision_Ctrl.EM_pid_t.Pitch_A_pid;
			Vision_Ctrl.Pitch_S_pid = &Vision_Ctrl.EM_pid_t.Pitch_S_pid;
		}
	}
}
//清除滤波、补偿的所有数据
void Clear_VisionData_Processing(void)
{
	//由于视觉值一直赋为上刻的值
	//角速度为0，角加速度也为0
	//所以卡尔曼的过程值可可以不用清0
	//唯一有变化的，就是:LP_Optimal上一刻的最优估计值协方差
	//其他或是常数，或是重新传递进去的数据，或是进过迭代计算出来的值
	//深度：
	FK_VisionData_Depth.LP_Optimal = 1;
	//Yaw轴
	mat_init(&SK_VisionData_Yaw.LP_Optimal, 2, 2, (float *)SK_Init_VisionData_Yaw.LP_Optimal);
	//Pitch轴
	mat_init(&SK_VisionData_Pitch.LP_Optimal, 2, 2, (float *)SK_Init_VisionData_Pitch.LP_Optimal);

	//卡尔曼的计算时间为0
	VisionData_Hand.Vision_FilterData.Kalman_Delay = 0;

	//斜坡当前值是必清，不让下次切换回去时，就不是从无速度补偿到有慢慢叠加上去了，而会从上次的值开始，会有个阶跃变化
	//Yaw斜坡当前值清0
	VisionData_Hand.Compensate_Data->Velocity_Ramp.Current_Value = 0.0f;
	//Pitch
	VisionData_Hand.Compensate_Data->Gravity_Ramp.Current_Value = 0.0f;

		//初始化视觉的目标角度，防炸鸡
		Vision_Ctrl.Pitch_FinalAngle = *Cloud.pitch_total;
		Vision_Ctrl.Yaw_FinalAngle = *Cloud.yaw_total;
}
int a , b =0;
//视觉控制云台
void Vision_Control_Cloud(void)
{
	//预测补偿

	//要调试出一个角度，当角度大于某个阈值时，不开启预测，不让像小陀螺这种突变得很离谱的，就会特别抖
	//那是不是可以通过此方法来判断小陀螺，然后做出相应的处理
	//而怎么去确定小陀螺时要打击哪里是一个难点，还有一点就是边走边小陀螺很难应对
	//控制射频去对付小陀螺是一种想法
	Vision_Emergency_Processing();

	//只有当识别到装甲，才做滤波才做预测
	//无识别到装甲板的话，总去滤一些无用的值/错误的值，对滤波中的过程变量应该会影响较大
	//卡尔曼滤波
	VisionData_Hand_Smooth_Processing();
	//Yaw轴
	if (VisionData_Hand.Vision_FilterData.Kalman_Delay > VisionData_Hand.Compensate_Data->Kalman_Delay_Open					 //刚开始的卡尔曼处理的数据并未完全贴合，得经过几代的迭代才能贴合
		&& fabs(VisionData_Hand.Vision_FilterData.Yaw_Omiga * 22.75f) > VisionData_Hand.Compensate_Data->Yaw_Omiga_Min				 //角速度大于最小值
		&& fabs(VisionData_Hand.Vision_FilterData.Yaw_Omiga) < VisionData_Hand.Compensate_Data->Yaw_Omiga_Max				 //角速度小于最大值
		&& (float)fabs(VisionData_Hand.Vision_RawData.Yaw_Delta_Angle) < VisionData_Hand.Compensate_Data->Yaw_DeltaAngle_Max //角度变化量太大时就不进行预测，例如小陀螺这种变化量大且高频的
	)
	{
		//速度分解：
		//根据已知的深度和子弹深度方向的初速度（把它简化成简单的匀速运动模型）得到运到到目标点的时间，此时间为深度方向的时间，
		//但是根据速度分解可知道，无论时Yaw轴还是Pitch上所运行的时间都是与detph上所需的时间是相同的。
		VisionData_Hand.Compensate_Data->Bullet_Times = (float)VisionData_Hand.Vision_FilterData.Depth / 1000.0f / Bullet_Velocity;
		if(VisionData_Hand.Vision_FilterData.Yaw_Omiga >= 0)
		{
			//速度补偿：预测速度 * 子弹飞行速度 * Kp
			VisionData_Hand.Compensate_Data->Velocity_result = VisionData_Hand.Compensate_Data->Velocity_Factor * (VisionData_Hand.Vision_FilterData.Yaw_Omiga - (float)VisionData_Hand.Compensate_Data->Yaw_Omiga_Min / 22.75f) * VisionData_Hand.Compensate_Data->Bullet_Times;
		}
		else if(VisionData_Hand.Vision_FilterData.Yaw_Omiga < 0)
		{
			VisionData_Hand.Compensate_Data->Velocity_result = VisionData_Hand.Compensate_Data->Velocity_Factor * (VisionData_Hand.Vision_FilterData.Yaw_Omiga + (float)VisionData_Hand.Compensate_Data->Yaw_Omiga_Min / 22.75f) * VisionData_Hand.Compensate_Data->Bullet_Times;
		}
		//速度补偿限幅
		Value_Limit(&VisionData_Hand.Compensate_Data->Velocity_result, &VisionData_Hand.Compensate_Data->Velocity_Min, &VisionData_Hand.Compensate_Data->Velocity_Max);
		//确定斜坡的目标值
		VisionData_Hand.Compensate_Data->Velocity_Ramp.Target_Value = VisionData_Hand.Compensate_Data->Velocity_result;
		//斜坡计算值
		VisionData_Hand.Compensate_Data->Velocity_Ramp.Current_Value = Ramp_Function(&VisionData_Hand.Compensate_Data->Velocity_Ramp);
		Vision_Ctrl.Yaw_FinalAngle = (float)VisionData_Hand.Vision_FilterData.Yaw_Angle + VisionData_Hand.Compensate_Data->Velocity_Ramp.Current_Value;
		//加速度
		//Vision_Ctrl.Yaw_FinalAngle += VisionData_Hand.Compensate_Data.A_Velocity_Factor * VisionData_Hand.Vision_RawData.Yaw_A_Omiga;
		a++;
	}
	else //变化得太快或太慢，都不加补偿只是卡尔曼滤波
	{
		//斜坡当前值是必清，不让下次切换回去时，就不是从无速度补偿到有慢慢叠加上去了，而会从上次的值开始，会有个阶跃变化
		//Yaw斜坡当前值清0
		VisionData_Hand.Compensate_Data->Velocity_Ramp.Current_Value = 0.0f;
		Vision_Ctrl.Yaw_FinalAngle = (float)VisionData_Hand.Vision_FilterData.Yaw_Angle;
		b++;
	}

	//Pitch
	if (VisionData_Hand.Vision_FilterData.Kalman_Delay > VisionData_Hand.Compensate_Data->Kalman_Delay_Open							  //刚开始的卡尔曼处理的数据并未完全贴合，得经过几代的迭代才能贴合
		&& VisionData_Hand.Vision_FilterData.Pitch_Omiga > VisionData_Hand.Compensate_Data->Pitch_Omiga_Min							  //角速度大于最小值
		&& VisionData_Hand.Vision_FilterData.Pitch_Omiga < VisionData_Hand.Compensate_Data->Pitch_Omiga_Max							  //角速度小于最大值
		&& ((float)VisionData_Hand.Vision_RawData.Pitch_Delta_Angle / 100.0f) > VisionData_Hand.Compensate_Data->Pitch_DeltaAngle_Max //角度变化量太大时就不进行预测，例如小陀螺这种变化量大且高频的
	)
	{
		VisionData_Hand.Compensate_Data->Bullet_Times = (float)VisionData_Hand.Vision_FilterData.Depth / 1000.0f / Bullet_Velocity;
		//重力补偿：预测速度 * 子弹飞行速度 * Kp
		VisionData_Hand.Compensate_Data->Gravity_result = VisionData_Hand.Compensate_Data->Gravity_Factor * VisionData_Hand.Vision_FilterData.Pitch_Omiga * VisionData_Hand.Compensate_Data->Bullet_Times;
		//重力补偿限幅
		Value_Limit(&VisionData_Hand.Compensate_Data->Gravity_result, &VisionData_Hand.Compensate_Data->Gravity_Min, &VisionData_Hand.Compensate_Data->Gravity_Max);
		//斜坡的目标值
		VisionData_Hand.Compensate_Data->Gravity_Ramp.Target_Value = VisionData_Hand.Compensate_Data->Gravity_result;
		//斜坡计算值
		VisionData_Hand.Compensate_Data->Gravity_Ramp.Current_Value = Ramp_Function(&VisionData_Hand.Compensate_Data->Gravity_Ramp);
		//重力速度斜坡
		Vision_Ctrl.Pitch_FinalAngle = VisionData_Hand.Vision_FilterData.Pitch_Angle;
		//重力加速度
		//Vision_Ctrl.Pitch_FinalAngle += VisionData_Hand.Compensate_Data.A_Gravity_Factor * VisionData_Hand.Vision_RawData.Pitch_A_Omiga;
	}
	else
	{
		//斜坡当前值是必清，不让下次切换回去时，就不是从无速度补偿到有慢慢叠加上去了，而会从上次的值开始，会有个阶跃变化
		//Yaw斜坡当前值清0
		VisionData_Hand.Compensate_Data->Gravity_Ramp.Current_Value = 0.0f;
		Vision_Ctrl.Pitch_FinalAngle = VisionData_Hand.Vision_FilterData.Pitch_Angle;
	}
}
