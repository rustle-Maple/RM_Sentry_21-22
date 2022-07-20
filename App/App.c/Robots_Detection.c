#include "Robots_Detection.h"

static uint32_t FPS_Calculate(uint32_t deltaTime)
{
	return (1.0f / (double)(deltaTime)) * 1000.0f; // 别忘了先转换为浮点数，否则会有精度丢失
}
//得到设备的数据传输帧率
void Get_FPS(WorldTime_RxTypedef *time, uint32_t *FPS)
{
	time->WorldTime = xTaskGetTickCount() * portTICK_PERIOD_MS;	  //获取当前系统的时钟节拍
	*FPS = FPS_Calculate(time->WorldTime - time->Last_WorldTime); //计算得到两次的时钟节拍间距，并转单位精度，即得到FPS
	time->Last_WorldTime = time->WorldTime;
}
//离线帧率清0
void Offline_Clear_FPS(void)
{
	/* Euler */
	//离线检测是否大于0
	if (Euler_Framerate.Offline_Detec > 0)
	{
		//大于0，则并非离线
		Euler_Framerate.Offline_Flag = 0;
	}
	else
	{
		//否则离线标志位置1
		Euler_Framerate.Offline_Flag = 1;
	}
	//一旦离线了就将上一刻的帧率清0
	if (Euler_Framerate.Offline_Flag == 1)
	{
		Euler_Framerate.FPS = 0;
	}
	//清除离线检测变量，用于下次检测
	Euler_Framerate.Offline_Detec = 0;

	/* Gyro */
	if (Gyro_Framerate.Offline_Detec > 0)
	{
		Gyro_Framerate.Offline_Flag = 0;
	}
	else
	{
		Gyro_Framerate.Offline_Flag = 1;
	}
	if (Gyro_Framerate.Offline_Flag == 1)
	{
		Gyro_Framerate.FPS = 0;
	}
	Gyro_Framerate.Offline_Detec = 0;

	/* RM_Judge */
	if (Judge_Monitor.Offline_Detec > 0)
	{
		Judge_Monitor.Offline_Flag = 0;
	}
	else
	{
		Judge_Monitor.Offline_Flag = 1;
	}
	if (Judge_Monitor.Offline_Flag == 1)
	{
		Judge_Monitor.FPS = 0;
	}
	Judge_Monitor.Offline_Detec = 0;

	/*摩擦轮离线检测*/
	if (M3508s1[0].OffLine_Detection > 0)
	{
		M3508s1[0].OffLine_Status = 0;
	}
	else
	{
		M3508s1[0].OffLine_Status = 1;
	}
	M3508s1[0].OffLine_Detection = 0;

	/*视觉*/
	if (VisionData.Offline_Detec > 0)
	{
		VisionData.Offline_Flag = 0;
	}
	else
	{
		VisionData.Offline_Flag = 1;
	}
	if (VisionData.Offline_Flag == 1)
	{
		VisionData.FPS = 0;
	}
	VisionData.Offline_Detec = 0;
	
	/*Sensor*/
	if (Sensor_L_Framerate.Offline_Detec > 0)
	{
		Sensor_L_Framerate.Offline_Flag = 0;
	}
	else
	{
		Sensor_L_Framerate.Offline_Flag = 1;
	}
	if (Sensor_L_Framerate.Offline_Flag == 1)
	{
		Sensor_L_Framerate.FPS = 0;
	}
	Sensor_L_Framerate.Offline_Detec = 0;
	
	if (Sensor_R_Framerate.Offline_Detec > 0)
	{
		Sensor_R_Framerate.Offline_Flag = 0;
	}
	else
	{
		Sensor_R_Framerate.Offline_Flag = 1;
	}
	if (Sensor_R_Framerate.Offline_Flag == 1)
	{
		Sensor_R_Framerate.FPS = 0;
	}
	Sensor_R_Framerate.Offline_Detec = 0;
	
	//编码器
	if(Chassis.encoder ->Update_times > 4000)
	{
		if(Chassis.encoder->FPS_lastline == Chassis.encoder->totalLine)
		{
			Chassis.encoder->Offline_flag = 1;
		}
		else
		{
			Chassis.encoder->Offline_flag = 0;
		}
		Chassis.encoder->FPS_lastline = Chassis.encoder->totalLine;
		Chassis.encoder->Update_times = 0;
		
	}
	
	
}
//对无帧率帧设备进行应急处理
void IMUFPS_0_Emergency_status(void)
{
	//由于总是断数据，决定看你是检测得有点严格了，试一下这样子吧
	if (Euler_Framerate.FPS != 0 || /* && */ Gyro_Framerate.FPS != 0)
	{
		Cloud.real_way = IMU;
	}
	else if (Euler_Framerate.FPS == 0 && /* || */ Gyro_Framerate.FPS == 0)
	{
		Cloud.real_way = EM;
	}
	if (Cloud.last_way != Cloud.real_way)
	{
		Cloud.way_change_flag = 1;
	}
	Cloud.last_way = Cloud.real_way;
	
	/* 对于视觉 突然无帧率 掉线的处理 */
	if(VisionData.Offline_Flag == 1)
	{
		//清除视觉残留的数据
		VisionData.RawData.Armour = 0;
		VisionData.RawData.Depth = 0;
		VisionData.RawData.Pitch_Angle = 0;
		VisionData.RawData.Yaw_Angle = 0;
	}
}
