#include "Wolf_GyIMU.h"

bno055_data_u bno055_data[2];
WorldTime_RxTypedef IMU_Times;

GY_IMU_t IMU_Cloud;

//以下为CAN 通信部分。

/**
  * @brief  从CAN报文中获取云台陀螺仪信息
  * @param  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void GY6050_getCloundInfo(CAN_Rx_TypeDef RxMessage) {
	//报文id确认
	if (RxMessage.CAN_RxMessage.StdId == GY_IMU_READID)
	{
		memcpy(bno055_data[0].dataBuff, RxMessage.CAN_RxMessageData, sizeof(uint8_t[8]));
		IMU_Cloud.Eular.Yaw = (float)bno055_data[0].yaw / 100.0f;
		IMU_Cloud.Gyro.z = bno055_data[0].gyro_z / 16;
		IMU_Cloud.Eular.Pitch = (float)bno055_data[0].pitch / 100.0f;
		IMU_Cloud.Gyro.x = bno055_data[0].gyro_x / 16;

		if (abs(IMU_Cloud.Gyro.z) < 2)
		{
			IMU_Cloud.Gyro.z = bno055_data[0].gyro_z = 0;
		}

		if (IMU_Cloud.Eular.Yaw - IMU_Cloud.last_Eular.Yaw < -300) {//经过跳变边沿。
			IMU_Cloud.turnCount++;
		}
		if (IMU_Cloud.last_Eular.Yaw - IMU_Cloud.Eular.Yaw < -300) {
			IMU_Cloud.turnCount--;
		}

		IMU_Cloud.total_Eular.Yaw = IMU_Cloud.Eular.Yaw + (360 * IMU_Cloud.turnCount);

		IMU_Cloud.last_Eular.Yaw = IMU_Cloud.Eular.Yaw;
		
		Get_FPS(&IMU_Times, &IMU_Cloud.FPS);
		
	}
}


