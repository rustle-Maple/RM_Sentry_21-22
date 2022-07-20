#include "Vision.h"

//视觉的接收数据缓冲区
uint8_t Vision_DataBuff[Vision_BuffSize];
//视觉的接收数据结构体
VisionData_t VisionData;
//视觉的发送数据结构体
VisionSend_Cloud_t Vision_Cloud;

//视觉的发送数据缓冲区
//帧头‘S’
//敌我双方：0红蓝 1红 2蓝
//模式：0默认 1自瞄 2大神符 3哨兵 4基地
//ID：1英雄 2工程 3步兵 6无人机 7哨兵
//帧尾‘E’
uint8_t Vision_SendBuff[5][16] = {'S', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '3', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '4', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E'};


//视觉接收函数
void Vision_DataReceive(uint8_t *data)
{
	//进行CRC校验
	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//将视觉发送过来的13个8位数据遍历一遍
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}

	//将Yaw\Pitch\Depth的高低八位合并
	VisionData.RawData.Yaw_Angle = (VisionData.RawData.VisionRawData[4] | (VisionData.RawData.VisionRawData[5] << 8));
	VisionData.RawData.Pitch_Angle = (VisionData.RawData.VisionRawData[7] | (VisionData.RawData.VisionRawData[8] << 8));
	VisionData.RawData.Depth = (VisionData.RawData.VisionRawData[9] | (VisionData.RawData.VisionRawData[10] << 8));

	//判断Yaw\Pitch的符号
	if (VisionData.RawData.Yaw_Dir == 0)
	{
		VisionData.RawData.Yaw_Angle *= -1.0f;
	}
	if (VisionData.RawData.Pitch_Dir == 0)
	{
		VisionData.RawData.Pitch_Angle *= -1.0f;
	}
	
	//接收到错误的信息，则相当于无接收到消息，则无视觉作用
	//脱离视野范围，是接收到正确的消息
	//无论哪种情况都 保存最后一刻的数据，保证切换时衔接得稳定
	//CRC检验失败 或 帧头或帧尾错误
	if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E')
	{
		//也视为离线
		VisionData.Offline_Flag = 1;
		return;
	}

	//数据更新成功标志位
	VisionData.DataUpdate_Flag = 1;
	//更新成功就说明没有错误信息
	VisionData.Offline_Flag = 0;
	
	//获取视觉帧率
	Get_FPS(&VisionData.Vision_WorldTimes, &VisionData.FPS);

	//视觉离线检测位
	VisionData.Offline_Detec++;
}


//设置机器人的ID 和 类型
static void Vision_Set_ID_Type(uint8_t ID, uint8_t Type)
{
	for (uint8_t n = 0; n < 5; n++)
	{
		Vision_SendBuff[n][1] = ID;
		Vision_SendBuff[n][3] = Type;
	}
}
void Vision_ID_Type_Init(void)
{
	switch (ext_game_robot_state.data.robot_id)
	{
	case 1:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Hero;
		Vision_Set_ID_Type('1', '1');
		break;

	case 2:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Engineer;
		Vision_Set_ID_Type('1', '2');
		break;

	case 3:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('1', '3');
		break;

	case 4:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('1', '3');
		break;

	case 5:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('1', '3');
		break;

	case 6:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Aerial;
		Vision_Set_ID_Type('1', '6');
		break;

	case 7:
		Robots_Control.TeamColor = TeamColor_Red;
		Robots_Control.Types = Types_Sentry;
		Vision_Set_ID_Type('1', '7');
		break;

	case 101:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Hero;
		Vision_Set_ID_Type('2', '1');
		break;

	case 102:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Engineer;
		Vision_Set_ID_Type('2', '2');
		break;

	case 103:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('2', '3');
		break;

	case 104:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('2', '3');
		break;

	case 105:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Standard;
		Vision_Set_ID_Type('2', '3');
		break;

	case 106:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Aerial;
		Vision_Set_ID_Type('2', '6');
		break;

	case 107:
		Robots_Control.TeamColor = TeamColor_Blue;
		Robots_Control.Types = Types_Sentry;
		Vision_Set_ID_Type('2', '7');
		break;
	}
}

//更新发送 给视觉的陀螺仪数据
void Updata_SendVision_Stata(void)
{
	Vision_Cloud.VisionSend_t.YawAngle_Error = DJIC_IMU.total_yaw;
	Vision_Cloud.VisionSend_t.PitchAngle_Error = DJIC_IMU.total_pitch;
}

//向视觉发送数据
static void Vision_DataSend(uint8_t *data)
{
	if (data == NULL)
		return;

	for (uint8_t i = 0; i < 16; i++)
	{
		while ((UART8->SR & 0X40) == 0);
		UART8->DR = data[i];
	}
}

//更新发送给视觉的数据,并发送
void Update_Vision_SendData(void)
{
	
	Vision_ID_Type_Init();
	
	for (uint8_t i = 0; i < 5; i++)
	{
		//云台Yaw轴的角度偏差 float -> uint8_t
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//云台Pitch轴的角度偏差
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][11] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];
		//Z的加速度
		Vision_SendBuff[i][12] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][13] = Vision_Cloud.Gyro_z_low;
		//首支枪管的速度限制
		Vision_SendBuff[i][14] = 28;

		
		//根据攻击的模式，发给视觉
		switch (Robots_Control.AttackTarget)
		{
		case ShootTarget_default:
			Vision_DataSend(Vision_SendBuff[0]);
			break;
		case ShootTarget_Self_aiming:
			Vision_DataSend(Vision_SendBuff[1]);
			break;
		case ShootTarget_BIG_WHEEL:
			Vision_DataSend(Vision_SendBuff[2]);
			break;
		case ShootTarget_Sentry:
			Vision_DataSend(Vision_SendBuff[3]);
			break;
		case ShootTarget_base:
			Vision_DataSend(Vision_SendBuff[4]);
			break;
		}
		
	}
}


//DMA数据流接收的数据处理
void Vision_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == 2)
		{
			Vision_DataReceive(Vision_DataBuff);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, Vision_BuffSize);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}
