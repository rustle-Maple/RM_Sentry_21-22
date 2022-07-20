#include "Vision.h"

//�Ӿ��Ľ������ݻ�����
uint8_t Vision_DataBuff[Vision_BuffSize];
//�Ӿ��Ľ������ݽṹ��
VisionData_t VisionData;
//�Ӿ��ķ������ݽṹ��
VisionSend_Cloud_t Vision_Cloud;

//�Ӿ��ķ������ݻ�����
//֡ͷ��S��
//����˫����0���� 1�� 2��
//ģʽ��0Ĭ�� 1���� 2����� 3�ڱ� 4����
//ID��1Ӣ�� 2���� 3���� 6���˻� 7�ڱ�
//֡β��E��
uint8_t Vision_SendBuff[5][16] = {'S', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '3', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E',
								  'S', '0', '4', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', 'E'};


//�Ӿ����պ���
void Vision_DataReceive(uint8_t *data)
{
	//����CRCУ��
	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//���Ӿ����͹�����13��8λ���ݱ���һ��
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}

	//��Yaw\Pitch\Depth�ĸߵͰ�λ�ϲ�
	VisionData.RawData.Yaw_Angle = (VisionData.RawData.VisionRawData[4] | (VisionData.RawData.VisionRawData[5] << 8));
	VisionData.RawData.Pitch_Angle = (VisionData.RawData.VisionRawData[7] | (VisionData.RawData.VisionRawData[8] << 8));
	VisionData.RawData.Depth = (VisionData.RawData.VisionRawData[9] | (VisionData.RawData.VisionRawData[10] << 8));

	//�ж�Yaw\Pitch�ķ���
	if (VisionData.RawData.Yaw_Dir == 0)
	{
		VisionData.RawData.Yaw_Angle *= -1.0f;
	}
	if (VisionData.RawData.Pitch_Dir == 0)
	{
		VisionData.RawData.Pitch_Angle *= -1.0f;
	}
	
	//���յ��������Ϣ�����൱���޽��յ���Ϣ�������Ӿ�����
	//������Ұ��Χ���ǽ��յ���ȷ����Ϣ
	//������������� �������һ�̵����ݣ���֤�л�ʱ�νӵ��ȶ�
	//CRC����ʧ�� �� ֡ͷ��֡β����
	if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E')
	{
		//Ҳ��Ϊ����
		VisionData.Offline_Flag = 1;
		return;
	}

	//���ݸ��³ɹ���־λ
	VisionData.DataUpdate_Flag = 1;
	//���³ɹ���˵��û�д�����Ϣ
	VisionData.Offline_Flag = 0;
	
	//��ȡ�Ӿ�֡��
	Get_FPS(&VisionData.Vision_WorldTimes, &VisionData.FPS);

	//�Ӿ����߼��λ
	VisionData.Offline_Detec++;
}


//���û����˵�ID �� ����
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

//���·��� ���Ӿ�������������
void Updata_SendVision_Stata(void)
{
	Vision_Cloud.VisionSend_t.YawAngle_Error = DJIC_IMU.total_yaw;
	Vision_Cloud.VisionSend_t.PitchAngle_Error = DJIC_IMU.total_pitch;
}

//���Ӿ���������
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

//���·��͸��Ӿ�������,������
void Update_Vision_SendData(void)
{
	
	Vision_ID_Type_Init();
	
	for (uint8_t i = 0; i < 5; i++)
	{
		//��̨Yaw��ĽǶ�ƫ�� float -> uint8_t
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//��̨Pitch��ĽǶ�ƫ��
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][11] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];
		//Z�ļ��ٶ�
		Vision_SendBuff[i][12] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][13] = Vision_Cloud.Gyro_z_low;
		//��֧ǹ�ܵ��ٶ�����
		Vision_SendBuff[i][14] = 28;

		
		//���ݹ�����ģʽ�������Ӿ�
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


//DMA���������յ����ݴ���
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
