/**
  ******************************************************************************
  * @file    Debug_DataScope.c
  * @author  IMTao
  * @version V2.0
  * @brief   MiniBalance��λ��ģ��
  ******************************************************************************
  */
#include "Debug_DataScope.h"
unsigned char DataScope_OutPut_Buffer[42] = { 0 };	   //���ڷ��ͻ�����
/*
�÷���
		1. Debug_addData(500*sin(a), 1 ); ��Ҫ��ʾ�ĸ�ͨ�����ݣ��������ӣ�����1��ʾ��һ������,�������ơ�
			Debug_addData(500* tan(a), 2 );
			Debug_addData( 500*cos(a), 3 );

		2. Debug_show(3);  ��ɺ󣬵���show���������ݷ��͵���λ���У�3��ʾһ������3��ͨ�������ݡ�

*/

//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ 
//����˵�����û�����ֱ�Ӳ����˺��� 
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷��� 
void Float2Byte(float *target, unsigned char *buf, unsigned char beg)
{
	unsigned char *point;
	point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}



//ԭ��������DataScope_Get_Channel_Data
//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data��ͨ������
//Channel��ѡ��ͨ����1-10��
//�����޷��� 
void Debug_addData(float Data, unsigned char Channel)
{
	if ((Channel > 10) || (Channel == 0)) return;  //ͨ����������10�����0��ֱ����������ִ�к���
	else
	{
		switch (Channel)
		{
		case 1:  Float2Byte(&Data, DataScope_OutPut_Buffer, 1); break;
		case 2:  Float2Byte(&Data, DataScope_OutPut_Buffer, 5); break;
		case 3:  Float2Byte(&Data, DataScope_OutPut_Buffer, 9); break;
		case 4:  Float2Byte(&Data, DataScope_OutPut_Buffer, 13); break;
		case 5:  Float2Byte(&Data, DataScope_OutPut_Buffer, 17); break;
		case 6:  Float2Byte(&Data, DataScope_OutPut_Buffer, 21); break;
		case 7:  Float2Byte(&Data, DataScope_OutPut_Buffer, 25); break;
		case 8:  Float2Byte(&Data, DataScope_OutPut_Buffer, 29); break;
		case 9:  Float2Byte(&Data, DataScope_OutPut_Buffer, 33); break;
		case 10: Float2Byte(&Data, DataScope_OutPut_Buffer, 37); break;
		}
	}
}


//����˵�������� DataScopeV1.0 ����ȷʶ���֡��ʽ
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ((Channel_Number > 10) || (Channel_Number == 0)) { return 0; }  //ͨ����������10�����0��ֱ����������ִ�к���
	else
	{
		DataScope_OutPut_Buffer[0] = '$';  //֡ͷ

		switch (Channel_Number)
		{
		case 1:   DataScope_OutPut_Buffer[5] = 5; return  6;
		case 2:   DataScope_OutPut_Buffer[9] = 9; return 10;
		case 3:   DataScope_OutPut_Buffer[13] = 13; return 14;
		case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;
		case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		case 7:   DataScope_OutPut_Buffer[29] = 29; return 30;
		case 8:   DataScope_OutPut_Buffer[33] = 33; return 34;
		case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
		case 10:  DataScope_OutPut_Buffer[41] = 41; return 42;
		}
	}
	return 0;
}

//����˵�������Ѿ����ӵ����ͻ����������ݷ�����
//Channel_Number����Ҫ���͵�ͨ������
//���ط��ͻ��������ݸ���
//����0��ʾ֡��ʽ����ʧ�� 
void Debug_show(int ChannelAmount) {
	int Send_Count = DataScope_Data_Generate(ChannelAmount);

	/*ѭ��Send_Count�ν�DataScope_OutPut_Buffer[i]�е����ݷ��ͳ�ȥ��
		�������һ�ε�ǰ֡�������ϴ���*/
	for (int i = 0; i < Send_Count; i++)
	{
		while ((USART6->SR & 0X40) == 0);
		USART6->DR = DataScope_OutPut_Buffer[i];
		//USART_sendChar(USART6, DataScope_OutPut_Buffer[i]);
	}
}

/* ������һ������debug ʱ�л�����ĵ�����Ϣ�Ĺ��ܣ����Ķ��Ŀ����޸ĺ�ʹ�á�
typedef enum {
	DataType_TempData,
	DataType_CloudData,
	DataType_ChassisData,
	DataType_VisionData,
	DataType_ShootData

}DataType_e;
DataType_e Debug_DataType = DataType_ShootData;//������debug��ʱ��ֱ���޸ĸ�ֵʵ���л����� 
static uint8_t DelayTime = 10;
void Task_DebugShow(void) {
	while (true)
	{
		vTaskDelay(DelayTime);//����ʱ����ɷ��Ͷ���ʧ�ܡ�
		if (Debug_DataType == DataType_TempData)
		{

		}
		else if (Debug_DataType == DataType_CloudData)
		{
			Debug_addData(M6020s_Pitch.pid_Angle.err, 1);
			Debug_addData(M6020s_Pitch.outCurrent, 2);
			Debug_addData(M6020s_Pitch.realSpeed, 3);

			Debug_addData(Cloud.YawAttitude_pid.err, 4);
			Debug_addData(M6020s_Yaw.outCurrent, 5);
			Debug_addData(IMU_Cloud.Gyro.z, 6);
			Debug_show(6);
		}
		else if (Debug_DataType == DataType_ChassisData)
		{
			Debug_addData(M3508s[0].pid_angle.err, 1);
			Debug_addData(IMU_Chassis.Gyro.z, 2);
			Debug_show(2);
		}
		else if (Debug_DataType == DataType_VisionData)
		{
			Debug_addData(VisionData.Final_Offset.x, 1);
			Debug_addData(VisionData.Final_Offset.y, 2);
			Debug_addData(VisionData.ErrorChange_Rate.x, 3);
			Debug_addData(VisionData.ErrorChange_Rate.y, 4);
			Debug_addData(VisionExportData.FinalOffset.x, 5);
			Debug_addData(VisionExportData.FinalOffset.y, 6);
			//Debug_addData(VisionData.Vision_SpeedPid_Yaw.pwm, 7);
			Debug_addData(VisionData.RawData.x, 7);
			Debug_show(7);
		}
		else if (Debug_DataType == DataType_ShootData)
		{
			Debug_addData(shootUnit1.Reloader_Motor->realSpeed, 1);
			Debug_addData(shootUnit1.Reloader_Motor->realTorque, 2);
			Debug_addData(shootUnit1.Reloader.PillObstruct_Time,3);
			Debug_show(3);
		}


	}


}
*/