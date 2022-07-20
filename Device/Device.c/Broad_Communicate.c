#include "Broad_communicate.h"


//����can������֡�����ֽ���Ϊ8
//һ��float���ͱ�������4���ֽ�,��ôҲֻ�ܷ�2��float����

#if Communicate_way == 0
	Broad_Data_u UpBroad_Receive;
	Broad_Data_u DownBroad_Receive;
/*��һ�ַ��ͷ�ʽ��������*/
//�ϰ���
	#if UpBroad_Enable == 1
	//����
	void UpBroad_Send_Fun(Broad_Data_u UpBroad_Send)
	{
		//8��1�ֽڵĻ���ֲ�����
		uint8_t data[8];
	
		//Yaw��angle
		data[0] = UpBroad_Send.BraodData[0];
		data[1] = UpBroad_Send.BraodData[1];
		data[2] = UpBroad_Send.BraodData[2];
		data[3] = UpBroad_Send.BraodData[3];
	
		//Pitch��angle
		data[4] = UpBroad_Send.BraodData[4];
		data[5] = UpBroad_Send.BraodData[5];
		data[6] = UpBroad_Send.BraodData[6];
		data[7] = UpBroad_Send.BraodData[7];
	
		//��CANͨѶ����
		CAN_SendData(&hcan2,CAN_ID_STD,Up_Broad_ID,data);
	}
	//����
	void UpBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure)
	{
		if(CAN_Rx_Structure.CAN_RxMessage.StdId != Down_Broad_ID)
		{
			return;
		}
		
		UpBroad_Receive.BraodData[0] = CAN_Rx_Structure.CAN_RxMessageData[0];
		UpBroad_Receive.BraodData[1] = CAN_Rx_Structure.CAN_RxMessageData[1];
		UpBroad_Receive.BraodData[2] = CAN_Rx_Structure.CAN_RxMessageData[2];
		UpBroad_Receive.BraodData[3] = CAN_Rx_Structure.CAN_RxMessageData[3];
		UpBroad_Receive.BraodData[4] = CAN_Rx_Structure.CAN_RxMessageData[4];
		UpBroad_Receive.BraodData[5] = CAN_Rx_Structure.CAN_RxMessageData[5];
		UpBroad_Receive.BraodData[6] = CAN_Rx_Structure.CAN_RxMessageData[6];
		UpBroad_Receive.BraodData[7] = CAN_Rx_Structure.CAN_RxMessageData[7];
		
	}
	#endif
	
	#if DownBroad_Enable == 1
	//�°���
	void DownBroad_Send_Fun(Broad_Data_u DownBroad_Send)
	{
		//8��1�ֽڵĻ���ֲ�����
		uint8_t data[8];
	
		//Yaw��angle
		data[0] = DownBroad_Send.BraodData[0];
		data[1] = DownBroad_Send.BraodData[1];
		data[2] = DownBroad_Send.BraodData[2];
		data[3] = DownBroad_Send.BraodData[3];
	
		//Pitch��angle
		data[4] = DownBroad_Send.BraodData[4];
		data[5] = DownBroad_Send.BraodData[5];
		data[6] = DownBroad_Send.BraodData[6];
		data[7] = DownBroad_Send.BraodData[7];
	
		//��CANͨѶ����
		CAN_SendData(&hcan2,CAN_ID_STD,Down_Broad_ID,data);
	}
	//����
	void DownBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure)
	{
		if(CAN_Rx_Structure.CAN_RxMessage.StdId != Up_Broad_ID)
		{
			return;
		}
	
		DownBroad_Receive.BraodData[0] = CAN_Rx_Structure.CAN_RxMessageData[0];
		DownBroad_Receive.BraodData[1] = CAN_Rx_Structure.CAN_RxMessageData[1];
		DownBroad_Receive.BraodData[2] = CAN_Rx_Structure.CAN_RxMessageData[2];
		DownBroad_Receive.BraodData[3] = CAN_Rx_Structure.CAN_RxMessageData[3];
		DownBroad_Receive.BraodData[4] = CAN_Rx_Structure.CAN_RxMessageData[4];
		DownBroad_Receive.BraodData[5] = CAN_Rx_Structure.CAN_RxMessageData[5];
		DownBroad_Receive.BraodData[6] = CAN_Rx_Structure.CAN_RxMessageData[6];
		DownBroad_Receive.BraodData[7] = CAN_Rx_Structure.CAN_RxMessageData[7];
	
	}
	#endif
	
#endif

#if Communicate_way == 1
/*�ڶ��ַ�����ʽ��ָ��*/
	Broad_Data_t UpBroad_Receive;
	Broad_Data_t DownBroad_Receive;
	
	#if UpBroad_Enable == 1
	//�ϰ���
	//����
	void UpBroad_Send_Fun(Broad_Data_t UpBroad_Send)
	{
		//����ָ�붼ռ4���ֽ�
		unsigned char* p[2];
		uint8_t data[8];
	
		p[0] = (unsigned char*)&UpBroad_Send.example_1;
		p[1] = (unsigned char*)&UpBroad_Send.example_2;
	
		//Yaw��angle
		data[0] = *p[0];
		data[1] = *(p[0] + 1);
		data[2] = *(p[0] + 2);
		data[3] = *(p[0] + 3);
	
		//Pitch��angle
		data[4] = *p[1];
		data[5] = *(p[1] + 1);
		data[6] = *(p[1] + 2);
		data[7] = *(p[1] + 3);
	
		//��CANͨѶ����
		CAN_SendData(&hcan2,CAN_ID_STD,Up_Broad_ID,data);
	}
	//����
	void UpBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure)
	{
		if(CAN_Rx_Structure.CAN_RxMessage.StdId != Down_Broad_ID)
		{
			return;
		}
	
		unsigned char* p;
		p = CAN_Rx_Structure.CAN_RxMessageData;
		
		UpBroad_Receive.example_1 = (float)((*p >> 24)|(*(p+1) >> 16 )|(*(p+2) >> 8)|(*(p+3)));
		UpBroad_Receive.example_2 = (float)((*(p+4) >> 24 )|(*(p+5) >> 16 )|(*(p+6) >> 8)|(*(p+7)));
	
	}
	#endif

	#if DownBroad_Enable == 1
	//�°���
	void DownBroad_Send_Fun(Broad_Data_t DownBroad_Send)
	{
		//����ָ�붼ռ4���ֽ�
		unsigned char* p[2];
		uint8_t data[8];
	
		p[0] = (unsigned char*)&DownBroad_Send.example_1;
		p[1] = (unsigned char*)&DownBroad_Send.example_2;
	
		//Yaw��angle
		data[0] = *p[0];
		data[1] = *(p[0] + 1);
		data[2] = *(p[0] + 2);
		data[3] = *(p[0] + 3);
	
		//Pitch��angle
		data[4] = *p[1];
		data[5] = *(p[1] + 1);
		data[6] = *(p[1] + 2);
		data[7] = *(p[1] + 3);
	
		//��CANͨѶ����
		CAN_SendData(&hcan2,CAN_ID_STD,Down_Broad_ID,data);
	}
	//����
	void DownBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure)
	{
		if(CAN_Rx_Structure.CAN_RxMessage.StdId != Up_Broad_ID)
		{
			return;
		}
	
		unsigned char* p;
		p = CAN_Rx_Structure.CAN_RxMessageData;
		
		DownBroad_Receive.example_1 = (float)((*p >> 24)|(*(p+1) >> 16 )|(*(p+2) >> 8)|(*(p+3)));
		DownBroad_Receive.example_2 = (float)((*(p+4) >> 24 )|(*(p+5) >> 16 )|(*(p+6) >> 8)|(*(p+7)));
	}
	#endif

#endif
