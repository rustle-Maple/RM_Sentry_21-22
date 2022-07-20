#include "M2006.h"

M2006_t M2006s[4];

//M2006 �������� 
//����������2������ĵ���ֵ
void M2006_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	//����M3508�ĵ�������ݸ�ʽ����Ӧ�÷��ͽoM3508��������ݽ��н���
	data[0] = Iid1 >> 8;				//���Ƶ���ֵ��8λ
	data[1] = Iid1;							//���Ƶ���ֵ��8λ
	data[2] = Iid2 >> 8;
	data[3] = Iid2;
	data[4] = Iid3 >> 8;
	data[5] = Iid3;
	data[6] = Iid4 >> 8;
	data[7] = Iid4;
	
	//CAN����֡���ͺ���
	//����1����CAN1����
	//����2��ID���ͣ���׼ID
	//����3��ID
	//����4��Ҫ���͵����� ����
	CAN_SendData(&hcan1,CAN_ID_STD,M2006_SENDID,data);
	
}

//�� CAN ���յ� M2006��C610��������� ���н���
void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	
	//�жϽ��յ��ı���ID �Ƿ���M2006��ID��Χ��
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != (M2006_READID_START + 2))
		return;
	
	uint32_t EMID;				//�洢���ID����
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M2006_READID_START);				//���յ�ID - ��ʼID 
	
	//��CAN�յ���M2006�����ص����ݣ������������ĵ�����������
	
	M2006s[EMID].last_totalAngle = M2006s[EMID].totalAngle;
	
	M2006s[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	M2006s[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	
	if(M2006s[EMID].realAngle - M2006s[EMID].lastAngle < -6000)
	{
		M2006s[EMID].turnCount++;
	}
	if(M2006s[EMID].lastAngle - M2006s[EMID].realAngle < -6000)
	{
		M2006s[EMID].turnCount--;
	}
	
	M2006s[EMID].totalAngle = M2006s[EMID].realAngle + M2006s[EMID].turnCount * 8192;
	
	M2006s[EMID].lastAngle = M2006s[EMID].realAngle;
		
//	M2006s[EMID].OffLine_Detection++;
	
}


