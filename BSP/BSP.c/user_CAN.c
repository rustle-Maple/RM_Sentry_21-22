#include "user_CAN.h"

//��Ҫ�����ԣ��Ͱ�����������Ϊȫ��
//CAN_Tx_Typedef CAN_Tx_Structure;				//CAN�ķ��ͽṹ��

//CAN1_Filter ��ʼ��
void CAN1_Filter0_Init(void)
{
	//CAN_Filter ��ʼ���ṹ��
	CAN_FilterTypeDef CANx_Filter;
	
	CANx_Filter.SlaveStartFilterBank = 14;				//Filter ������14���µ� ���䵽CAN1
	CANx_Filter.FilterBank = 0;				//ѡ��Filter0
	
	CANx_Filter.FilterMode = CAN_FILTERMODE_IDMASK;				//Filter ��ʶ������ģʽ
	CANx_Filter.FilterScale = CAN_FILTERSCALE_32BIT;				//Filter λ��ѡ��32λ
	
	CANx_Filter.FilterIdHigh = 0x0000;				//��ʶ��ID��16λ
	CANx_Filter.FilterIdLow = 0x0000;					
	CANx_Filter.FilterMaskIdHigh = 0x0000;					//�����16λ
	CANx_Filter.FilterMaskIdLow = 0x0000;
	
	CANx_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;				//������FIFO0
	
	CANx_Filter.FilterActivation = ENABLE;				//CANʹ��
	
	//CAN_Filter ��ʼ������
	if(HAL_CAN_ConfigFilter(&hcan1,&CANx_Filter) != HAL_OK)
	{
		while(1)
		{
		}
	}
}

//CAN1_Filter ��ʼ��
void CAN2_Filter0_Init(void)
{
	//CAN_Filter ��ʼ���ṹ��
	CAN_FilterTypeDef CANx_Filter;
	
	CANx_Filter.SlaveStartFilterBank = 0;				//Filter ������14���µ� ���䵽CAN1
	CANx_Filter.FilterBank = 0;				//ѡ��Filter0
	
	CANx_Filter.FilterMode = CAN_FILTERMODE_IDMASK;				//Filter ��ʶ������ģʽ
	CANx_Filter.FilterScale = CAN_FILTERSCALE_32BIT;				//Filter λ��ѡ��32λ
	
	CANx_Filter.FilterIdHigh = 0x0000;				//��ʶ��ID��16λ
	CANx_Filter.FilterIdLow = 0x0000;					
	CANx_Filter.FilterMaskIdHigh = 0x0000;					//�����16λ
	CANx_Filter.FilterMaskIdLow = 0x0000;
	
	CANx_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;				//������FIFO0
	
	CANx_Filter.FilterActivation = ENABLE;				//CANʹ��
	
	//CAN_Filter ��ʼ������
	if(HAL_CAN_ConfigFilter(&hcan2,&CANx_Filter) != HAL_OK)
	{
		while(1)
		{
		}
	}
}

//CAN��������֡����
//����1��ѡ��Ҫ���͵�CAN
//����2��Ҫ��������֡�� ID����
//����3��Ҫ���͵�����֡ ID
//����4��Ҫ���͵����� ����
CAN_Tx_Typedef CAN_Tx_Structure;				//CAN�ķ��ͽṹ��
void CAN_SendData(CAN_HandleTypeDef* hcanx,uint8_t id_type,uint32_t id,uint8_t data[8])
{
	//����Ҫ��⣬ֱ�����еĻ�����Ϊ�ֲ�����������
//	CAN_Tx_Typedef CAN_Tx_Structure;				//CAN�ķ��ͽṹ��
	
	uint32_t Len;				//��������֡����
	uint8_t i;				//ѭ�����Ʋ���
	uint32_t TxMailbox;				//����CAN����FIFO
	
	CAN_Tx_Structure.CAN_TxMessage.RTR = CAN_RTR_DATA;				//֡���ͣ�����֡
	CAN_Tx_Structure.CAN_TxMessage.IDE = id_type;				//ID���ͣ���׼ �� ��չ
	
	if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_STD)				//��Ϊ��׼ID
	{
		CAN_Tx_Structure.CAN_TxMessage.StdId = id;								
	}
	else if(CAN_Tx_Structure.CAN_TxMessage.IDE == CAN_ID_EXT)				//��Ϊ��չID
	{
		CAN_Tx_Structure.CAN_TxMessage.ExtId = id;
	}
	
	CAN_Tx_Structure.CAN_TxMessage.DLC = 0x08;				//ȷ��Ҫ���͵�֡���ȣ�max 8���ֽڣ�
	
	CAN_Tx_Structure.CAN_TxMessage.TransmitGlobalTime = DISABLE;				//��ʱ�䴥��
	
	Len = CAN_Tx_Structure.CAN_TxMessage.DLC;
	for(i = 0 ; i < Len ; i++)
	{
		CAN_Tx_Structure.CAN_TxMessageData[i] = data[i];				//��Ҫ���͵����ݴ��ݽo �洢CAN�������� ����
	}
	
	//���һ�����е�Tx�������һ����Ϣ�����Ҽ��������Ĵ�������
	HAL_CAN_AddTxMessage(hcanx,&CAN_Tx_Structure.CAN_TxMessage,CAN_Tx_Structure.CAN_TxMessageData,&TxMailbox);
	
}

//CAN1 Filter0��ʼʹ�ܡ�CAN1ʹ�� �� CAN1��Ϣ�����ж�
void CAN1_Config(void)
{
	//CAN1_Filter0 ��ʼ�� ʹ��
	CAN1_Filter0_Init();
	
	//CAN1ʹ��
	if(HAL_CAN_Start(&hcan1)!= HAL_OK)
	{
		while(1)
		{
		}
	}
	
	//ʹ��CAN1��Ϣ�����ж�
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);         //  CAN_IT_FMP0
//	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//		while(1)
//		{
//		}
//	}
}

void CAN2_Config(void)
{
	//CAN2_Filter0 ��ʼ�� ʹ��
	CAN2_Filter0_Init();
	
	//CAN2ʹ��
	if(HAL_CAN_Start(&hcan2)!= HAL_OK)
	{
		while(1)
		{
		}
	}
	
	//ʹ��CAN2��Ϣ�����ж�
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);         //  CAN_IT_FMP0
//	if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//		while(1)
//		{
//		}
//	}
}
