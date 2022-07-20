#ifndef USER_CAN_H
#define USER_CAN_H

#include "main.h"
#include "can.h"

typedef struct
{
	CAN_TxHeaderTypeDef CAN_TxMessage;				//CAN ���ͻ��������
	uint8_t CAN_TxMessageData[8];				//�洢CAN�������� ����
}CAN_Tx_Typedef;

//û��Ҫ������ļ�û��Ҫ�õ��������������ʱ����Ϊȫ��
//extern CAN_Tx_Typedef CAN_Tx_Structure;				//CAN���ͽṹ�� 

typedef struct
{
//	uint8_t CANx;															//ָ���ĸ�CAN
	CAN_RxHeaderTypeDef CAN_RxMessage;				//CAN ���ջ��������
	uint8_t CAN_RxMessageData[8];				//�洢CAN�������� ����
}CAN_Rx_TypeDef;

//û�б�Ҫ������Ϊȫ�ֱ���,ֻ���ڵ��Լ��Ҫ�۲����ʱ����Ϊȫ��
//extern CAN_Rx_TypeDef CAN1_Rx_Structure;				//CAN1���սṹ��
//extern CAN_Rx_TypeDef CAN2_Rx_Structure;				//CAN2���սṹ��

void CAN1_Filter0_Init(void);

void CAN1_Config(void);

void CAN2_Filter0_Init(void);

void CAN2_Config(void);

void CAN_SendData(CAN_HandleTypeDef* hcanx,uint8_t id_type,uint32_t id,uint8_t data[8]);

#endif


