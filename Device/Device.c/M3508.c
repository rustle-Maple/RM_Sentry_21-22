#include "M3508.h"

M3508_t M3508s[4];

//M3508 电流设置 
//参数：底盘四个电机的电流值
//但是,哨兵只有一个
void M3508_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	//根据M3508的电调的数据格式，将应该发送給M3508电调的数据进行解析
	data[0] = Iid1 >> 8;				//控制电流值高8位
	data[1] = Iid1;							//控制电流值低8位
	data[2] = Iid2 >> 8;
	data[3] = Iid2;
	data[4] = Iid3 >> 8;
	data[5] = Iid3;
	data[6] = Iid4 >> 8;
	data[7] = Iid4;
	
	//CAN数据帧发送函数
	//参数1：用CAN1发送
	//参数2：ID类型：标准ID
	//参数3：ID
	//参数4：要发送的数据 数组
	CAN_SendData(&hcan1,CAN_ID_STD,M3508_SENDID,data);
	
}

//对 CAN 接收到 M3508电调的数据 进行解析
void M3508_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	//判断接收到的报文ID 是否在M3508的ID范围内
	if((CAN_Rx_Structure.CAN_RxMessage.StdId != M3508_READID_START+3) && (CAN_Rx_Structure.CAN_RxMessage.StdId != M3508_READID_START+2) )
		return;
	
	uint32_t EMID;				//存储电机ID索引
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M3508_READID_START);				//接收到ID - 开始ID 
	
	//将CAN收到的M3508反馈回的数据，解析到相依的电机物理变量中
	M3508s[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	M3508s[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	M3508s[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	M3508s[EMID].temperture = CAN_Rx_Structure.CAN_RxMessageData[6];

	if(M3508s[EMID].realAngle - M3508s[EMID].last_angle > 4096)
  M3508s[EMID].turnCount --;
 else if (M3508s[EMID].realAngle - M3508s[EMID].last_angle < -4096)
  M3508s[EMID].turnCount ++;
 M3508s[EMID].totalAngle = M3508s[EMID].turnCount * 8192 + M3508s[EMID].realAngle;
  M3508s[EMID].last_angle=M3508s[EMID].realAngle;
//	M3508s[EMID].OffLine_Detection++;	

}


/* ---------------------------------------------------------------------------------------------------- */

M3508s1_t M3508s1[4];

//摩擦轮M3508电机，电流设置
void M3508s1_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	//根据M3508的电调的数据格式，将应该发送給M3508电调的数据进行解析
	data[0] = Iid1 >> 8;				//控制电流值高8位
	data[1] = Iid1;							//控制电流值低8位
	data[2] = Iid2 >> 8;
	data[3] = Iid2;
	data[4] = Iid3 >> 8;
	data[5] = Iid3;
	data[6] = Iid4 >> 8;
	data[7] = Iid4;
	
	//CAN数据帧发送函数
	//参数1：用CAN1发送
	//参数2：ID类型：标准ID
	//参数3：ID
	//参数4：要发送的数据 数组
	CAN_SendData(&hcan1,CAN_ID_STD,M3508s1_SENDID,data);
	
}

//摩擦轮：对 CAN 接收到 M3508电调的数据 进行解析
void M3508s1_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	//判断接收到的报文ID 是否在M3508的ID范围内
	if(CAN_Rx_Structure.CAN_RxMessage.StdId < M3508s1_START || CAN_Rx_Structure.CAN_RxMessage.StdId > M3508s1_END)
		return;
	
	uint32_t EMID;				//存储电机ID索引
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M3508s1_START);				//接收到ID - 开始ID 
	
	//将CAN收到的M3508反馈回的数据，解析到相依的电机物理变量中
	M3508s1[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	M3508s1[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	M3508s1[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	M3508s1[EMID].temperture = CAN_Rx_Structure.CAN_RxMessageData[6];

	M3508s1[EMID].OffLine_Status = 0;
	M3508s1[EMID].OffLine_Detection++;
	
}


