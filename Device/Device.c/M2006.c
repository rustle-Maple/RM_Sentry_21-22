#include "M2006.h"

M2006_t M2006s[4];

//M2006 电流设置 
//参数：拨盘2个电机的电流值
void M2006_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
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
	CAN_SendData(&hcan1,CAN_ID_STD,M2006_SENDID,data);
	
}

//对 CAN 接收到 M2006的C610电调的数据 进行解析
void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	
	//判断接收到的报文ID 是否在M2006的ID范围内
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != (M2006_READID_START + 2))
		return;
	
	uint32_t EMID;				//存储电机ID索引
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M2006_READID_START);				//接收到ID - 开始ID 
	
	//将CAN收到的M2006反馈回的数据，解析到相依的电机物理变量中
	
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


