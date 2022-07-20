#ifndef BROAD_COMMUNICATE_H
#define BROAD_COMMUNICATE_H

#include "main.h"
#include "can.h"
#include "user_can.h"
#pragma anon_unions


#define Communicate_way 0
#define UpBroad_Enable 1
#define DownBroad_Enable 0

#define Up_Broad_ID 0x295
#define Down_Broad_ID 0x265

#if Communicate_way == 0

//以联合体形式发送，可以将float 类型的数据 转成 以uint8_t类型去发送
typedef union
{
	struct
	{
		float example_1;
		float example_2;
	};
	uint8_t BraodData[8];
}Broad_Data_u;
	#if UpBroad_Enable== 1
	//发送
	extern Broad_Data_u UpBroad_Send;
	//接收
	extern Broad_Data_u UpBroad_Receive;

	void UpBroad_Send_Fun(Broad_Data_u UpBroad_Send);
	void UpBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure);
	#endif
	
	#if DownBroad_Enable == 1
	extern Broad_Data_u DownBroad_Send;
	extern Broad_Data_u DownBroad_Receive;

	void DownBroad_Send_Fun(Broad_Data_u DownBroad_Send);
	void DownBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure);
	#endif
#endif

#if Communicate_way == 1

typedef struct
{
	float example_1;
	float example_2;
}Broad_Data_t;

	#if UpBroad_Enable == 1
	//发送
	extern Broad_Data_t UpBroad_Send;
	//接收
	extern Broad_Data_t UpBroad_Receive;

	void UpBroad_Send_Fun(Broad_Data_t UpBroad_Send);
	void UpBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure);
	#endif

	#if DownBroad_Enable == 1
	extern Broad_Data_t DownBroad_Send;
	extern Broad_Data_t DownBroad_Receive;

	void DownBroad_Send_Fun(Broad_Data_t DownBroad_Send);
	void DownBroad_Receive_Fun(CAN_Rx_TypeDef CAN_Rx_Structure);
	#endif

#endif


#endif

