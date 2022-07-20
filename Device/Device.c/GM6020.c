#include "GM6020.h"

GM6020_t GM6020s[4];

void GM6020_SetVoltage(int16_t Vid1,int16_t Vid2,int16_t Vid3,int16_t Vid4)
{
	uint8_t data[8];
	
	data[0] = Vid1 >> 8;
	data[1] = Vid1;
	data[2] = Vid2 >> 8;
	data[3] = Vid2;
	data[4] = Vid3 >> 8;
	data[5] = Vid3;
	data[6] = Vid4 >> 8;
	data[7] = Vid4;
	
	CAN_SendData(&hcan1,CAN_ID_STD,GM6020_SENDID,data);
	
}


/* ------------------------------------------------------------------------------------------------------------------------------------- */
//云台 Pitch轴
//以右侧6020为主动电机，以左侧为从动电机
void GM6020_Pitch_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != (GM6020_READID_START+1))
		return;
	
	int32_t EMID;
	EMID = CAN_Rx_Structure.CAN_RxMessage.StdId - GM6020_READID_START;
	
	GM6020s[EMID].readAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	GM6020s[EMID].readSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	GM6020s[EMID].freadSpeed = (float)GM6020s[EMID].readSpeed;
	GM6020s[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	GM6020s[EMID].readTemperture = CAN_Rx_Structure.CAN_RxMessageData[6];
	
	//根据实际装好后，Pitch轴的上下限位去改值
	//其Pitch轴最高为600,最低为1880，
	//但是不可以取-1880，因为-1880太小了，不足以成为过零处理的判断值
	if(GM6020s[EMID].readAngle - GM6020s[EMID].lastAngle < -7200)						
	{
		GM6020s[EMID].turnCount++;
	}
	if(GM6020s[EMID].lastAngle - GM6020s[EMID].readAngle < -7200)
	{
		GM6020s[EMID].turnCount--;
	}
	
	GM6020s[EMID].totalAngle = GM6020s[EMID].readAngle + (8192 * GM6020s[EMID].turnCount);
	
	GM6020s[EMID].lastAngle = GM6020s[EMID].readAngle;
	
}


//云台 Yaw轴
//以右侧6020为主动电机，以左侧为从动电机
void GM6020_Yaw_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != GM6020_READID_START)
		return;
	
	int32_t EMID;
	EMID = CAN_Rx_Structure.CAN_RxMessage.StdId - GM6020_READID_START;
	
	GM6020s[EMID].readAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	GM6020s[EMID].readSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	GM6020s[EMID].freadSpeed = (float)GM6020s[EMID].readSpeed;
	GM6020s[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	GM6020s[EMID].readTemperture = CAN_Rx_Structure.CAN_RxMessageData[6];
	
	if(GM6020s[EMID].readAngle - GM6020s[EMID].lastAngle < -6500)								
	{
		GM6020s[EMID].turnCount++;
	}
	if(GM6020s[EMID].lastAngle - GM6020s[EMID].readAngle < -6500)
	{
		GM6020s[EMID].turnCount--;
	}
	
	GM6020s[EMID].totalAngle = GM6020s[EMID].readAngle + (8192 * GM6020s[EMID].turnCount);
	
	GM6020s[EMID].lastAngle = GM6020s[EMID].readAngle;
		
}



