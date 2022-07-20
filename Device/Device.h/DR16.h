#ifndef DR16_H
#define DR16_H

#include "main.h"
#include "CAN2_SEND.h"

//���������¶�Ӧֵ
#define DR16_SWITCH_UP 1
#define DR16_SWITCH_MID 3
#define DR16_SWITCH_DOWN 2

#define ch4_DW_UP -660
#define ch4_DW_MID 0
#define ch4_DW_DOWN 660

typedef struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	uint8_t Switch_Left;
	uint8_t Switch_Right;

	int16_t ch4_DW; //����

	uint16_t OffLine_Detection; //���߼��
	uint8_t OffLine_Status;		//���߱�־λ

} DR16_t;
extern DR16_t DR16;

#define DR16_RxBUFF_LEN 18
extern uint8_t DR16_RxBUFF[DR16_RxBUFF_LEN + 2];

void DR16_Process(uint8_t *pData);

void DR16_Handler(UART_HandleTypeDef *huart);

#endif
