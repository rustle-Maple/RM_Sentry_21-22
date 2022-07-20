#include "DR16.h"

uint8_t DR16_RxBUFF[DR16_RxBUFF_LEN + 2];

DR16_t DR16;
// DR16锟斤拷锟捷斤拷锟斤拷
void DR16_Process(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.ch0 -= 1024;
	DR16.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
	DR16.ch1 -= 1024;

	DR16.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
	DR16.ch2 -= 1024;

	DR16.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
	DR16.ch3 -= 1024;
	DR16.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.Switch_Right = ((pData[5] >> 4) & 0x0003);

	DR16.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
	DR16.ch4_DW -= 1024;

	//	DR16.OffLine_Detection++;

	/* prevent remote control zero deviation */
	if (DR16.ch0 <= 20 && DR16.ch0 >= -20)
		DR16.ch0 = 0;
	if (DR16.ch1 <= 20 && DR16.ch1 >= -20)
		DR16.ch1 = 0;
	if (DR16.ch2 <= 20 && DR16.ch2 >= -20)
		DR16.ch2 = 0;
	if (DR16.ch3 <= 20 && DR16.ch3 >= -20)
		DR16.ch3 = 0;
	if (DR16.ch4_DW <= 20 && DR16.ch4_DW >= -20)
		DR16.ch4_DW = 0;

	//	DR16.OffLine_Detection++;
}

void DR16_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		// if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == 2)
		{
			DR16_Process(DR16_RxBUFF);
			send_to_C = 1;
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RxBUFF_LEN + 2);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}
