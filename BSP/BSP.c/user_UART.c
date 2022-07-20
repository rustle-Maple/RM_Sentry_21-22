#include "user_UART.h"

//USART的DMA使能 和 DMAx_Streamy数据流使能
//初始化 USART句柄 和
//补全 DMAx_Stream 变量中的 外设基地址 和 存储内存基地址
int USART_RX_DMA_ENABLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (huart->RxState == HAL_UART_STATE_READY)
	{

		/*输入的地址或者数据有问题的话*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		//初始化USART句柄的状态变量
		huart->pRxBuffPtr = pData; //将存储BUFF内存地址 赋给 USART句柄中的 pRxBuffPtr
		huart->RxXferSize = Size;  //将存储BUFF内存地址 赋给 USART句柄中的 RxXferSize

		//USART句柄的状态变量
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		

		//将外设基地址赋给源地址
		//将存储内存基地址赋给目标地址
		//使能DMA
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
		//使能USART_RX的DMA方
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	}

	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}
