#include "user_UART.h"

//USART��DMAʹ�� �� DMAx_Streamy������ʹ��
//��ʼ�� USART��� ��
//��ȫ DMAx_Stream �����е� �������ַ �� �洢�ڴ����ַ
int USART_RX_DMA_ENABLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (huart->RxState == HAL_UART_STATE_READY)
	{

		/*����ĵ�ַ��������������Ļ�*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		//��ʼ��USART�����״̬����
		huart->pRxBuffPtr = pData; //���洢BUFF�ڴ��ַ ���� USART����е� pRxBuffPtr
		huart->RxXferSize = Size;  //���洢BUFF�ڴ��ַ ���� USART����е� RxXferSize

		//USART�����״̬����
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		

		//���������ַ����Դ��ַ
		//���洢�ڴ����ַ����Ŀ���ַ
		//ʹ��DMA
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
		//ʹ��USART_RX��DMA��
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	}

	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}
