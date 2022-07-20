#include "Sensor.h"

uint8_t SENSOR_L_DataBuff[SENSOR_BuffSize];

sensor_t Sensor_L;

uint8_t SENSOR_R_DataBuff[SENSOR_BuffSize];

sensor_t Sensor_R;

//帧率结构体
Frame_rate_t Sensor_R_Framerate;
Frame_rate_t Sensor_L_Framerate;

//接收函数
void Sensor_L_DataReceive(uint8_t *data)
{
	//进行CRC校验
//	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//将视觉发送过来的13个8位数据遍历一遍
	for (uint8_t i = 0; i < 9; i++)
	{
		Sensor_L.RawData.SENSOR_RawData[i] = data[i];
	}
	//将Yaw\Pitch\Depth的高低八位合并
	Sensor_L.RawData.DIST = (Sensor_L.RawData.SENSOR_RawData[2] | (Sensor_L.RawData.SENSOR_RawData[3] << 8));
	Sensor_L.RawData.APM  = (Sensor_L.RawData.SENSOR_RawData[4] | (Sensor_L.RawData.SENSOR_RawData[5] << 8));
	Sensor_L.RawData.TEMP = (Sensor_L.RawData.SENSOR_RawData[6] | (Sensor_L.RawData.SENSOR_RawData[7] << 8));
	
	//获取其帧率
	Get_FPS(&Sensor_L_Framerate.times, &Sensor_L_Framerate.FPS);
	//离线检测
	Sensor_L_Framerate.Offline_Detec++;
	Sensor_L_Framerate.Offline_Flag = 0;
//	if(Sensor_L.RawData.APM > 1000)
//	{
//		Sensor_L.Last_Dist = Sensor_L.RawData.DIST;
//	}
//	
//	Sensor_L.RawData.DIST = Sensor_L.Last_Dist;
	
}
//接收函数
void Sensor_R_DataReceive(uint8_t *data)
{
	//进行CRC校验
//	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//将视觉发送过来的13个8位数据遍历一遍
	for (uint8_t i = 0; i < 9; i++)
	{
		Sensor_R.RawData.SENSOR_RawData[i] = data[i];
	}
	//将Yaw\Pitch\Depth的高低八位合并
	Sensor_R.RawData.DIST = (Sensor_R.RawData.SENSOR_RawData[2] | (Sensor_R.RawData.SENSOR_RawData[3] << 8));
	Sensor_R.RawData.APM = (Sensor_R.RawData.SENSOR_RawData[4] | (Sensor_R.RawData.SENSOR_RawData[5] << 8));
	Sensor_R.RawData.TEMP = (Sensor_R.RawData.SENSOR_RawData[6] | (Sensor_R.RawData.SENSOR_RawData[7] << 8));
	
	//获取其帧率
	Get_FPS(&Sensor_R_Framerate.times, &Sensor_R_Framerate.FPS);
	//离线检测
	Sensor_R_Framerate.Offline_Detec++;
	Sensor_R_Framerate.Offline_Flag = 0;
//		if(Sensor_R.RawData.APM > 1000)
//	{
//		Sensor_R.Last_Dist = Sensor_R.RawData.DIST;
//	}
//	
//	Sensor_R.RawData.DIST = Sensor_R.Last_Dist;
	
}

uint32_t q = 0;
//DMA数据流接收的数据处理
void SENSOR_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == 2)
		{
			q++;
			Sensor_R_DataReceive(SENSOR_R_DataBuff);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, SENSOR_BuffSize);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

//DMA数据流接收的数据处理
void SENSOL_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == 2)
		{
			Sensor_L_DataReceive(SENSOR_L_DataBuff);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, SENSOR_BuffSize);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

