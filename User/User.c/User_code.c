#include "User_code.h"


void total_Init(void)
{

	//DR16
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	USART_RX_DMA_ENABLE(&huart1,DR16_RxBUFF,DR16_RxBUFF_LEN+2);
	
	//CAN总线的初始化
	CAN1_Config();
	
	CAN2_Config();
	
	//视觉
	__HAL_UART_CLEAR_IDLEFLAG(&huart8);
	
	__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);
	
	USART_RX_DMA_ENABLE(&huart8,Vision_DataBuff,Vision_BuffSize);
	
	//编码器模式初始化
	//AB
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	
	//Snail电机的初始化
	Snail_Init();
	
	//上位机的初始化	
	__HAL_UART_ENABLE(&huart6);
	
	HAL_TIM_Base_Start_IT(&htim6);
		
	HAL_Delay(2000);
	
	//底盘初始化
	Chassis_Init();
	
	//云台的初始化
	Cloud_Init();
	
	//视觉的初始化
	Automatic_Aim_Parameter_Init();
	
	//拨盘的初始化
	Attack_Driver_Init();
	
	HAL_TIM_Base_Start_IT(&htim7);
	
}


void Global_Control(void)
{
	
	/* ----------------------------- 条件判断 -------------------------------- */
	//底盘条件控制
	Chassis_Control();
	
	//云台的条件控制
	Cloud_Control();
	
	//视觉的条件控制
	Vision_Control();
	
		//攻击
	//判断DR16的值，确定攻击模式
	Detectio_Attack_Sign();
	//发射
	Attack_setShootSpeed();
	//拨盘
	Attack_setDriverAngle(&Attack.Left_Driver);
	Attack_setDriverAngle(&Attack.Right_Driver);
	
	/* ---------------------------- 执行 ------------------------------------------------------ */
	//左 右 两枪管
	//左
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Snail.Final_OutputSpeed);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Snail.Final_OutputSpeed);
	//右
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Snail.Final_OutputSpeed);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Snail.Final_OutputSpeed);	
	
	//底盘的电流设置函数
	//拨盘
	M3508_setCurrent(M3508s[0].OutputCurrent,Attack.Left_Driver.Driver_EM->S_PID_t.Output,Attack.Right_Driver.Driver_EM->S_PID_t.Output,0);
	
	//云台的电压设置
	GM6020_SetVoltage(GM6020s[0].PSoutVoltage,GM6020s[1].PSoutVoltage,0,0);
	
}

