#include "User_code.h"


void total_Init(void)
{

	//DR16
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	
	USART_RX_DMA_ENABLE(&huart1,DR16_RxBUFF,DR16_RxBUFF_LEN+2);
	
	//CAN���ߵĳ�ʼ��
	CAN1_Config();
	
	CAN2_Config();
	
	//�Ӿ�
	__HAL_UART_CLEAR_IDLEFLAG(&huart8);
	
	__HAL_UART_ENABLE_IT(&huart8,UART_IT_IDLE);
	
	USART_RX_DMA_ENABLE(&huart8,Vision_DataBuff,Vision_BuffSize);
	
	//������ģʽ��ʼ��
	//AB
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	
	//Snail����ĳ�ʼ��
	Snail_Init();
	
	//��λ���ĳ�ʼ��	
	__HAL_UART_ENABLE(&huart6);
	
	HAL_TIM_Base_Start_IT(&htim6);
		
	HAL_Delay(2000);
	
	//���̳�ʼ��
	Chassis_Init();
	
	//��̨�ĳ�ʼ��
	Cloud_Init();
	
	//�Ӿ��ĳ�ʼ��
	Automatic_Aim_Parameter_Init();
	
	//���̵ĳ�ʼ��
	Attack_Driver_Init();
	
	HAL_TIM_Base_Start_IT(&htim7);
	
}


void Global_Control(void)
{
	
	/* ----------------------------- �����ж� -------------------------------- */
	//������������
	Chassis_Control();
	
	//��̨����������
	Cloud_Control();
	
	//�Ӿ�����������
	Vision_Control();
	
		//����
	//�ж�DR16��ֵ��ȷ������ģʽ
	Detectio_Attack_Sign();
	//����
	Attack_setShootSpeed();
	//����
	Attack_setDriverAngle(&Attack.Left_Driver);
	Attack_setDriverAngle(&Attack.Right_Driver);
	
	/* ---------------------------- ִ�� ------------------------------------------------------ */
	//�� �� ��ǹ��
	//��
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,Snail.Final_OutputSpeed);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,Snail.Final_OutputSpeed);
	//��
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,Snail.Final_OutputSpeed);
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,Snail.Final_OutputSpeed);	
	
	//���̵ĵ������ú���
	//����
	M3508_setCurrent(M3508s[0].OutputCurrent,Attack.Left_Driver.Driver_EM->S_PID_t.Output,Attack.Right_Driver.Driver_EM->S_PID_t.Output,0);
	
	//��̨�ĵ�ѹ����
	GM6020_SetVoltage(GM6020s[0].PSoutVoltage,GM6020s[1].PSoutVoltage,0,0);
	
}

