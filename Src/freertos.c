/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "User_common.h"
#include "Handle.h"
#include "BREAK.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// CAN���о��
osMessageQId CAN1_Queue;
osMessageQId CAN2_Queue;
//����������
osThreadId RobotCtrl_Handle;
//������ں���
void Robot_Control(void const *argument);

/* USER CODE END Variables */
osThreadId Task_Robot_InitHandle;
osThreadId Task_Can1_ReceiveHandle;
osThreadId Task_Can2_ReiveceHandle;
osThreadId Task_Show_DebugHandle;
osThreadId Task_Data_UpdataHandle;
osThreadId Task_Vision_SendDHandle;
osThreadId Robot_Detec_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Robot_Init(void const *argument);
void Can1_Receive(void const *argument);
void Can2_Reivece(void const *argument);
void Show_Debug(void const *argument);
void Data_Updata(void const *argument);
void Vision_SendD(void const *argument);
void Robot_Detec(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	CAN1_Queue = xQueueCreate(32, sizeof(CAN_Rx_TypeDef));
	CAN2_Queue = xQueueCreate(32, sizeof(CAN_Rx_TypeDef));

	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of Task_Robot_Init */
	osThreadDef(Task_Robot_Init, Robot_Init, osPriorityNormal, 0, 128);
	Task_Robot_InitHandle = osThreadCreate(osThread(Task_Robot_Init), NULL);

	/* definition and creation of Task_Can1_Receive */
	osThreadDef(Task_Can1_Receive, Can1_Receive, osPriorityHigh, 0, 256);
	Task_Can1_ReceiveHandle = osThreadCreate(osThread(Task_Can1_Receive), NULL);

	/* definition and creation of Task_Can2_Reivece */
	osThreadDef(Task_Can2_Reivece, Can2_Reivece, osPriorityHigh, 0, 256);
	Task_Can2_ReiveceHandle = osThreadCreate(osThread(Task_Can2_Reivece), NULL);

	/* definition and creation of Task_Show_Debug */
	osThreadDef(Task_Show_Debug, Show_Debug, osPriorityBelowNormal, 0, 128);
	Task_Show_DebugHandle = osThreadCreate(osThread(Task_Show_Debug), NULL);

	/* definition and creation of Task_Data_Updata */
	osThreadDef(Task_Data_Updata, Data_Updata, osPriorityHigh, 0, 256);
	Task_Data_UpdataHandle = osThreadCreate(osThread(Task_Data_Updata), NULL);

	/* definition and creation of Task_Vision_SendD */
	osThreadDef(Task_Vision_SendD, Vision_SendD, osPriorityNormal, 0, 128);
	Task_Vision_SendDHandle = osThreadCreate(osThread(Task_Vision_SendD), NULL);

	/* definition and creation of Robot_Detec_Task */
	osThreadDef(Robot_Detec_Task, Robot_Detec, osPriorityAboveNormal, 0, 128);
	Robot_Detec_TaskHandle = osThreadCreate(osThread(Robot_Detec_Task), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_Robot_Init */
/**
 * @brief  Function implementing the Task_Robot_Init thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Robot_Init */
void Robot_Init(void const *argument)
{
	/* USER CODE BEGIN Robot_Init */
	/* Infinite loop */

	// DR16
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart1, DR16_RxBUFF, DR16_RxBUFF_LEN + 2);

	// Sensor_R
	__HAL_UART_CLEAR_IDLEFLAG(&huart7);

	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart7, SENSOR_R_DataBuff, SENSOR_BuffSize);

	// Sensor_L
	__HAL_UART_CLEAR_IDLEFLAG(&huart8);

	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart8, SENSOR_L_DataBuff, SENSOR_BuffSize);
		
	
		//��21������
		//�Ӿ�
//		__HAL_UART_CLEAR_IDLEFLAG(&huart8);

//		__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

//		USART_RX_DMA_ENABLE(&huart8, Vision_DataBuff, Vision_BuffSize);
	

	//����ϵͳ
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart3, JudgeSystem_rxBuff, JUDGESYSTEM_PACKSIZE);

	//������ģʽ��ʼ��
	// AB
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

	//��ʹ���豸��ȷ��CAN��������ͨѶ�����Ի�ȡ�������ǰ������
	// CAN���ߵĳ�ʼ��
	CAN1_Config();

	CAN2_Config();

	//��λ���ĳ�ʼ��
	__HAL_UART_ENABLE(&huart6);

	//����ʹ����CAN�жϣ�����CAN�жϻ������񣬶�����CANͨѶ���Ӷ��������
	//��ȡ���ݺ�ᷢ�����У������в�Ϊ�գ���CAN_Reivece����ͽ������̬
	//��CAN_Reivece�и��ݶ�Ӧ��ID������ͬ�����ݣ��Ӷ���ȡ�����������
	//���˹�����Ҫʱ�䣬�õȴ������ȶ�
	HAL_Delay(6000);

	//���Ƶĳ�ʼ��
	//���˵������ǰ�ĳ�ʼ���ݣ�����Ҫ�Կ��ƽ��г�ʼ������ֹը��
	//��Ϊ��CAN����������ʱ��CAN_Reivece�Ϳ��Խ������ݣ�����CAN_Reivece�������ȼ��ø���Robot_Init����
	//�������ʼ������ȴ���ܱ������ȼ���������
	//����Ӧ�ý����ٽ���
	//Ϊʲô�����ٽ����Ϳ����أ���Ϊ�����ٽ�����CAN�жϾͻᱻ���Σ�CAN�жϱ������˾��޷��������ݣ�CAN���о������ݣ�CAN_Reivece����ͱ�����
	taskENTER_CRITICAL(); //�����ٽ���

	//����״̬�ĳ�ʼ��
	Robots_Control_FUN.Init_RobotStatus();
	//��̨�ĳ�ʼ��
	Cloud_FUN.Cloud_Init();
	//���̳�ʼ��
	Chassis_FUN.Chassis_Init();
	//�Ӿ���ʼ��
	Vision_Control_Init();
	//������ʼ��
	Attack_FUN.Attack_Init();

	//���Ƴ�ʼ����ɺ�Ҫ������������
	//���ڿ�����������ȼ�ֻ����Ϊ��ߣ���Ȼ�����������Ʈ��
	//����ֻ���ڿ��Ƴ�ʼ����֮��ſ��Կ��ơ�
	//����ֻ���ڿ��Ƴ�ʼ����ȥ����������
	/*
	xTaskCreate((TaskFunction_t)Robot_Control,
				(const char *)"Task_Robot_Control",
				(configSTACK_DEPTH_TYPE)RobotCtrl_Size,
				(void *)NULL,
				(UBaseType_t)RobotCtrl_Priority,
				(TaskHandle_t *)&RobotCtrl_Handle);
	*/
	osThreadDef(Task_Robot_Control, Robot_Control, RobotCtrl_Priority, 0, RobotCtrl_Size);
	RobotCtrl_Handle = osThreadCreate(osThread(Task_Robot_Control), NULL);

	taskEXIT_CRITICAL(); //�˳��ٽ���

	//���ڳ�ʼ������ֻ��ִ��һ�Σ�ִ�����Ϳ���ɾ����
	vTaskDelete(NULL);

	//���ڳ�ʼ������ֻ��ִ��һ�Σ�����������ѭ��
	//  for(;;)
	//  {
	//    osDelay(1);
	//  }
	/* USER CODE END Robot_Init */
}

/* USER CODE BEGIN Header_Can1_Receive */
/**
 * @brief Function implementing the Task_Can1_Receive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Can1_Receive */
void Can1_Receive(void const *argument)
{
	/* USER CODE BEGIN Can1_Receive */

	// CAN1���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN1_Rx_Structure;

	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;

	/* Infinite loop */
	for (;;)
	{
		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN1_Queue, &CAN1_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�
		if (ExitQueue_Status == pdTRUE)
		{
			/*Cloud*/
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START)
			{
				//��̨��yaw��
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (GM6020_READID_START + 1))
			{
				//��̨Pitch��
				GM6020_Pitch_getInfo(CAN1_Rx_Structure);
			}
			/*Shoot*/
//			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == M3508s1_START || CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508s1_START + 1))
//			{
//				M3508s1_getInfo(CAN1_Rx_Structure);
//			}
//			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == M2006_READID_START + 2)
//			{
//				M2006_getInfo(CAN1_Rx_Structure);
//			}
			/*Chassis*/
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == M3508_READID_START + 3 || CAN1_Rx_Structure.CAN_RxMessage.StdId == M3508_READID_START + 2)
			{
				M3508_getInfo(CAN1_Rx_Structure);
			}
			
		}

		//    osDelay(1);
	}
	/* USER CODE END Can1_Receive */
}

/* USER CODE BEGIN Header_Can2_Reivece */
/**
 * @brief Function implementing the Task_Can2_Reivece thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Can2_Reivece */
void Can2_Reivece(void const *argument)
{
	/* USER CODE BEGIN Can2_Reivece */
	// CAN2���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN2_Rx_Structure;
	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;

	/* Infinite loop */
	for (;;)
	{
		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN2_Queue, &CAN2_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�
		if (ExitQueue_Status == pdTRUE)
		{
			/*DJIC_IMU*/
			// IMU_Euler
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Angle)
			{
				DJI_C_Euler_getInfo(CAN2_Rx_Structure);
			}
			// Gyro
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Gyro)
			{
				DJI_C_Gyro_getInfo(CAN2_Rx_Structure);
			}
		}

		//    osDelay(1);
	}
	/* USER CODE END Can2_Reivece */
}

/* USER CODE BEGIN Header_Show_Debug */
/**
 * @brief Function implementing the Task_Show_Debug thread.
 * @param argument: Not used
 * @retval None
 */

// uint32_t Control_FPS;
// uint32_t Control_LastWorldTimes;
/* USER CODE END Header_Show_Debug */
void Show_Debug(void const *argument)
{
	/* USER CODE BEGIN Show_Debug */
	/* Infinite loop */
	for (;;)
	{
		//		Debug_addData(Cloud.Yaw_Angle_pid->result,2);
		//		Debug_addData (*Cloud.gyro_z,3);
		//		Debug_addData(ext_game_robot_state.data.shooter_id1_17mm_cooling_rate,2);
		//		Debug_addData(ext_game_robot_state.data.shooter_id1_17mm_cooling_limit,3);
		//		Debug_addData(Barrel.user_realHeat,4);
		//		Debug_addData(Barrel.judge_realHeat,5);
		//
		//		Debug_show(6);
		ANO_Send_Data_Init(0, 0, ext_power_heat_data.data.chassis_power_buffer, 0, Chassis.EM->targetSpeed, Chassis.EM->realSpeed, 0, 0);
		ANO_Send_Data_V4();

		//������ʲ��ɹ���
		vTaskDelay(5);
	}
	/* USER CODE END Show_Debug */
}

/* USER CODE BEGIN Header_Data_Updata */
/**
 * @brief Function implementing the Task_Data_Updata thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Data_Updata */
void Data_Updata(void const *argument)
{
	/* USER CODE BEGIN Data_Updata */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); //ÿʮ����ǿ�ƽ����ܿ���
	/* Infinite loop */
	for (;;)
	{
		//���´���������
		Chassis_FUN.Updata_Chassis_Sensor();
		//��������������
		Updata_Hand_Euler_Gyro_Data();
		//�����Ӿ�����
		Updata_VisionData_Hand();
		//���·��͸��Ӿ�������������
		// Updata_SendVision_Stata();

		//���ݸ�������ֻ�������´������ľͿ����ˣ��������Ʊ�־λ�ģ���Ҫ�ڴ˸��£��ڿ����и��£���֤ʱ���һ��
		//���¿���״̬
		Robots_Control_FUN.Updata_RobotStatus();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
	/* USER CODE END Data_Updata */
}

/* USER CODE BEGIN Header_Vision_SendD */
/**
 * @brief Function implementing the Task_Vision_SendD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Vision_SendD */
void Vision_SendD(void const *argument)
{
	/* USER CODE BEGIN Vision_SendD */
	/* Infinite loop */
	for (;;)
	{
		//  Update_Vision_SendData();
		if (send_to_C == 1) //ң��������
		{
			DR16_send_master_control();
			// send_to_C_times++;
			send_to_C = 0;
		}
		if (send_to_C_JS_HURT == 1) //�˺�״̬���ݣ��˺���������
		{
			JS_send_HURT_control();
			send_to_C_JS_HURT = 0;
			
		}
		if (send_to_C_JS_SHOOT == 1) //ʵʱ������ݣ��ӵ��������
		{
			JS_send_SHOOT_control();
			// JS_SEND_times++;
			send_to_C_JS_SHOOT = 0;
		}
		if (send_to_C_JS_STATUS == 1) //����ϵͳ_״̬����_10Hz ���ڷ���
		{
			// send_to_C_STATUS_times++;
			JS_send_STATUS_control();
			send_to_C_JS_STATUS = 0;
		}
		if (send_to_C_JS_HEAT == 1)
		{
			JS_send_HEAT_control();
			send_to_C_JS_HEAT = 0;
		}
		if(send_to_C_JS_ROBOTHP == 1)
		{
			JS_send_robotHP_control();
			send_to_C_JS_ROBOTHP = 0;
		}
		if(send_to_C_JS_GAMESTATUS == 1)
		{
			JS_send_gamestatus_control();
			send_to_C_JS_GAMESTATUS = 0;
		}
		
		
		if (send_to_C_IN_END == 1)
		{
			PLACE_send_control();
			//	place_SEND_times++;
			send_to_C_IN_END = 0;
		}
		if(send_to_C_B0_FLAG == 1)
		{
			disturbB0_send_control();
			send_to_C_B0_FLAG = 0;
		}
		
		//������ֵ
		Encoder_Send_control();
		

		//		Update_Vision_SendData();
		vTaskDelay(1);
	}
	/* USER CODE END Vision_SendD */
}

/* USER CODE BEGIN Header_Robot_Detec */
/**
 * @brief Function implementing the Robot_Detec_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Robot_Detec */
void Robot_Detec(void const *argument)
{
	/* USER CODE BEGIN Robot_Detec */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //ÿ200����ǿ�ƽ����ܿ���,�������� ���ȼ� �� ��ⲻ�ɹ�Ƶ��
	/* Infinite loop */
	for (;;)
	{
		Offline_Clear_FPS();
		IMUFPS_0_Emergency_status();
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
	/* USER CODE END Robot_Detec */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Robot_Control(void const *argument)
{
	/* USER CODE BEGIN RobotControl */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿʮ����ǿ�ƽ����ܿ���
	/* Infinite loop */
	for (;;)
	{
		//���̿���ִ��
		Chassis_FUN.Chassis_Control();
		//��̨����ִ��
//		Cloud_FUN.Cloud_Control();
		//��������ִ��
//		Attack_FUN.Attack_Processing();

		// CAN�������ݸ����
		//��̨
//		GM6020_SetVoltage(Cloud.yaw_output, Cloud.pitch_output, 0, 0);
		//Ħ����
		// M3508s1_setCurrent(Shoot.Shoot1->OutputCurrent,Shoot.Shoot2->OutputCurrent,0,0);
		//����
		// M2006_setCurrent(0,0,M2006s[2].OutputCurrent,0);
		//����
		// M3508_setCurrent(0,0,0,Chassis.EM->OutputCurrent);
		//�������Ƕ���ͬ��CAN1���͵�
		M3508s1_setCurrent(Shoot.Shoot1->OutputCurrent, Shoot.Shoot2->OutputCurrent, send_to_break, Chassis.EM->OutputCurrent);
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}

	/* USER CODE END RobotControl */
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
