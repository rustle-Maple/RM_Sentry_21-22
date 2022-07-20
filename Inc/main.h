/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vision_TX_Pin GPIO_PIN_1
#define Vision_TX_GPIO_Port GPIOE
#define Vision_RX_Pin GPIO_PIN_0
#define Vision_RX_GPIO_Port GPIOE
#define DR16_RX_Pin GPIO_PIN_7
#define DR16_RX_GPIO_Port GPIOB
#define DR16_TX_Pin GPIO_PIN_6
#define DR16_TX_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define Limit_switch_2_Pin GPIO_PIN_11
#define Limit_switch_2_GPIO_Port GPIOA
#define Limit_switch_1_Pin GPIO_PIN_10
#define Limit_switch_1_GPIO_Port GPIOA
#define PEswitch_2_Pin GPIO_PIN_9
#define PEswitch_2_GPIO_Port GPIOA
#define PEswitch_1_Pin GPIO_PIN_8
#define PEswitch_1_GPIO_Port GPIOA
#define Debug_RX_Pin GPIO_PIN_7
#define Debug_RX_GPIO_Port GPIOC
#define Debug_TX_Pin GPIO_PIN_6
#define Debug_TX_GPIO_Port GPIOC
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Limit_switch_3_Pin GPIO_PIN_2
#define Limit_switch_3_GPIO_Port GPIOA
#define Sensor_tx_Pin GPIO_PIN_8
#define Sensor_tx_GPIO_Port GPIOE
#define CAN2_RX_Pin GPIO_PIN_12
#define CAN2_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_13
#define CAN2_TX_GPIO_Port GPIOB
#define Judge_RX_Pin GPIO_PIN_9
#define Judge_RX_GPIO_Port GPIOD
#define Judge_TX_Pin GPIO_PIN_8
#define Judge_TX_GPIO_Port GPIOD
#define Limit_switch_4_Pin GPIO_PIN_3
#define Limit_switch_4_GPIO_Port GPIOA
#define Sensor_rx_Pin GPIO_PIN_7
#define Sensor_rx_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
