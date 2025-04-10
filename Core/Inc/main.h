/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOC
#define MOTOR2_DRC_Pin GPIO_PIN_3
#define MOTOR2_DRC_GPIO_Port GPIOC
#define MOTOR4_PWM_Pin GPIO_PIN_2
#define MOTOR4_PWM_GPIO_Port GPIOA
#define SBUZZ_Pin GPIO_PIN_3
#define SBUZZ_GPIO_Port GPIOA
#define IR_SD_Pin GPIO_PIN_4
#define IR_SD_GPIO_Port GPIOA
#define IR_BT_R_Pin GPIO_PIN_5
#define IR_BT_R_GPIO_Port GPIOA
#define IR_CC_R1_Pin GPIO_PIN_6
#define IR_CC_R1_GPIO_Port GPIOA
#define IRHIT_L_Pin GPIO_PIN_7
#define IRHIT_L_GPIO_Port GPIOA
#define SIGNAL1_Pin GPIO_PIN_4
#define SIGNAL1_GPIO_Port GPIOC
#define SIGNAL2_Pin GPIO_PIN_5
#define SIGNAL2_GPIO_Port GPIOC
#define IRHIT_R_Pin GPIO_PIN_0
#define IRHIT_R_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define IR_CC_L1_Pin GPIO_PIN_10
#define IR_CC_L1_GPIO_Port GPIOB
#define IR_BT_F_Pin GPIO_PIN_11
#define IR_BT_F_GPIO_Port GPIOB
#define IR_BT_L_Pin GPIO_PIN_12
#define IR_BT_L_GPIO_Port GPIOB
#define MOTOR1_DRC_Pin GPIO_PIN_15
#define MOTOR1_DRC_GPIO_Port GPIOB
#define MOTOR3_PWM_Pin GPIO_PIN_6
#define MOTOR3_PWM_GPIO_Port GPIOC
#define MOTOR1_PWM_Pin GPIO_PIN_7
#define MOTOR1_PWM_GPIO_Port GPIOC
#define PUMP_PWM_Pin GPIO_PIN_8
#define PUMP_PWM_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_9
#define FAN_PWM_GPIO_Port GPIOC
#define MOTOR1_ENCODERB_Pin GPIO_PIN_8
#define MOTOR1_ENCODERB_GPIO_Port GPIOA
#define MOTOR1_ENCODERA_Pin GPIO_PIN_9
#define MOTOR1_ENCODERA_GPIO_Port GPIOA
#define FAN_FG_Pin GPIO_PIN_12
#define FAN_FG_GPIO_Port GPIOA
#define WIFI_RX_Pin GPIO_PIN_10
#define WIFI_RX_GPIO_Port GPIOC
#define WIFI_TX_Pin GPIO_PIN_11
#define WIFI_TX_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOC
#define WIFI_STATE_Pin GPIO_PIN_4
#define WIFI_STATE_GPIO_Port GPIOB
#define WIFI_EN_Pin GPIO_PIN_5
#define WIFI_EN_GPIO_Port GPIOB
#define JY90_RX_Pin GPIO_PIN_6
#define JY90_RX_GPIO_Port GPIOB
#define JY90_TX_Pin GPIO_PIN_7
#define JY90_TX_GPIO_Port GPIOB
#define MOTOR2_PWM_Pin GPIO_PIN_8
#define MOTOR2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
