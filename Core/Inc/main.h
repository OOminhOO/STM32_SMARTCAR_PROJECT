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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOA
#define LCD_RES_Pin GPIO_PIN_1
#define LCD_RES_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ECHO_LEFT_Pin GPIO_PIN_4
#define ECHO_LEFT_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOA
#define ECHO_RIGHT_Pin GPIO_PIN_0
#define ECHO_RIGHT_GPIO_Port GPIOB
#define LBB_Pin GPIO_PIN_10
#define LBB_GPIO_Port GPIOB
#define TRIG_RIGHT_Pin GPIO_PIN_7
#define TRIG_RIGHT_GPIO_Port GPIOC
#define TRIG_LEFT_Pin GPIO_PIN_8
#define TRIG_LEFT_GPIO_Port GPIOA
#define LBF_Pin GPIO_PIN_9
#define LBF_GPIO_Port GPIOA
#define LFB_Pin GPIO_PIN_10
#define LFB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LFF_Pin GPIO_PIN_3
#define LFF_GPIO_Port GPIOB
#define RFB_Pin GPIO_PIN_4
#define RFB_GPIO_Port GPIOB
#define RFF_Pin GPIO_PIN_5
#define RFF_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB
#define DHT11_Pin GPIO_PIN_7
#define DHT11_GPIO_Port GPIOB
#define RBF_Pin GPIO_PIN_8
#define RBF_GPIO_Port GPIOB
#define RBB_Pin GPIO_PIN_9
#define RBB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
