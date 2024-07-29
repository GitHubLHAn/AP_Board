/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
	typedef struct{
		volatile uint32_t start_time;
		volatile uint32_t current_time;
		volatile uint32_t interval;
	}timeInterval_t;


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
#define RESET_RX_Pin GPIO_PIN_1
#define RESET_RX_GPIO_Port GPIOA
#define RESET_TX_Pin GPIO_PIN_2
#define RESET_TX_GPIO_Port GPIOA
#define NSS_RX_Pin GPIO_PIN_3
#define NSS_RX_GPIO_Port GPIOA
#define NSS_TX_Pin GPIO_PIN_4
#define NSS_TX_GPIO_Port GPIOA
#define DIO0_IRQ_LORA_Pin GPIO_PIN_1
#define DIO0_IRQ_LORA_GPIO_Port GPIOB
#define DIO0_IRQ_LORA_EXTI_IRQn EXTI1_IRQn
#define SCS_ETH_Pin GPIO_PIN_12
#define SCS_ETH_GPIO_Port GPIOB
#define RESETETH_Pin GPIO_PIN_11
#define RESETETH_GPIO_Port GPIOA
#define LEDTB4_Pin GPIO_PIN_3
#define LEDTB4_GPIO_Port GPIOB
#define LEDTB3_Pin GPIO_PIN_4
#define LEDTB3_GPIO_Port GPIOB
#define LEDTB2_Pin GPIO_PIN_5
#define LEDTB2_GPIO_Port GPIOB
#define LEDTB_Pin GPIO_PIN_6
#define LEDTB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */