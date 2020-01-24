/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOB
#define AK4452_DZF_Pin GPIO_PIN_5
#define AK4452_DZF_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define ENET_RST_N_Pin GPIO_PIN_11
#define ENET_RST_N_GPIO_Port GPIOA
#define ADS127_START_Pin GPIO_PIN_4
#define ADS127_START_GPIO_Port GPIOD
#define ADS127_RESET_PWDN_N_Pin GPIO_PIN_3
#define ADS127_RESET_PWDN_N_GPIO_Port GPIOD
#define uSD_DETECT_Pin GPIO_PIN_10
#define uSD_DETECT_GPIO_Port GPIOA
#define PTT_GPIO_Pin GPIO_PIN_11
#define PTT_GPIO_GPIO_Port GPIOI
#define TPA_RESET_N_Pin GPIO_PIN_13
#define TPA_RESET_N_GPIO_Port GPIOD
#define TPA_OTW_CLIP_N_Pin GPIO_PIN_12
#define TPA_OTW_CLIP_N_GPIO_Port GPIOD
#define TPA_FAULT_N_Pin GPIO_PIN_11
#define TPA_FAULT_N_GPIO_Port GPIOD
#define V_BATT_SW_EN_Pin GPIO_PIN_12
#define V_BATT_SW_EN_GPIO_Port GPIOB
#define BOOST_EN_Pin GPIO_PIN_13
#define BOOST_EN_GPIO_Port GPIOB
#define AK4452_PDN_Pin GPIO_PIN_1
#define AK4452_PDN_GPIO_Port GPIOB
#define AK4954_PDN_Pin GPIO_PIN_0
#define AK4954_PDN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
