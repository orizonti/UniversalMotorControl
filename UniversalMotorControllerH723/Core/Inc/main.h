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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUE_LED_Pin GPIO_PIN_3
#define BLUE_LED_GPIO_Port GPIOE
#define WHITE_LED_Pin GPIO_PIN_4
#define WHITE_LED_GPIO_Port GPIOE
#define DIR_MOTOR2_Pin GPIO_PIN_5
#define DIR_MOTOR2_GPIO_Port GPIOE
#define ENABLE_MOTORS_Pin GPIO_PIN_6
#define ENABLE_MOTORS_GPIO_Port GPIOE
#define SW2_USER_Pin GPIO_PIN_13
#define SW2_USER_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_14
#define GREEN_LED_GPIO_Port GPIOC
#define DIR_MOTOR1_Pin GPIO_PIN_0
#define DIR_MOTOR1_GPIO_Port GPIOC
#define TIM3_CH1_MOTOR2_Pin GPIO_PIN_6
#define TIM3_CH1_MOTOR2_GPIO_Port GPIOA
#define SPI2_MISO_DAC_Pin GPIO_PIN_14
#define SPI2_MISO_DAC_GPIO_Port GPIOB
#define SPI2_MOSI_DAC_Pin GPIO_PIN_15
#define SPI2_MOSI_DAC_GPIO_Port GPIOB
#define SPI2_CS_DAC_Pin GPIO_PIN_11
#define SPI2_CS_DAC_GPIO_Port GPIOD
#define TIM4_CH1_FILTER_Pin GPIO_PIN_12
#define TIM4_CH1_FILTER_GPIO_Port GPIOD
#define TIM4_CH2_FILTER_Pin GPIO_PIN_13
#define TIM4_CH2_FILTER_GPIO_Port GPIOD
#define USART6_TX_RS485_Pin GPIO_PIN_6
#define USART6_TX_RS485_GPIO_Port GPIOC
#define USART6_RX_RS485_Pin GPIO_PIN_7
#define USART6_RX_RS485_GPIO_Port GPIOC
#define SPI2_SCK_DAC_Pin GPIO_PIN_9
#define SPI2_SCK_DAC_GPIO_Port GPIOA
#define SPI3_SCK_W5500_Pin GPIO_PIN_10
#define SPI3_SCK_W5500_GPIO_Port GPIOC
#define SPI3_MISO_W5500_Pin GPIO_PIN_11
#define SPI3_MISO_W5500_GPIO_Port GPIOC
#define SPI3_MOSI_W5500_Pin GPIO_PIN_12
#define SPI3_MOSI_W5500_GPIO_Port GPIOC
#define USART2_TX_EBT_Pin GPIO_PIN_5
#define USART2_TX_EBT_GPIO_Port GPIOD
#define USART2_RX_EBT_Pin GPIO_PIN_6
#define USART2_RX_EBT_GPIO_Port GPIOD
#define SPI3_CS_W5500_Pin GPIO_PIN_7
#define SPI3_CS_W5500_GPIO_Port GPIOD
#define W5500_Reset_Pin GPIO_PIN_4
#define W5500_Reset_GPIO_Port GPIOB
#define TIM3_CH2_MOTOR1_Pin GPIO_PIN_5
#define TIM3_CH2_MOTOR1_GPIO_Port GPIOB
#define EBT_Reset_Pin GPIO_PIN_6
#define EBT_Reset_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
