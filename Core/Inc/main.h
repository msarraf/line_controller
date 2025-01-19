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
#include "stm32f0xx_hal.h"

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
uint8_t check_value_counts(uint8_t *samples, uint8_t value);

void check_signal_length_up(uint8_t counts, uint32_t *length);
void check_signal_length_down(uint8_t counts, uint32_t *length);

void check_signal_length_left_key(uint8_t counts, uint32_t *length);
void check_signal_length_right_key(uint8_t counts, uint32_t *length);

void check_general_state(uint8_t counts);

void transfer_data(void);
void sampling_data(void);
void shift_and_insert_values(uint32_t *values, uint32_t new_value);
void check_up_periods(uint8_t counts, uint32_t *period);
void check_down_periods(uint8_t counts, uint32_t *period);
void check_down_up_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff);
void check_up_down_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_MOVE_KEY_Pin GPIO_PIN_1
#define LEFT_MOVE_KEY_GPIO_Port GPIOF
#define GENERAL_STATE_Pin GPIO_PIN_0
#define GENERAL_STATE_GPIO_Port GPIOA
#define MOTOR_RIGHT_Pin GPIO_PIN_1
#define MOTOR_RIGHT_GPIO_Port GPIOA
#define ON_BOARD_LED_Pin GPIO_PIN_4
#define ON_BOARD_LED_GPIO_Port GPIOA
#define DOWN_SENSOR_Pin GPIO_PIN_5
#define DOWN_SENSOR_GPIO_Port GPIOA
#define MOTOR_LEFT_Pin GPIO_PIN_6
#define MOTOR_LEFT_GPIO_Port GPIOA
#define SAMPLING_FREQ_Pin GPIO_PIN_7
#define SAMPLING_FREQ_GPIO_Port GPIOA
#define RIGHT_MOVE_KEY_Pin GPIO_PIN_1
#define RIGHT_MOVE_KEY_GPIO_Port GPIOB
#define BASE_TIMER_Pin GPIO_PIN_9
#define BASE_TIMER_GPIO_Port GPIOA
#define UP_SENSOR_Pin GPIO_PIN_10
#define UP_SENSOR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
