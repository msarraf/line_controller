/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SAMPLING_ARRAY_LENGTH 2
#define MEAN_ARRAY_LENGTH 4
#define NO_SIGNAL_WATCH_TIME 10000 // 50 us sampling timer * SAMPLING_ARRAY_LENGTH = 10 sample per each ms, 1 s wait time = 10000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

bool timer_1_flag = false;
bool timer_3_flag = false;

bool auto_flag = false;
bool down_preiod_posetive_flag = false;
bool up_preiod_posetive_flag = false;

bool down_lenght_posetive_flag = false;
bool up_lenght_posetive_flag = false;
bool general_state_posetive_flag = false;

bool down_diff_posetive_flag = false;
bool up_diff_posetive_flag = false;

uint8_t general_state_samples[SAMPLING_ARRAY_LENGTH];
uint8_t down_sensor_samples[SAMPLING_ARRAY_LENGTH];
uint8_t up_sensor_samples[SAMPLING_ARRAY_LENGTH];
uint8_t sampleIndex = 0;

uint32_t down_sensor_lenght_values[MEAN_ARRAY_LENGTH];
uint32_t up_sensor_lenght_values[MEAN_ARRAY_LENGTH];

uint32_t down_up_diff_values[MEAN_ARRAY_LENGTH];
uint32_t up_down_diff_values[MEAN_ARRAY_LENGTH];
uint32_t down_period_values[MEAN_ARRAY_LENGTH];
uint32_t up_period_values[MEAN_ARRAY_LENGTH];
uint8_t valueIndex = 0;

uint32_t down_sensor_lenght = 0;
uint32_t up_sensor_lenght = 0;

uint32_t down_sensor_period = 0;
uint32_t up_sensor_period = 0;
uint32_t down_up_diff = 0;
uint32_t up_down_diff = 0;

uint32_t up_length_watch_time = 0;
uint32_t down_length_watch_time = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM1)
    {
    	timer_1_flag = true;

    }
    if (htim->Instance == TIM3)
    {
    	timer_3_flag = true;
    }

}

void transfer_data(void)
{
	HAL_GPIO_TogglePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin);
	char message[100];
	snprintf(message, sizeof(message),
			 "sF: %d, uL: %lu, dL: %lu \r\nuP: %lu, dP: %lu \r\nudD: %lu, duD: %lu \r\n",
			  auto_flag,
			  up_sensor_lenght_values[MEAN_ARRAY_LENGTH -1],
			  down_sensor_lenght_values[MEAN_ARRAY_LENGTH -1],
			  up_period_values[MEAN_ARRAY_LENGTH -1],
			  down_period_values[MEAN_ARRAY_LENGTH -1],
			  up_down_diff_values[MEAN_ARRAY_LENGTH -1],
			  down_up_diff_values[MEAN_ARRAY_LENGTH -1]);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	timer_1_flag = false;

}

void sampling_data(void)
{
	general_state_samples[sampleIndex] = HAL_GPIO_ReadPin(GENERAL_STATE_GPIO_Port, GENERAL_STATE_Pin);
	down_sensor_samples[sampleIndex] = HAL_GPIO_ReadPin(DOWN_SENSOR_GPIO_Port, DOWN_SENSOR_Pin);
	up_sensor_samples[sampleIndex] = HAL_GPIO_ReadPin(UP_SENSOR_GPIO_Port, UP_SENSOR_Pin);
	sampleIndex++;

	if (sampleIndex >= SAMPLING_ARRAY_LENGTH)
		{
			uint8_t general_state_one_counts = check_value_counts(general_state_samples, 1);
			uint8_t down_sensor_one_counts = check_value_counts(down_sensor_samples, 1);
			uint8_t up_sensor_one_counts = check_value_counts(up_sensor_samples, 1);

			check_general_state(general_state_one_counts);
			check_signal_length_down(down_sensor_one_counts, &down_sensor_lenght);
			check_signal_length_up(up_sensor_one_counts, &up_sensor_lenght);

			check_down_periods(down_sensor_one_counts, &down_sensor_period);
			check_up_periods(up_sensor_one_counts, &up_sensor_period);

			check_down_up_diff(down_sensor_one_counts, up_sensor_one_counts, &down_up_diff);
			check_up_down_diff(down_sensor_one_counts, up_sensor_one_counts, &up_down_diff);
			sampleIndex = 0;

	}
		timer_3_flag = false;

}

void check_down_up_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff)
{
	if (down_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(down_diff_posetive_flag == false)
		{
			*diff = 0;
			down_diff_posetive_flag = true;
		}

	}
	if (up_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if (down_diff_posetive_flag == true)
		{
			shift_and_insert_values(down_up_diff_values ,*diff);
			down_diff_posetive_flag = false;
		}

	}

	*diff = *diff + 1;


}

void check_up_down_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff)
{
	if (up_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(up_diff_posetive_flag == false)
		{
			*diff = 0;
			up_diff_posetive_flag = true;
		}

	}
	if (down_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if (up_diff_posetive_flag == true)
		{
			shift_and_insert_values(up_down_diff_values ,*diff);
			up_diff_posetive_flag = false;
		}

	}

	*diff = *diff + 1;


}

void check_down_periods(uint8_t counts, uint32_t *period)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(down_preiod_posetive_flag == false)
		{
			// save period
			shift_and_insert_values(down_period_values ,*period);
			*period = 0;
			down_preiod_posetive_flag = true;
		}

	}
	else
	{
		if (down_preiod_posetive_flag)
		{
			down_preiod_posetive_flag = false;
		}

	}

	*period = *period + 1;

}

void check_up_periods(uint8_t counts, uint32_t *period)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(up_preiod_posetive_flag == false)
		{
			// save period
			shift_and_insert_values(up_period_values ,*period);
			*period = 0;
			up_preiod_posetive_flag = true;
		}

	}
	else
	{
		if (up_preiod_posetive_flag)
		{
			up_preiod_posetive_flag = false;
		}

	}

	*period = *period + 1;

}

void shift_and_insert_values(uint32_t *values, uint32_t new_value) {

    for (int i = 0; i < MEAN_ARRAY_LENGTH - 1; i++) {
    	values[i] = values[i + 1];
    }

    values[MEAN_ARRAY_LENGTH - 1] = new_value;
}

void check_signal_length_up(uint8_t counts, uint32_t *length)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		*length = *length + 1;
		if (up_lenght_posetive_flag == false)
		{
			up_lenght_posetive_flag = true;
			up_length_watch_time = 0;
		}
	}
	else
	{
		if (up_lenght_posetive_flag)
		{
			shift_and_insert_values(up_sensor_lenght_values ,*length);
			*length = 0;
			up_lenght_posetive_flag = false;
		}
		else
		{
			up_length_watch_time++;
			if (up_length_watch_time >= NO_SIGNAL_WATCH_TIME)
			{
				shift_and_insert_values(up_sensor_lenght_values ,0);
				up_length_watch_time = 0;
			}

		}
	}

}

void check_signal_length_down(uint8_t counts, uint32_t *length)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		*length = *length + 1;
		if (down_lenght_posetive_flag == false)
		{
			down_lenght_posetive_flag = true;
			down_length_watch_time = 0;
		}
	}
	else
	{
		if (down_lenght_posetive_flag)
		{
			shift_and_insert_values(down_sensor_lenght_values ,*length);
			*length = 0;
			down_lenght_posetive_flag = false;
		}
		else
		{
			down_length_watch_time++;
			if (down_length_watch_time >= NO_SIGNAL_WATCH_TIME)
			{
				shift_and_insert_values(down_sensor_lenght_values ,0);
				down_length_watch_time = 0;
			}
		}
	}

}

void check_general_state(uint8_t counts)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		auto_flag = true;
	}
	else
	{
		auto_flag = false;
	}

}

uint8_t check_value_counts(uint8_t *samples, uint8_t value)
{
	uint8_t counts = 0;
	for (uint8_t i = 0; i< SAMPLING_ARRAY_LENGTH; i++)
	{
		if (samples[i] == value)
		{
			counts++;
		}
	}
	return counts;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (timer_1_flag)
	{
		transfer_data();
	}

	if (timer_3_flag)
	{
        sampling_data();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 49;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_RIGHT_Pin|ON_BOARD_LED_Pin|MOTOR_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GENERAL_STATE_Pin DOWN_SENSOR_Pin UP_SENSOR_Pin */
  GPIO_InitStruct.Pin = GENERAL_STATE_Pin|DOWN_SENSOR_Pin|UP_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_RIGHT_Pin ON_BOARD_LED_Pin MOTOR_LEFT_Pin */
  GPIO_InitStruct.Pin = MOTOR_RIGHT_Pin|ON_BOARD_LED_Pin|MOTOR_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
