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
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SAMPLING_ARRAY_LENGTH 2
#define MEAN_ARRAY_LENGTH 4
#define NO_SIGNAL_WATCH_TIME 40000 // 50 us sampling timer * SAMPLING_ARRAY_LENGTH = 10 sample per each ms, 4 s wait time = 40000
#define DEBOUNCE 200
#define ACCEPTABLE_ERROR_PERCENTAGE 50
#define COMAN_LENGHT_MIN 5000
#define COMAND_WAITING 5000

typedef enum {
    ERROR_NONE = 0,
    ERROR_UP_SENSOR,
    ERROR_DOWN_SENSOR,
} ErrorCode_t;

typedef struct {
    uint32_t error;
    bool sign; //down up difference is the smaller value
} Error;

void ErrorHandler(ErrorCode_t error);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

bool timer_1_flag = false;
bool timer_3_flag = false;

bool auto_flag = false;

ErrorCode_t error = ERROR_NONE;

bool down_preiod_posetive_flag = false;
bool up_preiod_posetive_flag = false;

bool left_key_length_posetive_flag = false;
bool right_key_length_posetive_flag = false;

bool down_lenght_posetive_flag = false;
bool up_lenght_posetive_flag = false;


bool ud_down_posetive_flag = false;
bool ud_up_posetive_flag = false;
bool du_down_posetive_flag = false;
bool du_up_posetive_flag = false;

bool left_move_key_effect_a_flag = false;
bool right_move_key_effect_a_flag = false;

bool control_command = false;
bool control_command_start = false;
bool control_command_countinue = false;
bool command_sign = false;
uint32_t command_lenght_right = 0;
uint32_t command_lenght_left = 0;
uint32_t command_lenght = 0;
uint32_t command_counter = 0;
uint32_t command_waiting_counter = 0;

uint8_t general_state_samples[SAMPLING_ARRAY_LENGTH] = {0};
uint8_t down_sensor_samples[SAMPLING_ARRAY_LENGTH] = {0};
uint8_t up_sensor_samples[SAMPLING_ARRAY_LENGTH] = {0};
uint8_t left_move_key_samples[SAMPLING_ARRAY_LENGTH] = {0};
uint8_t right_move_key_samples[SAMPLING_ARRAY_LENGTH] = {0};

uint8_t sampleIndex = 0;
uint8_t left_move_key_effect_a_counter = 0;
uint8_t right_move_key_effect_a_counter = 0;

uint32_t down_sensor_lenght_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t up_sensor_lenght_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t left_move_key_length_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t right_move_key_length_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t down_up_diff_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t up_down_diff_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t down_period_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t up_period_values[MEAN_ARRAY_LENGTH] = {0};

uint32_t left_move_key_effect_error_a_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t left_move_key_effect_error_b_values[MEAN_ARRAY_LENGTH] = {0};

uint32_t right_move_key_effect_error_a_values[MEAN_ARRAY_LENGTH] = {0};
uint32_t right_move_key_effect_error_b_values[MEAN_ARRAY_LENGTH] = {0};

uint8_t valueIndex = 0;

uint32_t down_sensor_lenght = 0;
uint32_t up_sensor_lenght = 0;

uint32_t left_key_lenght = 0;
uint32_t right_key_lenght = 0;

uint32_t down_sensor_period = 0;
uint32_t up_sensor_period = 0;

uint32_t down_up_diff = 0;
uint32_t up_down_diff = 0;

uint32_t up_length_watch_time = 0;
uint32_t down_length_watch_time = 0;

uint32_t left_move_key_debounce = 0;
uint32_t right_move_key_debounce = 0;
uint32_t general_state_debounce = 0;

Error left_key_error_b;
Error right_key_error_b;

Error manual_max_error;

uint32_t manual_priod = 0;
uint32_t manual_right_key_lenght = 0;
uint32_t manual_left_key_lenght = 0;
uint32_t manual_right_key_a = 0;
uint32_t manual_left_key_a = 0;

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
Error calculate_error(uint32_t *values_1, uint32_t *values_2);
Error calculate_error_partialy(uint32_t *values_1, uint32_t *values_2, uint8_t counter);
uint32_t check_b_a_error(Error error_a, Error error_b);
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

	char buffer[150];
	memset(buffer, 0, sizeof(buffer));
	char num_str1[10], num_str2[10], num_str3[10], num_str4[10], num_str5[10], num_str6[10];


	itoa(auto_flag, num_str1, 10);  // Convert num1 to string
	itoa(up_period_values[MEAN_ARRAY_LENGTH -1], num_str2, 10);  // Convert num2 to string
	itoa(down_period_values[MEAN_ARRAY_LENGTH -1], num_str3, 10);
	itoa(up_down_diff_values[MEAN_ARRAY_LENGTH -1], num_str4,  10);
	itoa(down_up_diff_values[MEAN_ARRAY_LENGTH -1], num_str5, 10);
	itoa(error, num_str6, 10);

	strcpy(buffer, "SF: ");
	strcat(buffer, num_str1);
	strcat(buffer, ", UP: ");
	strcat(buffer, num_str2);
	strcat(buffer, ", DP: ");
	strcat(buffer, num_str3);
	strcat(buffer, "\r\n");

	strcat(buffer, "UPD: ");
	strcat(buffer, num_str4);
	strcat(buffer, ", DUP: ");
	strcat(buffer, num_str5);
	strcat(buffer, "\r\n");
	strcat(buffer, "E: ");
	strcat(buffer, num_str6);
	strcat(buffer, "\r\n");
	strcat(buffer, "\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
	timer_1_flag = false;
}

void sampling_data(void)
{
	general_state_samples[sampleIndex] = HAL_GPIO_ReadPin(GENERAL_STATE_GPIO_Port, GENERAL_STATE_Pin);
	down_sensor_samples[sampleIndex] = HAL_GPIO_ReadPin(DOWN_SENSOR_GPIO_Port, DOWN_SENSOR_Pin);
	up_sensor_samples[sampleIndex] = HAL_GPIO_ReadPin(UP_SENSOR_GPIO_Port, UP_SENSOR_Pin);
	left_move_key_samples[sampleIndex]= HAL_GPIO_ReadPin(LEFT_MOVE_KEY_GPIO_Port, LEFT_MOVE_KEY_Pin);
	right_move_key_samples[sampleIndex]= HAL_GPIO_ReadPin(RIGHT_MOVE_KEY_GPIO_Port, RIGHT_MOVE_KEY_Pin);
	sampleIndex++;

	if (sampleIndex >= SAMPLING_ARRAY_LENGTH)
		{
			uint8_t general_state_one_counts = check_value_counts(general_state_samples, 1);
			uint8_t down_sensor_one_counts = check_value_counts(down_sensor_samples, 1);
			uint8_t up_sensor_one_counts = check_value_counts(up_sensor_samples, 1);
			uint8_t left_key_one_counts = check_value_counts(left_move_key_samples, 1);
			uint8_t right_key_one_counts = check_value_counts(right_move_key_samples, 1);

			check_general_state(general_state_one_counts);
			check_signal_length_down(down_sensor_one_counts, &down_sensor_lenght);
			check_signal_length_up(up_sensor_one_counts, &up_sensor_lenght);

			if (auto_flag == false)
			{
				check_signal_length_left_key(left_key_one_counts, &left_key_lenght);
				check_signal_length_right_key(right_key_one_counts, &right_key_lenght);

				if (left_move_key_effect_a_flag)
					{
						left_move_key_effect_a_counter++;
						if (left_move_key_effect_a_counter >= MEAN_ARRAY_LENGTH)
						{
							left_move_key_effect_a_flag = false;
							Error error = calculate_error(down_up_diff_values, up_down_diff_values);
							uint32_t b_a_error = check_b_a_error(error, left_key_error_b);
							shift_and_insert_values(left_move_key_effect_error_a_values ,b_a_error);
							left_move_key_effect_a_counter = 0;
						}
					}

				if (right_move_key_effect_a_flag)
					{
						right_move_key_effect_a_counter++;
						if (right_move_key_effect_a_counter >= MEAN_ARRAY_LENGTH)
						{
							right_move_key_effect_a_flag = false;
							Error error = calculate_error(down_up_diff_values, up_down_diff_values);
							uint32_t b_a_error = check_b_a_error(error, right_key_error_b);
							shift_and_insert_values(right_move_key_effect_error_a_values ,b_a_error);
							right_move_key_effect_a_counter = 0;
						}
					}
			}

			check_down_periods(down_sensor_one_counts, &down_sensor_period);
			check_up_periods(up_sensor_one_counts, &up_sensor_period);

			check_down_up_diff(down_sensor_one_counts, up_sensor_one_counts, &down_up_diff);
			check_up_down_diff(down_sensor_one_counts, up_sensor_one_counts, &up_down_diff);

			if (auto_flag)
			{
				if ((control_command_countinue) && (control_command)) command_counter++;
				if ((control_command) && (control_command_start == false) && (control_command_countinue == false)) command_waiting_counter++;
			}

			sampleIndex = 0;

	}
		timer_3_flag = false;

}

void check_down_up_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff)
{
	if (down_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(du_down_posetive_flag == false)
		{
			*diff = 0;
			du_down_posetive_flag = true;
		}

	}
	else
	{
		du_down_posetive_flag = false;
	}
	if (up_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if (du_up_posetive_flag == false)
		{
			shift_and_insert_values(down_up_diff_values ,*diff);
			du_up_posetive_flag = true;
		}

	}
	else
	{
		du_up_posetive_flag = false;
	}

	*diff = *diff + 1;

}

void check_up_down_diff(uint8_t down_counts, uint8_t up_counts, uint32_t *diff)
{
	if (up_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if(ud_up_posetive_flag == false)
		{
			*diff = 0;
			ud_up_posetive_flag = true;
		}

	}
	else
	{
		ud_up_posetive_flag = false;
	}
	if (down_counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if (ud_down_posetive_flag == false)
		{
			shift_and_insert_values(up_down_diff_values ,*diff);
			ud_down_posetive_flag = true;
		}

	}
	else
	{
		ud_down_posetive_flag = false;
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
			if (error == ERROR_UP_SENSOR) error = ERROR_NONE;
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
				error = ERROR_UP_SENSOR;
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
			if (error == ERROR_DOWN_SENSOR) error = ERROR_NONE;
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
				error = ERROR_DOWN_SENSOR;
				down_length_watch_time = 0;
			}
		}
	}

}

void check_signal_length_left_key(uint8_t counts, uint32_t *length)
{

	if ((counts > (SAMPLING_ARRAY_LENGTH / 2)) && (left_move_key_debounce >= DEBOUNCE))
	{
		*length = *length + 1;
		if (left_key_length_posetive_flag == false)
		{
			left_key_error_b = calculate_error(down_up_diff_values, up_down_diff_values);
			shift_and_insert_values(left_move_key_effect_error_b_values ,left_key_error_b.error);
			left_key_length_posetive_flag = true;
			if ((left_move_key_effect_a_flag) && (left_move_key_effect_a_counter < MEAN_ARRAY_LENGTH))
			{
				Error error = calculate_error_partialy(down_up_diff_values, up_down_diff_values, left_move_key_effect_a_counter);
				uint32_t b_a_error = check_b_a_error(error, left_key_error_b);
				shift_and_insert_values(left_move_key_effect_error_a_values ,b_a_error);
			}
			left_move_key_effect_a_flag = false;
			left_move_key_effect_a_counter = 0;
			left_move_key_debounce = 0;
		}
	}
	else
	{
		if ((left_key_length_posetive_flag) && (*length > DEBOUNCE))
		{
			shift_and_insert_values(left_move_key_length_values ,*length);
			*length = 0;
			left_key_length_posetive_flag = false;
			left_move_key_effect_a_flag = true;
		}
		left_move_key_debounce++;
		if (left_move_key_debounce > 2*DEBOUNCE) left_move_key_debounce = 2*DEBOUNCE;
	}


}

void check_signal_length_right_key(uint8_t counts, uint32_t *length)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		*length = *length + 1;
		if ((right_key_length_posetive_flag == false) && (right_move_key_debounce >= DEBOUNCE))
		{
			right_key_error_b = calculate_error(down_up_diff_values, up_down_diff_values);
			shift_and_insert_values(right_move_key_effect_error_b_values ,right_key_error_b.error);
			right_key_length_posetive_flag = true;
			if ((right_move_key_effect_a_flag) && (right_move_key_effect_a_counter < MEAN_ARRAY_LENGTH))
			{
				Error error = calculate_error_partialy(down_up_diff_values, up_down_diff_values, right_move_key_effect_a_counter);
				uint32_t b_a_error = check_b_a_error(error, right_key_error_b);
				shift_and_insert_values(right_move_key_effect_error_a_values ,b_a_error);
			}
			right_move_key_effect_a_flag = false;
			right_move_key_effect_a_counter = 0;
			right_move_key_debounce = 0;
		}
	}
	else
	{
		if ((right_key_length_posetive_flag) && (*length > DEBOUNCE))
		{
			shift_and_insert_values(right_move_key_length_values ,*length);
			*length = 0;
			right_key_length_posetive_flag = false;
			right_move_key_effect_a_flag = true;
		}
		right_move_key_debounce++;
		if (right_move_key_debounce > 2*DEBOUNCE) right_move_key_debounce = 2*DEBOUNCE;
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

Error calculate_error(uint32_t *values_1, uint32_t *values_2)
{
	Error error;
	uint32_t value_1_mean = calculate_mean(values_1);
	uint32_t value_2_mean = calculate_mean(values_2);

	if (value_1_mean < value_2_mean)
		{
			error.error = value_1_mean;
			error.sign = true; // down up difference is the smaller value
		}
	else
		{
			error.error = value_2_mean;
			error.sign = false;
		}
	return error;
}

uint32_t calculate_mean(uint32_t *values)
{
    uint32_t sum = 0;
    uint8_t num = 0;
    for (uint8_t i = 0; i < MEAN_ARRAY_LENGTH; i++) {
        if (values[i] > 0)
        {
        	sum += values[i];
        	num +=1;
        }
    }
    if (num > 0)
    {
    	return sum / num;
    }
    else
    {
    	return 0;
    }

}

Error calculate_error_partialy(uint32_t *values_1, uint32_t *values_2, uint8_t counter)
{
	Error error;
	uint32_t value_1_mean = calculate_mean_partialy(values_1, counter);
	uint32_t value_2_mean = calculate_mean_partialy(values_2, counter);

	if (value_1_mean < value_2_mean)
		{
			error.error = value_1_mean;
			error.sign = true;
		}
	else
		{
			error.error = value_2_mean;
			error.sign = false;
		}
	return error;
}

uint32_t calculate_mean_partialy(uint32_t *values, uint8_t counter)
{
    uint32_t sum = 0;
    uint8_t num = 0;
    for (uint8_t i = MEAN_ARRAY_LENGTH -1; i >= (MEAN_ARRAY_LENGTH - counter); i--) {
        if (values[i] > 0)
        {
        	sum += values[i];
        	num +=1;
        }
    }

    if (num > 0)
    {
    	return sum / num;
    }
    else
    {
    	return 0;
    }

}

uint32_t check_b_a_error(Error error_a, Error error_b)
{
	if (error_b.sign)
	{
		if (error_a.sign)
		{
			if (error_a.error > error_b.error) return error_a.error - error_b.error;
			else return error_b.error - error_a.error;
		}
		else return error_b.error + error_a.error;

	}
	else
	{
			if (error_a.sign) return error_b.error + error_a.error;
			else
			{
				if (error_a.error > error_b.error) return error_a.error - error_b.error;
				else return error_b.error - error_a.error;

			}

	}

}

void check_general_state(uint8_t counts)
{
	if (counts > (SAMPLING_ARRAY_LENGTH / 2))
	{
		if ((auto_flag == false) && (general_state_debounce > DEBOUNCE))
		{
			auto_flag = true;
			manual_max_error = calculate_error(down_up_diff_values, up_down_diff_values);
			manual_priod = calculate_mean(down_period_values);
			manual_right_key_lenght = calculate_mean(right_move_key_length_values);
			manual_left_key_lenght = calculate_mean(left_move_key_length_values);
			manual_right_key_a = calculate_mean(right_move_key_effect_error_a_values);
			manual_left_key_a = calculate_mean(left_move_key_effect_error_a_values);
			general_state_debounce = 0;

		}
		general_state_debounce++;
		if (general_state_debounce > 2*DEBOUNCE) general_state_debounce = 2*DEBOUNCE;
	}
	else
	{
		if((auto_flag) && (general_state_debounce > DEBOUNCE))
		{
			general_state_debounce = 0;
			auto_flag = false;
			memset(right_move_key_length_values, 0, sizeof(right_move_key_length_values));
			memset(left_move_key_length_values, 0, sizeof(left_move_key_length_values));
			memset(right_move_key_effect_error_a_values, 0, sizeof(right_move_key_effect_error_a_values));
			memset(left_move_key_effect_error_a_values, 0, sizeof(left_move_key_effect_error_a_values));
		}
		general_state_debounce++;
		if (general_state_debounce > 2*DEBOUNCE) general_state_debounce = 2*DEBOUNCE;
	}

}

void motor_control(void)
{
	if (control_command)
	{
		if(control_command_start)
		{
			// calculate the length of the command signal
			if (command_sign)
			{
				HAL_GPIO_WritePin(MOTOR_RIGHT_GPIO_Port, MOTOR_RIGHT_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_LEFT_GPIO_Port, MOTOR_LEFT_Pin, GPIO_PIN_RESET);
				if (command_lenght_right > COMAN_LENGHT_MIN) command_lenght = command_lenght_right;
				else command_lenght = COMAN_LENGHT_MIN;
			}
			else
			{
				HAL_GPIO_WritePin(MOTOR_LEFT_GPIO_Port, MOTOR_LEFT_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR_RIGHT_GPIO_Port, MOTOR_RIGHT_Pin, GPIO_PIN_RESET);
				if (command_lenght_left > COMAN_LENGHT_MIN) command_lenght = command_lenght_left;
				else command_lenght = COMAN_LENGHT_MIN;
			}
			control_command_countinue = true;
			control_command_start = false;
			command_counter = 0;
		}
		else if (control_command_countinue)
		{
			if (command_counter >= command_lenght)
			{
				HAL_GPIO_WritePin(MOTOR_LEFT_GPIO_Port, MOTOR_LEFT_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR_RIGHT_GPIO_Port, MOTOR_RIGHT_Pin, GPIO_PIN_RESET);
				control_command_countinue = false;
				command_waiting_counter = 0;
			}

		}
		else
		{
			if (command_waiting_counter >= COMAND_WAITING) control_command = false;
		}

	}
	else
	{
		Error error = calculate_error(down_up_diff_values, up_down_diff_values);
		uint32_t period = calculate_mean(down_period_values);


		if ((period != 0)&&(manual_priod != 0))
		{
			float acceptable_error = ((float)manual_max_error.error / manual_priod)* ((float)ACCEPTABLE_ERROR_PERCENTAGE / 100)  * period;

			if (error.error > acceptable_error)
			{

				float command_lenght_l = (float)(error.error * manual_left_key_lenght) / manual_left_key_a;
				float command_lenght_r = (float)(error.error * manual_right_key_lenght) / manual_right_key_a;

				command_lenght_left = (int)(command_lenght_l * period / manual_priod + 0.5);
				command_lenght_right = (int)(command_lenght_r * period /manual_priod + 0.5);

				command_sign = error.sign;
				control_command = true;
				control_command_start = true;
			}
		}

	}

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

	if ((auto_flag) && (error == ERROR_NONE))
	{
		motor_control();
	}
	else
	{
		HAL_GPIO_WritePin(MOTOR_RIGHT_GPIO_Port, MOTOR_RIGHT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_LEFT_GPIO_Port, MOTOR_LEFT_Pin, GPIO_PIN_RESET);
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
  htim1.Init.Period = 1000-1;
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
  sConfigOC.Pulse = 1000-1;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_RIGHT_Pin|ON_BOARD_LED_Pin|MOTOR_LEFT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEFT_MOVE_KEY_Pin */
  GPIO_InitStruct.Pin = LEFT_MOVE_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LEFT_MOVE_KEY_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : RIGHT_MOVE_KEY_Pin */
  GPIO_InitStruct.Pin = RIGHT_MOVE_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RIGHT_MOVE_KEY_GPIO_Port, &GPIO_InitStruct);

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
