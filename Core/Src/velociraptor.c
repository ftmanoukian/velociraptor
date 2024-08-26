/*
 * velociraptor.c
 *
 *  Created on: Aug 19, 2024
 *      Author: franciscomanoukian
 */
#include "main.h"
#include "velociraptor.h"

extern TIM_HandleTypeDef htim2,htim3,htim4;
extern ADC_HandleTypeDef hadc2;

struct
{
	uint32_t tick_cntr;
	uint16_t adc_read[2];
	uint16_t step_ticks[2];
	uint32_t prev_ticks[2];
	uint8_t prev_state[2];
	uint16_t upp_thr[2];
	uint16_t lwr_thr[2];
} encoders_data;

struct
{
	float base_speed;
	float max_speed;
	float e_prev;
	float e_int;
	float e_dv;
	float e_norm;
	float e_pid;
	float kp, ki, kd;
	float vel_l, vel_r;
} pid_data;

typedef enum
{
	STOPPED,
	RUNNING
} velo_mde_state_t;

velo_mde_state_t velo_mde_state = STOPPED;
uint8_t new_data_flag = 0;

uint32_t prev_dwt_tick = 0;
uint32_t cycle_dwt_ticks = 0;
uint32_t isr_dwt_ticks = 0;

void velociraptor_start(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_Base_Start_IT(&htim2);

	for(uint8_t i = 0; i < 2; i++)
	{
		encoders_data.step_ticks[i] = 0;
		encoders_data.prev_ticks[i] = 0;
		encoders_data.prev_state[i] = 0;
	}

	encoders_data.upp_thr[0] = 2500;
	encoders_data.lwr_thr[0] = 1500;

	encoders_data.upp_thr[1] = 1000;
	encoders_data.lwr_thr[1] = 750;

	encoders_data.tick_cntr = 0;

	pid_data.max_speed = 0.8f;
	pid_data.base_speed = 0;

	pid_data.e_prev = 0;
	pid_data.e_int = 0;
	pid_data.e_dv = 0;
	pid_data.e_norm = 0;
	pid_data.e_pid = 0;

	pid_data.kp = 1;
	pid_data.ki = 0.0;
	pid_data.kd = 0.0;

	pid_data.vel_l = 0;
	pid_data.vel_r = 0;

	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *) encoders_data.adc_read, 2);
	HAL_TIM_Base_Start(&htim3);

	CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT -> CYCCNT = 0;
	DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void velociraptor_setmotorspeed(uint8_t n_motor, float speed)
{
	if (speed > 1.0f) speed = 1.0f;
	if (speed < -1.0f) speed = -1.0f;

	float lower_lim = (float) MIN_LOADED_SPEED / (float) MAXSPEED;

	if (speed < 0.0f)
	{
		speed *= -1;
		speed *= (1.0f - lower_lim);
		speed += lower_lim;
		speed *= (MAXSPEED - 1);

		__HAL_TIM_SET_COMPARE(
			&htim4,
			(n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2,
			(uint16_t) speed);
		__HAL_TIM_SET_COMPARE(
			&htim4,
			(n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1,
			0);
	}
	else if (speed > 0.0f)
	{
		speed *= (1.0f - lower_lim);
		speed += lower_lim;
		speed *= (MAXSPEED - 1);

		__HAL_TIM_SET_COMPARE(
			&htim4,
			(n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1,
			(uint16_t) speed);
		__HAL_TIM_SET_COMPARE(
			&htim4,
			(n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2,
			0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_1 : TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_2 : TIM_CHANNEL_4, 0);
	}
}

void velociraptor_brake(void)
{
	uint32_t aux_arrel = __HAL_TIM_GET_AUTORELOAD(&htim4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, aux_arrel);
}

uint16_t sensor_data[16];
uint8_t n_sensor = 0;
float error = 0, prev_error = 0;
#define SENSOR_THRESHOLD  1000

void velociraptor_sensor_handler(void)
{
	sensor_data[n_sensor] = HAL_ADC_GetValue(&hadc2);

	n_sensor += 1;
	n_sensor %= 8;

	if(!n_sensor)
	{
		int32_t active_sensors = 0;
		int32_t weighted_sum = 0;

		prev_error = error;

		for(uint8_t n = 0; n < 8; n++)
		{
			if(sensor_data[n] > SENSOR_THRESHOLD)
			{
				active_sensors += 1;
				weighted_sum += n;
			}
		}

		if(active_sensors > 0)
		{
			error = (float) weighted_sum / (float) active_sensors;
			error -= 3.5f;
		}

		new_data_flag = 1;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, n_sensor & 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, n_sensor & 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, n_sensor & 4);

	HAL_ADC_Start(&hadc2);
}

void velociraptor_encoder_handler(void)
{
	uint32_t dwt_start_tick = DWT -> CYCCNT;

	for(uint8_t i = 0; i < 2; i++)
	{
		if(encoders_data.prev_state[i] == 0)
		{
			if(encoders_data.adc_read[i] > encoders_data.upp_thr[i])
			{
				//flanco positivo
				encoders_data.prev_state[i] = 1;

				if(encoders_data.prev_ticks[i] > encoders_data.tick_cntr)
				{
					uint32_t aux = 0xFFFFFFFF - encoders_data.prev_ticks[i];
					encoders_data.step_ticks[i] = encoders_data.tick_cntr + aux;
				}
				else
				{
					encoders_data.step_ticks[i] = encoders_data.tick_cntr - encoders_data.prev_ticks[i];
				}

				encoders_data.prev_ticks[i] = encoders_data.tick_cntr;
			}
		}
		else
		{
			if(encoders_data.adc_read[i] < encoders_data.lwr_thr[i])
			{
				//flanco negativo
				encoders_data.prev_state[i] = 0;
			}
		}
	}
	encoders_data.tick_cntr++;

	isr_dwt_ticks = DWT -> CYCCNT - dwt_start_tick;
	cycle_dwt_ticks = DWT -> CYCCNT - prev_dwt_tick;
	prev_dwt_tick = DWT -> CYCCNT;

}

void velociraptor_main_loop(void)
{
	switch(velo_mde_state)
	{
	case STOPPED:
		velociraptor_setmotorspeed(MOTOR_L, 0);
		velociraptor_setmotorspeed(MOTOR_R, 0);

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
		{
			velo_mde_state = RUNNING;
		}
		break;

	case RUNNING:
		if(new_data_flag)
		{
			new_data_flag = 0;

			pid_data.e_norm = error / 3.5f; // Normalización de error a -1, 1

			pid_data.e_int += pid_data.e_norm; // error integral
			pid_data.e_dv = pid_data.e_norm - pid_data.e_prev; // error diferencial
			pid_data.e_prev = pid_data.e_norm; // actualizacion de registro previo

			pid_data.e_pid = pid_data.kp * pid_data.e_norm;
			pid_data.e_pid += pid_data.ki * pid_data.e_int;
			pid_data.e_pid += pid_data.kd * pid_data.e_dv;

			pid_data.base_speed = pid_data.max_speed * (1.0f - pid_data.e_pid * (pid_data.e_pid < 0));

			pid_data.vel_l = pid_data.base_speed + pid_data.e_pid;
			pid_data.vel_r = pid_data.base_speed - pid_data.e_pid;

			velociraptor_setmotorspeed(MOTOR_L, pid_data.vel_l);
			velociraptor_setmotorspeed(MOTOR_R, pid_data.vel_r);
		}

		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
		{
			velo_mde_state = STOPPED;
			velociraptor_brake();
		}

		break;

	default:
		velo_mde_state = STOPPED;
	}
}
