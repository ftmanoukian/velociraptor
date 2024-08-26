/*
 * velociraptor2.c
 *
 *  Created on: Aug 23, 2024
 *      Author: franciscomanoukian
 */

#include "main.h"
#include "velociraptor2.h"
#include "velociraptor2_comms.h"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2, htim4;

typedef enum
{
	BUFFER_0,
	BUFFER_1
} active_buffer_t;

enum
{
	stopped,
	running
} robot_state = stopped;

struct
{
	uint16_t sensor_val[16];
	uint16_t threshold[8];
	uint8_t active_sensor;
	uint8_t flag_data_ready;
	active_buffer_t active_buffer;
	float error, prev_error;
	enum
	{
		W_OVER_B,
		B_OVER_W
	} track_color;
} line_sensor;

struct
{
	float correction, prev_error;
	float error_int;
	float error_dv;
	float kp, ki, kd;
} pid;

struct
{
	float max_speed, base_speed;
	float l_speed, r_speed;
	float brake_factor;
} speed;

uint8_t cross_line_flag = 0;
float * error_ptr = &(line_sensor.error);

void velociraptor2_init(void)
{
	// Datos sensores
	line_sensor.active_sensor = 0;
	line_sensor.active_buffer = BUFFER_0;
	line_sensor.flag_data_ready = 0;
	line_sensor.prev_error = 0.f;
	line_sensor.track_color = W_OVER_B;
	line_sensor.threshold[0] = 1000;
	line_sensor.threshold[1] = 1500;
	line_sensor.threshold[2] = 1500;
	line_sensor.threshold[3] = 1800;
	line_sensor.threshold[4] = 2000;
	line_sensor.threshold[5] = 1500;
	line_sensor.threshold[6] = 1500;
	line_sensor.threshold[7] = 1500;

	pid.correction = 0.f;
	pid.error_dv = 0.f;
	pid.error_int = 0.f;
	pid.kp = 1.f;
	pid.ki = 0.f;
	pid.kd = 0.f;
	pid.prev_error = 0.f;

	speed.max_speed = 1.0f;
	speed.brake_factor = 1.0f;

	// Timer adc
	HAL_TIM_Base_Start_IT(&htim2);

	// Timer motores
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

float max_speed = 1.0f;

void velociraptor2_main_loop(void)
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET && robot_state == stopped)
	{
		robot_state = running;
	}
	else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET && robot_state == running)
	{
		robot_state = stopped;
	}

	if(line_sensor.flag_data_ready)
	{
		velociraptor2_calc_error();

		switch(robot_state)
		{
		case stopped:
			velociraptor2_brake();
			break;

		case running:
			pid.error_int += line_sensor.error;
			pid.error_dv = line_sensor.error - pid.prev_error;

			pid.correction = pid.kp * line_sensor.error;
			pid.correction += pid.ki * pid.error_int;
			pid.correction += pid.kd * pid.error_dv;

			if(pid.correction >= 0) speed.base_speed = 1.0f - pid.correction * speed.brake_factor;
			if(pid.correction < 0) speed.base_speed = 1.0f + pid.correction * speed.brake_factor;

			speed.l_speed = speed.max_speed * (speed.base_speed - pid.correction);
			speed.r_speed = speed.max_speed * (speed.base_speed + pid.correction);

			pid.prev_error = line_sensor.error;

			velociraptor2_setmotorspeed(MOTOR_L, speed.l_speed);
			velociraptor2_setmotorspeed(MOTOR_R, speed.r_speed);
			break;
		}
	}
}

void velociraptor2_calc_error(void)
{
	line_sensor.flag_data_ready = 0;

	uint16_t * buffer_ptr = line_sensor.sensor_val;
	buffer_ptr += 8 * (line_sensor.active_buffer == BUFFER_1);

	int32_t weighted_sum = 0;
	int32_t active_sensors = 0;

	line_sensor.error = line_sensor.prev_error;

	for(uint8_t n = 0; n < 8; n++)
	{
		uint8_t cond = (line_sensor.track_color == W_OVER_B ?
				buffer_ptr[n] > line_sensor.threshold[n] :
				buffer_ptr[n] < line_sensor.threshold[n]);

		active_sensors += cond;
		weighted_sum += n * cond;
	}

	if(active_sensors >= 6)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		cross_line_flag = 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		cross_line_flag = 0;
	}
	if(active_sensors > 0)
	{
		line_sensor.error = (float) weighted_sum / ((float) active_sensors * 3.5f);
		line_sensor.error -= 1.f;
	}

	line_sensor.prev_error = line_sensor.error;
}

void velociraptor2_timer_handler(void)
{
	velociraptor2_linesensor_routine();
}

void velociraptor2_linesensor_routine(void)
{
	line_sensor.sensor_val[line_sensor.active_sensor] = HAL_ADC_GetValue(&hadc2);

	line_sensor.active_sensor += 1;
	line_sensor.active_sensor %= 16;

	if(!(line_sensor.active_sensor % 8))
	{
		if(!line_sensor.active_sensor) line_sensor.active_buffer = BUFFER_0;
		else line_sensor.active_buffer = BUFFER_1;

		line_sensor.flag_data_ready = 1;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, line_sensor.active_sensor & 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, line_sensor.active_sensor & 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, line_sensor.active_sensor & 4);

	HAL_ADC_Start(&hadc2);
}

void velociraptor2_brake(void)
{
	uint32_t aux_arrel = __HAL_TIM_GET_AUTORELOAD(&htim4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, aux_arrel);
}

void velociraptor2_setmotorspeed(uint8_t n_motor, float speed)
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

		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2, (uint16_t) speed);
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1, 0);
	}
	else if (speed > 0.0f)
	{
		speed *= (1.0f - lower_lim);
		speed += lower_lim;
		speed *= (MAXSPEED - 1);

		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1, (uint16_t) speed);
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_1 : TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_2 : TIM_CHANNEL_4, 0);
	}
}
