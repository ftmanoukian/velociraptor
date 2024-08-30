/*
 * velociraptor2.c
 *
 *  Created on: Aug 23, 2024
 *      Author: franciscomanoukian
 */

#include "main.h"
#include "velociraptor2.h"
#include "velociraptor2_comms.h"
#include "ADXL345.h"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2, htim4;

extern adxl_data_t adxl_data;

enum
{
	stopped,
	running,
	braking
} robot_state = stopped;

struct
{
	uint16_t sensor_val[16];
	uint16_t threshold[8];
	uint8_t active_sensor;
	uint8_t flag_data_ready;
	uint8_t out_of_sight;
	float error, prev_error;
	enum
	{
		BUFFER_0,
		BUFFER_1
	} active_buffer;
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
	float slope_correction;
	float l_speed, r_speed;
	float brake_factor;
} speed;

struct
{
	GPIO_TypeDef * gpio[4];
	uint16_t pin[4];
	GPIO_PinState state[4];
	GPIO_PinState prev_state[4];
	uint8_t flag[4];
	uint8_t ticks[4];
} debounce_data;

uint8_t cross_line_flag = 0;
uint32_t brake_timer = 0;
float * error_ptr = &(line_sensor.error);

void velociraptor2_init(void)
{
	ADXL345_Deselect();
	if (ADXL345_CheckDevice()) {
		for(uint8_t i = 0; i < 2; i++)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			HAL_Delay(50);
		}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	ADXL345_Init();

	debounce_data.gpio[0] = GPIOB;
	debounce_data.gpio[1] = GPIOB;
	debounce_data.gpio[2] = GPIOC;
	debounce_data.gpio[3] = GPIOC;
	debounce_data.pin[0] = GPIO_PIN_11;
	debounce_data.pin[1] = GPIO_PIN_10;
	debounce_data.pin[2] = GPIO_PIN_15;
	debounce_data.pin[3] = GPIO_PIN_14;

	for(uint8_t i = 0; i < 4; i++)
	{
		debounce_data.state[i] = GPIO_PIN_SET;
		debounce_data.prev_state[i]	= GPIO_PIN_SET;
		debounce_data.flag[i] = 0;
		debounce_data.ticks[i] = 0;
	}

	// Datos sensores
	line_sensor.active_sensor = 0;
	line_sensor.active_buffer = BUFFER_0;
	line_sensor.flag_data_ready = 0;
	line_sensor.prev_error = 0.f;
	line_sensor.track_color = W_OVER_B;
	//específico al array de sensores que estoy usando!!!
	line_sensor.threshold[0] = 1000;
	line_sensor.threshold[1] = 1500;
	line_sensor.threshold[2] = 1500;
	line_sensor.threshold[3] = 1800;
	line_sensor.threshold[4] = 2000;
	line_sensor.threshold[5] = 1500;
	line_sensor.threshold[6] = 1500;
	line_sensor.threshold[7] = 1500;
	line_sensor.out_of_sight = 0;

	pid.correction = 0.f;
	pid.error_dv = 0.f;
	pid.error_int = 0.f;
	pid.kp = 1.f;
	pid.ki = 0.f;
	pid.kd = 0.6f;
	pid.prev_error = 0.f;

	speed.max_speed = 1.0f;
	speed.brake_factor = 0.7f;
	speed.slope_correction = 0.0f;

	// Timer adc
	HAL_TIM_Base_Start_IT(&htim2);

	// Timer motores
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void velociraptor2_motors_pid(void)
{
	velociraptor2_calc_error();

	pid.error_int += line_sensor.error;
	pid.error_dv = line_sensor.error - pid.prev_error;

	pid.correction = pid.kp * line_sensor.error;
	pid.correction += pid.ki * pid.error_int;
	pid.correction += pid.kd * pid.error_dv;

	if(pid.correction >= 0) speed.base_speed = 1.0f - pid.correction * speed.brake_factor;
	if(pid.correction < 0) speed.base_speed = 1.0f + pid.correction * speed.brake_factor;

	speed.l_speed = speed.max_speed * (1.0f - speed.slope_correction) * (speed.base_speed + pid.correction);
	speed.r_speed = speed.max_speed * (1.0f - speed.slope_correction) * (speed.base_speed - pid.correction);

	pid.prev_error = line_sensor.error;

	velociraptor2_setmotorspeed(MOTOR_L, speed.l_speed);
	velociraptor2_setmotorspeed(MOTOR_R, speed.r_speed);
}

void velociraptor2_slope_correction(void)
{
	uint16_t current_x = adxl_data.accel[adxl_data.active_buffer].x;

	if(current_x < 50)
	{
		speed.slope_correction = 0.0f;
	}
	else if(current_x >= 50 && current_x < 100)
	{
		speed.slope_correction = (current_x - 50) / 500.f;
	}
	else
	{
		speed.slope_correction = 1.0f;
	}
}

void velociraptor2_main_loop(void)
{
	velociraptor2_debounce_loop();

	switch(robot_state)
	{
	case stopped:

		if(debounce_data.flag[3] && !debounce_data.state[3])
		{
			debounce_data.flag[3] = 0;
			robot_state = running;
			pid.error_int = 0.f;
		}
		break;

	case running:
		if(line_sensor.flag_data_ready)
		{
			velociraptor2_slope_correction();
			if(speed.slope_correction > 0.95f)
			{
				velociraptor2_brake();
			}
			else
			{
				velociraptor2_motors_pid();
			}
		}


		if(debounce_data.flag[2] && !debounce_data.state[2])
		{
			velociraptor2_brake();
			debounce_data.flag[2] = 0;
			robot_state = stopped;
		}
		break;

	case braking:

		if(!brake_timer)
		{
			robot_state = running;
		}

		break;
	}
}

void velociraptor2_calc_error(void)
{
	line_sensor.flag_data_ready = 0;
	cross_line_flag = 1;

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
		if(n == 0 || n == 7) cross_line_flag &= cond;
	}

	if(active_sensors > 0)
	{
		line_sensor.error = (float) weighted_sum / ((float) active_sensors * 3.5f);
		line_sensor.error -= 1.f;
		line_sensor.out_of_sight = 0;
	}
	else
	{
		if(line_sensor.error > 0.95f || line_sensor.error < -0.95f)
		{
			line_sensor.out_of_sight = 1;
		}
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

void velociraptor2_debounce_loop(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		GPIO_PinState current_state = HAL_GPIO_ReadPin(debounce_data.gpio[i], debounce_data.pin[i]);
		if(current_state != debounce_data.prev_state[i])
		{
			debounce_data.prev_state[i] = current_state;
			debounce_data.ticks[i] = DEBOUNCE_TICKS;
		}
		if(debounce_data.ticks[i])
		{
			debounce_data.ticks[i]--;
			if(!debounce_data.ticks[i])
			{
				debounce_data.state[i] = current_state;
				debounce_data.flag[i] = 1;
			}
		}
	}
}
