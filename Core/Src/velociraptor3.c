/*
 * velociraptor3.c
 *
 *  Created on: Sep 12, 2024
 *      Author: franciscomanoukian
 */


#include "main.h"
#include "velociraptor3.h"
#include "velociraptor3_comms.h"
#include "ADXL345.h"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2, htim4;
extern memory_data_t memory_data;

extern adxl_data2_t adxl_data2;

/* begin momento cabeza */
extern uint8_t led_parp_flag;
/* end momento cabeza */

enum
{
	stopped,
	comms,
	running,
	cleaning
} robot_state = stopped;

struct
{
	GPIO_TypeDef * gpio;
	uint16_t pin;
	GPIO_PinState state;
	GPIO_PinState prev_state;
	uint8_t flag;
	uint8_t ticks;
} debounce[4];

struct
{
	float correction, prev_error;
	float error_int; 
	float error_dv;
	float kp, kd, ki;
} pid;

struct
{
	uint8_t sensor_val[16];
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
		B_OVER_W,
		AUTO
	} track_color;
} sensors;

struct
{
	float max_speed, base_speed;
	float slope_correction;
	float l_speed, r_speed;
	float brake_factor;
} speed;

void velociraptor3_init(void)
{
	ADXL345_Deselect();
	if (ADXL345_CheckDevice())
	{
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

	velociraptor3_debounce_init();
	velociraptor3_sensors_init();
	velociraptor3_pid_init();
	velociraptor3_speed_init();
	velociraptor3_timers_init();

	led_parp_flag = 0;
}

void velociraptor3_timers_init(void)
{
	// Timer ADC
	HAL_TIM_Base_Start_IT(&htim2);

	// Timer Motores
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void velociraptor3_debounce_init(void)
{
	debounce[0].gpio = GPIOB;
	debounce[1].gpio = GPIOB;
	debounce[2].gpio = GPIOC;
	debounce[3].gpio = GPIOC;
	debounce[0].pin = GPIO_PIN_11;
	debounce[1].pin = GPIO_PIN_10;
	debounce[2].pin = GPIO_PIN_15;
	debounce[3].pin = GPIO_PIN_14;

	for(uint8_t i = 0; i < 4; i++)
	{
		debounce[i].state = GPIO_PIN_SET;
		debounce[i].prev_state = GPIO_PIN_SET;
		debounce[i].flag = 0;
		debounce[i].ticks = 0;
	}
}

void velociraptor3_speed_init(void)
{
	speed.max_speed = 0.9f;
	speed.brake_factor = .0f;
	speed.slope_correction = 0.0f;
}

void velociraptor3_pid_init(void)
{
	pid.correction = 0.f;
	pid.error_dv = 0.f;
	pid.error_int = 0.f;

	// TODO: cargar desde mem
	pid.kp = 2.5f;
	pid.ki = 0.0f;
	pid.kd = 0.5f;

	pid.prev_error = 0.f;
}

void velociraptor3_sensors_init(void)
{
	sensors.active_sensor = 0;
	sensors.active_buffer = BUFFER_0;
	sensors.flag_data_ready = 0;
	sensors.prev_error = 0.f;
	sensors.track_color = AUTO;		// TODO: cargar desde mem
	
	sensors.threshold[0] = 1000;
	sensors.threshold[1] = 1500;
	sensors.threshold[2] = 1500;
	sensors.threshold[3] = 1800;
	sensors.threshold[4] = 2000;
	sensors.threshold[5] = 1500;
	sensors.threshold[6] = 1500;
	sensors.threshold[7] = 1500;
}

void velociraptor3_main_loop(void)
{
	switch(robot_state)
	{
	case stopped:

		if(debounce[2].flag && !debounce[2].state)
		{
			debounce[2].flag = 0;
			pid.error_int = 0.f;
			led_parp_flag = 1;
			robot_state = running;
		}
		else if(debounce[1].flag && !debounce[1].state)
		{
			debounce[1].flag = 0;
			speed.l_speed = .5f;
			speed.r_speed = .5f;
			velociraptor3_setpwm();
			robot_state = cleaning;
		}
		/*else if(debounce[0].flag && !debounce[0].state)
		{
			debounce[0].flag = 0;
			robot_state = comms;

			memory_data.kp = pid.kp;
			memory_data.ki = pid.ki;
			memory_data.kd = pid.kd;
			memory_data.brake_factor = speed.brake_factor;
			memory_data.max_speed = speed.max_speed;
			memory_data.slope_correction_factor = speed.slope_correction;
			memory_data.track_color = sensors.track_color;
		}*/
		
		break;

	case running:
		
		if(sensors.flag_data_ready)
		{
			sensors.flag_data_ready = 0;
			velociraptor3_motors_pid();
		}
		
		if(
			debounce[0].flag && !debounce[0].state ||
			debounce[1].flag && !debounce[1].state ||
			debounce[2].flag && !debounce[2].state ||
			debounce[3].flag && !debounce[3].state
		)
		{
			debounce[0].flag = 0;
			debounce[1].flag = 0;
			debounce[2].flag = 0;
			debounce[3].flag = 0;
			velociraptor3_brake();
			led_parp_flag = 0;
			robot_state = stopped;
		}

		break;

	case cleaning:

		if(debounce[1].flag && debounce[1].state)
		{
			debounce[1].flag = 0;
			speed.l_speed = 0.0f;
			speed.r_speed = 0.0f;
			velociraptor3_setpwm();
			robot_state = stopped;
		}

		break;

	case comms:

		velociraptor3_comms_loop();

		if(debounce[0].flag && !debounce[0].state)
		{
			debounce[0].flag = 0;
			robot_state = stopped;

			pid.kp = memory_data.kp;
			pid.ki = memory_data.ki;
			pid.kd = memory_data.kd;
			speed.brake_factor = memory_data.brake_factor;
			speed.max_speed = memory_data.max_speed;
			speed.slope_correction = memory_data.slope_correction_factor;
			sensors.track_color = memory_data.track_color;
		}

		break;
	}
}

void velociraptor3_timer_handler(void)
{
	velociraptor3_sensors_routine();
}

void velociraptor3_sensors_routine(void)
{
	uint16_t adc_read = HAL_ADC_GetValue(&hadc2);
	sensors.sensor_val[sensors.active_sensor] = (uint8_t) (adc_read > sensors.threshold[sensors.active_sensor % 8]);

	sensors.active_sensor += 1;
	sensors.active_sensor %= 16;

	if(!(sensors.active_sensor % 8))
	{
		if(!sensors.active_sensor) sensors.active_buffer = BUFFER_0;
		else sensors.active_buffer = BUFFER_1;

		sensors.flag_data_ready = 1;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, sensors.active_sensor & 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, sensors.active_sensor & 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, sensors.active_sensor & 4);

	HAL_ADC_Start(&hadc2);
}

void velociraptor3_motors_pid(void)
{
	velociraptor3_calc_error();
	velociraptor3_calc_slope();

	pid.error_int += sensors.error;
	pid.error_dv = sensors.error - pid.prev_error;

	pid.correction = pid.kp * sensors.error;
	pid.correction += pid.ki * pid.error_int;
	pid.correction += pid.kd * pid.error_dv;

	speed.base_speed = speed.max_speed;
	/*
	 * if(pid.correction >= speed.max_speed)
	{
		speed.base_speed -= pid.correction;
	}
	else if(pid.correction <= speed.max_speed * -1.f)
	{
		speed.base_speed += pid.correction;
	}*/

	speed.l_speed = speed.base_speed + pid.correction;
	speed.r_speed = speed.base_speed - pid.correction;

	/*speed.base_speed = 1.0f;
	if(pid.correction > 0.0f)
	{
		speed.base_speed -= pid.correction * speed.brake_factor;
	}
	else
	{
		speed.base_speed += pid.correction * speed.brake_factor;
	}
	speed.base_speed *= (1.0f - speed.slope_correction);

	speed.l_speed = speed.max_speed * (speed.base_speed + pid.correction);
	speed.r_speed = speed.max_speed * (speed.base_speed - pid.correction);*/

	velociraptor3_setpwm();
}

void velociraptor3_calc_slope(void)
{
	if(adxl_data2.x_avg > 80.f)
	{
		speed.slope_correction = 1.0f;
	}
	else if(adxl_data2.x_avg > 30.f)
	{
		speed.slope_correction = (adxl_data2.x_avg - 30.f) * 0.02f;
	}
	else
	{
		speed.slope_correction = 0.0f;
	}
}

void velociraptor3_calc_error(void)
{
	uint8_t * buffer_ptr;
	uint8_t white_count = 0, black_count;
	float white_sum = 0, black_sum = 0;

	// buffer apuntando a la posición de lectura (buffer inact.)
	buffer_ptr = sensors.sensor_val;
	buffer_ptr += 8 * (sensors.active_buffer == BUFFER_1);

	// análisis de sensores
	for(uint8_t n = 0; n < 8; n++)
	{
		white_count += sensors.sensor_val[n];
		
		if(!sensors.sensor_val[n]) black_sum += ((float) n - 3.5f);
		else white_sum += ((float) n - 3.5f);
	}

	// auxiliar nomá
	black_count = 8 - white_count;

	// división sólo si hay sensores activos!!!
	if(sensors.track_color == W_OVER_B || (sensors.track_color == AUTO && white_count < 4))
	{
		if(white_count > 0) sensors.error = (float) white_sum / ((float) white_count * 3.5f);
	}
	else if(sensors.track_color == B_OVER_W || (sensors.track_color == AUTO && black_count < 4))
	{
		if(black_count > 0) sensors.error = (float) black_sum / ((float) black_count * 3.5f);
	}

	sensors.prev_error = sensors.error;
}

void velociraptor3_brake(void)
{
	uint32_t aux_arrel = __HAL_TIM_GET_AUTORELOAD(&htim4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, aux_arrel);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, aux_arrel);
}

void velociraptor3_setpwm(void)
{
	float local_speed;
	const float lower_lim = (float) MIN_SPEED / (float) MAX_SPEED;

	for(uint8_t n_motor = MOTOR_L; n_motor <= MOTOR_R; n_motor++)
	{
		local_speed = (n_motor == MOTOR_L) ? speed.l_speed : speed.r_speed;

		if(local_speed > 1.0f) local_speed = 1.0f;
		if(local_speed < -1.0f) local_speed = -1.0f;

		if(local_speed < 0.0f)
		{
			local_speed *= -1;
			local_speed *= (1.0f - lower_lim);
			local_speed += lower_lim;
			local_speed *= (MAX_SPEED - 1);

			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2, (uint16_t) local_speed);
			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1, 0);
		}
		else if(local_speed > 0.0f)
		{
			local_speed *= (1.0f - lower_lim);
			local_speed += lower_lim;
			local_speed *= (MAX_SPEED - 1);

			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_4 : TIM_CHANNEL_1, (uint16_t) local_speed);
			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_3 : TIM_CHANNEL_2, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_1 : TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim4, (n_motor == MOTOR_L) ? TIM_CHANNEL_2 : TIM_CHANNEL_4, 0);
		}
	}
}

void velociraptor3_debounce_loop(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		GPIO_PinState current_state = HAL_GPIO_ReadPin(debounce[i].gpio, debounce[i].pin);
		if(current_state != debounce[i].prev_state)
		{
			debounce[i].prev_state = current_state;
			debounce[i].ticks = DEBOUNCE_TICKS;
		}
		if(debounce[i].ticks)
		{
			debounce[i].ticks--;
			if(!debounce[i].ticks)
			{
				debounce[i].state = current_state;
				debounce[i].flag = 1;
			}
		}
	}
}
