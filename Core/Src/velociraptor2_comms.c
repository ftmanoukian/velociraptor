/*
 * velociraptor2_comms.c
 *
 *  Created on: Aug 25, 2024
 *      Author: franciscomanoukian
 */

#include "main.h"
#include "velociraptor2_comms.h"
#include "stdlib.h"

extern UART_HandleTypeDef huart1;
extern float * error_ptr;
extern struct
{
	float correction, prev_error;
	float error_int;
	float error_dv;
	float kp, ki, kd;
} pid;

extern struct
{
	float max_speed, base_speed;
	float l_speed, r_speed;
	float brake_factor;
} speed;

enum
{
	idle,
	receiving,
	processing
} comms_state = idle;

uint8_t rx_buff[11];
uint8_t tx_buff[3] = "ok";
uint8_t err_tx_buff[4] = "err";
uint8_t rx_process_flag = 0;

void velociraptor2_comms_init(void)
{
	rx_buff[10] = '\0';
}

void velociraptor2_comms_loop(void)
{
	switch(comms_state)
	{
	case idle:
		HAL_UART_Receive_DMA(&huart1, rx_buff, 10);

		comms_state = receiving;
		break;

	case receiving:
		if(rx_process_flag)
		{
			rx_process_flag = 0;
			comms_state = processing;
		}
		break;

	case processing:
		switch(rx_buff[0])
		{
		case 'p':
			pid.kp = atoff((char *) &(rx_buff[1]));
			HAL_UART_Transmit_DMA(&huart1, tx_buff, 3);
			break;
		case 'i':
			pid.ki = atoff((char *) &(rx_buff[1]));
			HAL_UART_Transmit_DMA(&huart1, tx_buff, 3);
			break;
		case 'd':
			pid.kd = atoff((char *) &(rx_buff[1]));
			HAL_UART_Transmit_DMA(&huart1, tx_buff, 3);
			break;
		case 'M':
			speed.max_speed = atoff((char *) &(rx_buff[1]));
			HAL_UART_Transmit_DMA(&huart1, tx_buff, 3);
			break;
		case 'b':
			speed.brake_factor = atoff((char *) &(rx_buff[1]));
			HAL_UART_Transmit_DMA(&huart1, tx_buff, 3);
			break;
		default:
			HAL_UART_Transmit_DMA(&huart1, err_tx_buff, 4);
			break;
		}

		comms_state = idle;
		break;
	}
}
