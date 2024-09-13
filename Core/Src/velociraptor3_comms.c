/*
 * velociraptor3_comms.c
 *
 *  Created on: Sep 13, 2024
 *      Author: franciscomanoukian
 *
 *
 *  NO LEER ESTE ARCHIVO - PARECE PROGRAMADO POR UN ALUMNO DE 7MO GRADO. ME EXCUSO POR LA HORA (03:50)
 *  despues lo voy a reescribir como corresponde. ahora lo importante es que "anda (?"
 */

#include "main.h"
#include "velociraptor3_comms.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;

memory_data_t memory_data;

uint8_t rx_flag;
char rx_buffer[20];
char tx_buffer[10];

enum
{
	idle,
	validate_rx,
	recv_val,
	validate_val
} comms_state;

char mensajes_menu[8][15] = {
	"Configuracion\n",
	"1. kp\n",
	"2. ki\n",
	"3. kd\n",
	"4. freno\n",
	"5. vel. max\n",
	"6. k. pend\n",
	"7. color base\n"
};

uint8_t sel_option = 0;
uint8_t rx_pos = 0;

void velociraptor3_comms_loop(void)
{
	switch(comms_state)
	{
	case idle:
		for(uint8_t i = 0; i < 8; i++)
		{
			HAL_UART_Transmit_DMA(&huart1, (uint8_t *) mensajes_menu[i], strlen(mensajes_menu[i]));
			while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		}
		HAL_UART_Receive_DMA(&huart1, (uint8_t *) rx_buffer, 1);
		comms_state = validate_rx;
		break;
	case validate_rx:
		if(rx_flag)
		{
			rx_flag = 0;

			sel_option = atoi(&rx_buffer[0]);
			if (sel_option == 0 || sel_option > 7)
			{
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "invalido\n", 9);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n======\n", 8);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				comms_state = idle;
			}
			else
			{
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n", 1);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "actual: ", 8);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				switch(sel_option)
				{
				case 1:
					sprintf(tx_buffer, "%.6f", memory_data.kp);
					break;
				case 2:
					sprintf(tx_buffer, "%.6f", memory_data.ki);
					break;
				case 3:
					sprintf(tx_buffer, "%.6f", memory_data.kd);
					break;
				case 4:
					sprintf(tx_buffer, "%.6f", memory_data.brake_factor);
					break;
				case 5:
					sprintf(tx_buffer, "%.6f", memory_data.max_speed);
					break;
				case 6:
					sprintf(tx_buffer, "%.6f", memory_data.slope_correction_factor);
					break;
				case 7:
					if(memory_data.track_color == 0) 		strcpy(tx_buffer, "N (negro)");
					else if(memory_data.track_color == 1) 	strcpy(tx_buffer, "B (blanc)");
					else if(memory_data.track_color == 2) 	strcpy(tx_buffer, "A (autom)");
					break;
				default:
					comms_state = idle;
				}
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) tx_buffer, 10);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n======\n", 8);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				comms_state = recv_val;
				rx_pos = 0;
				HAL_UART_Receive_DMA(&huart1, &(rx_buffer[rx_pos]), 1);
			}
		}
	case recv_val:

		if(rx_flag)
		{
			rx_flag = 0;

			if(rx_buffer[rx_pos] != '\n' && rx_buffer[rx_pos] != '\0' && rx_pos < 9)
			{
				rx_pos++;
				HAL_UART_Receive_DMA(&huart1, &(rx_buffer[rx_pos]), 1);
			}
			else
			{
				rx_buffer[rx_pos] = '\0';
				comms_state = validate_val;
			}
		}
		break;

	case validate_val:
		float aux;

		if(sel_option != 7)
		{
			uint8_t * end_ptr;
			aux = strtof(rx_buffer, &end_ptr);

			if(end_ptr == rx_buffer || *end_ptr != '\0')
			{
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "invalido\n", 9);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n======\n", 8);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				comms_state = idle;
			}
		}
		else
		{
			if(rx_buffer[0] != 'B' && rx_buffer[0] != 'N' && rx_buffer[0] != 'A')
			{
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "invalido\n", 9);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n======\n", 8);
				while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
				comms_state = idle;
			}
		}
		if(comms_state != idle)
		{
			switch(sel_option)
			{
			case 1:
				memory_data.kp = aux;
				break;
			case 2:
				memory_data.ki = aux;
				break;
			case 3:
				memory_data.kd = aux;
				break;
			case 4:
				memory_data.brake_factor = aux;
				break;
			case 5:
				memory_data.max_speed = aux;
				break;
			case 6:
				memory_data.slope_correction_factor = aux;
				break;
			case 7:
				switch(rx_buffer[0])
				{
				case 'N':
					memory_data.track_color = 0;
					break;
				case 'B':
					memory_data.track_color = 1;
					break;
				case 'A':
					memory_data.track_color = 2;
					break;
				}
				break;
			default:
				comms_state = idle;
			}

			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "aplicado\n", 9);
			while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) "\n======\n", 8);
			while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
			comms_state = idle;
		}

		break;

	default:

		comms_state = idle;
		break;
	}
}
