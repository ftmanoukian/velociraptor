/*
 * ADXL345.h
 *
 *  Created on: Aug 28, 2024
 *      Author: franciscomanoukian
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#define ADXL345_SPI_CS_PORT GPIOA
#define ADXL345_SPI_CS_PIN GPIO_PIN_15

typedef struct
{
	uint8_t active_buffer;
	struct
	{
		int16_t x, y, z;
	} accel[2];
} adxl_data_t;

void ADXL345_Select(void);
void ADXL345_Deselect(void);
void ADXL345_Init(void);
uint8_t ADXL345_ReadRegister(uint8_t reg);
void ADXL345_WriteRegister(uint8_t reg, uint8_t value);
void ADXL345_ReadXYZ(int16_t *x, int16_t *y, int16_t *z);
uint8_t ADXL345_CheckDevice(void);
void ADXL345_IRQHandler(void);

#endif /* INC_ADXL345_H_ */
