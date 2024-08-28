/*
 * ADXL345.c
 *
 *  Created on: Aug 28, 2024
 *      Author: franciscomanoukian
 */

#include "main.h"
#include "ADXL345.h"

extern SPI_HandleTypeDef hspi1;

struct
{
	uint8_t active_buffer;
	struct
	{
		int16_t x, y, z;
	} accel[2];
} adxl_data;

void ADXL345_Select(void) {
    HAL_GPIO_WritePin(ADXL345_SPI_CS_PORT, ADXL345_SPI_CS_PIN, GPIO_PIN_RESET); // CS low
}

void ADXL345_Deselect(void) {
    HAL_GPIO_WritePin(ADXL345_SPI_CS_PORT, ADXL345_SPI_CS_PIN, GPIO_PIN_SET); // CS high
}

void ADXL345_Init(void) {
	ADXL345_WriteRegister(0x2D, 0x08); // POWER_CTL register (0x2D): Set to measurement mode

	// Set the output data rate (ODR) and range
	ADXL345_WriteRegister(0x2C, 0x0A); // BW_RATE register (0x2C): Set ODR to 100 Hz (0x0A)

	// Set the full-scale range
	ADXL345_WriteRegister(0x31, 0x09); // DATA_FORMAT register (0x31): Full-resolution, ±16g (0x0B)

	// Ensure FIFO is in bypass mode
	ADXL345_WriteRegister(0x38, 0x00); // FIFO_CTL register (0x38): FIFO in bypass mode

	// Enable Data Ready interrupt
	ADXL345_WriteRegister(0x2E, 0x80); // INT_ENABLE register (0x2E): Enable Data Ready interrupt

	// Map Data Ready interrupt to INT1
	ADXL345_WriteRegister(0x2F, 0x00); // INT_MAP register (0x2F): Route Data Ready to INT1 (bit 0 = 0)

	/*ADXL345_Deselect();
	volatile uint8_t aux = ADXL345_ReadRegister(0x00);
	asm volatile("nop");
	aux = ADXL345_ReadRegister(0x2c);
	asm volatile("nop");
	aux = ADXL345_ReadRegister(0x30);
	asm volatile("nop");*/
}

uint8_t ADXL345_ReadRegister(uint8_t reg) {
    uint8_t receivedData;
    reg |= 0x80; // Set MSB to 1 for read operation

    ADXL345_Select();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);   // Send register address
    HAL_SPI_Receive(&hspi1, &receivedData, 1, HAL_MAX_DELAY); // Read register value
    ADXL345_Deselect();

    return receivedData;
}

void ADXL345_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t txData[2];

    // Combine register address and value into a transmission array
    txData[0] = reg;       // Register address
    txData[1] = value;     // Data to write

    ADXL345_Select();
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY); // Transmit register address and data
    ADXL345_Deselect();
}

void ADXL345_ReadXYZ(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t x0, x1, y0, y1, z0, z1;

    // Read X-axis
    x0 = ADXL345_ReadRegister(0x32); // X0
    x1 = ADXL345_ReadRegister(0x33); // X1

    // Read Y-axis
    y0 = ADXL345_ReadRegister(0x34); // Y0
    y1 = ADXL345_ReadRegister(0x35); // Y1

    // Read Z-axis
    z0 = ADXL345_ReadRegister(0x36); // Z0
    z1 = ADXL345_ReadRegister(0x37); // Z1

    // Combine high and low bytes
    *x = (int16_t)((x1 << 8) | x0);
    *y = (int16_t)((y1 << 8) | y0);
    *z = (int16_t)((z1 << 8) | z0);
}

uint8_t ADXL345_CheckDevice(void) {
    uint8_t deviceID = ADXL345_ReadRegister(0x00); // 0x00 is the DEVID register
    if (deviceID == 0xE5) {
        return 1; // Device is present and responding correctly
    } else {
        return 0; // Device is not responding
    }
}

void ADXL345_IRQHandler(void)
{
	int16_t *x, *y, *z;

	//escritura en buffer inactivo
	x = &(adxl_data.accel[!adxl_data.active_buffer].x);
	y = &(adxl_data.accel[!adxl_data.active_buffer].y);
	z = &(adxl_data.accel[!adxl_data.active_buffer].z);

	ADXL345_ReadXYZ(x, y, z);

	adxl_data.active_buffer = !adxl_data.active_buffer;
}
