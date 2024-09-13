/*
 * velociraptor3_comms.h
 *
 *  Created on: Sep 13, 2024
 *      Author: franciscomanoukian
 */

#ifndef INC_VELOCIRAPTOR3_COMMS_H_
#define INC_VELOCIRAPTOR3_COMMS_H_

typedef struct
{
	float max_speed, slope_correction_factor, brake_factor;
	float kp, ki, kd;
	uint8_t track_color;
} memory_data_t;

void velociraptor3_comms_loop(void);

#endif /* INC_VELOCIRAPTOR3_COMMS_H_ */
