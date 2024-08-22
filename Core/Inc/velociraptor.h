/*
 * velociraptor.h
 *
 *  Created on: Aug 21, 2024
 *      Author: franciscomanoukian
 */

#ifndef INC_VELOCIRAPTOR_H_
#define INC_VELOCIRAPTOR_H_

#define MAXSPEED 	720
#define MOTOR_L		1
#define MOTOR_R		2

#define MIN_LOADED_SPEED		350

#define ENC_BLK		0
#define ENC_WHT		1

void velociraptor_start(void);
void velociraptor_setmotorspeed(uint8_t n_motor, float speed);
void velociraptor_brake(void);
void velociraptor_sensor_handler(void);
void velociraptor_encoder_handler(void);
void velociraptor_main_loop(void);

#endif /* INC_VELOCIRAPTOR_H_ */
