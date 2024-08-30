/*
 * velociraptor2.h
 *
 *  Created on: Aug 23, 2024
 *      Author: franciscomanoukian
 */

#ifndef INC_VELOCIRAPTOR2_H_
#define INC_VELOCIRAPTOR2_H_

#define MAXSPEED 	720
#define MOTOR_L		1
#define MOTOR_R		2

#define MIN_LOADED_SPEED		350

#define DEBOUNCE_TICKS			20

void velociraptor2_init(void);
void velociraptor2_main_loop(void);
void velociraptor2_calc_error(void);
void velociraptor2_timer_handler(void);
void velociraptor2_linesensor_routine(void);
void velociraptor2_brake(void);
void velociraptor2_setmotorspeed(uint8_t n_motor, float speed);
void velociraptor2_debounce_loop(void);

#endif /* INC_VELOCIRAPTOR2_H_ */
