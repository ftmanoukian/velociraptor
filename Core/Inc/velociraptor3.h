/*
 * velociraptor3.h
 *
 *  Created on: Sep 12, 2024
 *      Author: franciscomanoukian
 */

#ifndef INC_VELOCIRAPTOR3_H_
#define INC_VELOCIRAPTOR3_H_

#define MAX_SPEED 	720
#define MIN_SPEED   350
#define MOTOR_L		1
#define MOTOR_R		2

#define DEBOUNCE_TICKS			20

void velociraptor2_init(void);
void velociraptor3_timers_init(void);
void velociraptor3_debounce_init(void);
void velociraptor3_speed_init(void);
void velociraptor3_pid_init(void);
void velociraptor3_sensors_init(void);
void velociraptor3_main_loop(void);
void velociraptor3_timer_handler(void);
void velociraptor3_sensors_routine(void);
void velociraptor3_motors_pid(void);
void velociraptor3_calc_error(void);
void velociraptor3_brake(void);
void velociraptor3_setpwm(void);
void velociraptor3_debounce_loop(void);

#endif /* INC_VELOCIRAPTOR3_H_ */
