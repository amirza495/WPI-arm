/*
 * motor.h
 *
 *  Created on: Apr 24, 2020
 *      Author: Adam Mirza
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "tim.h"
#include <stdint.h>

typedef struct{
	uint32_t channel;
	float voltage;
	uint16_t dir1;
	uint16_t dir2;
}Motor_t;

void Motor_init(Motor_t *dcMotor);

void Motor_setSpeed(Motor_t *dcMotor, float voltage);


#endif /* MOTOR_H_ */
