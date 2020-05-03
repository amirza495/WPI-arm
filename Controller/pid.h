/*
 * pid.h
 *
 *  Created on: May 2, 2020
 *      Author: Adam Mirza
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct{
	float p;	// proportional
	float i;	// integral
	float d;	// derivative
}PID_t;

typedef struct{
	float pos;	// position (deg)
	uint32_t t;	// time (ms)
}State_t;

typedef struct{
	PID_t K;		// gain values
	PID_t e;		// error values
	State_t last;	// previous state
	State_t curr;	// current state
	uint32_t dt;	// timestep (ms)
	float u;		// control input (V)
	float pos; 		// setpoint (deg)
}PIDCtrl_t;


void PID_init(PIDCtrl_t *pid, float Kp, float Ki, float Kd, float setPoint, float initialPos);
void PID_update(PIDCtrl_t *pid, float pos);
void PID_changeSetpoint(PIDCtrl_t *pid, float pos);

#endif /* PID_H_ */
