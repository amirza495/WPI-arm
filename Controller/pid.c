/*
 * pid.c
 *
 *  Created on: May 2, 2020
 *      Author: Adam Mirza
 */

#include "pid.h"

void PID_init(PIDCtrl_t *pid, float Kp, float Ki, float Kd, float setPoint, float initialPos)
{
	/* initialize gains */
	pid->K.p = Kp;
	pid->K.i = Ki;
	pid->K.d = Kd;

	/* zero errors */
	pid->e.p = 0;
	pid->e.i = 0;
	pid->e.d = 0;

	/* set initial values */
	/* setpoint */
	pid->pos = setPoint;

	/* initial state */
	pid->last.t = HAL_GetTick();
	pid->last.pos = initialPos;


}

void PID_update(PIDCtrl_t *pid, float pos)
{
	/* set current position */
	pid->curr.pos = pos;

	/* calculate time step */
	pid->curr.t = HAL_GetTick();
	pid->dt = pid->curr.t - pid->last.t;

	/* calculate errors */
	pid->e.p = pid->pos - pid->curr.pos;
	pid->e.i += pid->e.p * pid->dt/1000;
	pid->e.d = (pid->curr.pos - pid->last.pos)/(pid->dt/1000);

	/* calculate control input */
	pid->u = pid->K.p * pid->e.p + pid->K.i * pid->e.i;

	/* check bounds on control input */
	if(pid->u > 12){
		pid->u = 12.0;
	}else if(pid->u < -12){
		pid->u = -12.0;
	}

	/* update last values */
	pid->last.t = pid->curr.t;
	pid->last.pos = pid->curr.pos;

}

void PID_changeSetpoint(PIDCtrl_t *pid, float setPoint)
{
	/* update setpoint */
	pid->pos = setPoint;
}
