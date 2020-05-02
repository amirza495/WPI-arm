/*
 * motor.c
 *
 *  Created on: Apr 24, 2020
 *      Author: Adam Mirza
 */
#include "motor.h"

void Motor_init(Motor_t *dcMotor){

	/* turn off motor direction pins */
	HAL_GPIO_WritePin(GPIOA, dcMotor->dir1, RESET);
	HAL_GPIO_WritePin(GPIOA, dcMotor->dir2, RESET);

	/* activate pwm channel */
	HAL_TIM_PWM_Start(&htim1, dcMotor->channel);

	/* set motor voltage to 0 V */
	Motor_setSpeed(dcMotor, 0);

}

void Motor_setSpeed(Motor_t *dcMotor, float voltage){

	/* set voltage value */
	dcMotor->voltage = voltage;

	/* handle direction setting, and get absolute value of voltage */
	if(voltage > 0){
		/* moving up */

		/* set direction pins */
		HAL_GPIO_WritePin(GPIOA, dcMotor->dir2, SET);
		HAL_GPIO_WritePin(GPIOA, dcMotor->dir1, RESET);


	}else if(voltage < 0){
		/* moving down */

		/* set direction pins */
		HAL_GPIO_WritePin(GPIOA, dcMotor->dir1, SET);
		HAL_GPIO_WritePin(GPIOA, dcMotor->dir2, RESET);

		/* reverse input voltage */
		voltage *= -1;

	}

	/* calculate period based on input voltage
	 * 1) input voltage normalized to 0.1 scale to calculate duty cycle
	 * 2) duty cycle converted to period value
	 * 3) period set on PWM channel
	 * */
	uint32_t period = (voltage * PWM_PERIOD) / 12 ;
	__HAL_TIM_SET_COMPARE(&htim1, dcMotor->channel, period);


}
