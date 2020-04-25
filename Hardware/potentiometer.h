/*
 * potentiometer.h
 *
 *  Created on: Apr 19, 2020
 *      Author: Adam Mirza
 */

#ifndef POTENTIOMETER_H_
#define POTENTIOMETER_H_

#include <stdint.h>
#include "adc.h"

typedef struct{
	uint16_t zero; // lsb
	float convFactor; // lsb/deg
}PotCalibration_t;

typedef struct{
	PotCalibration_t cal;
	uint16_t rawPot;
	float pos;
}Potentiometer_t;

void Potentiometer_read(void);

void Potentiometer_getPosition(Potentiometer_t *pot);

#endif /* POTENTIOMETER_H_ */
