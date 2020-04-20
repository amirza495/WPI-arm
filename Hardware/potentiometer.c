/*
 * potentiometer.c
 *
 *  Created on: Apr 19, 2020
 *      Author: Adam Mirza
 */

#include "potentiometer.h"

extern Potentiometer_t gLowerPot;
extern Potentiometer_t gUpperPot;

uint16_t adcBuff[2];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle){
	gLowerPot.rawPot = adcBuff[0];
	gUpperPot.rawPot = adcBuff[1];
}

void Potentiometer_read(){

	HAL_ADC_Start_DMA(&hadc1, adcBuff, 2);

	HAL_ADC_PollForEvent(&hadc1, ADC_EOSMP_EVENT, 1000);

}

void Potentiometer_getPosition(Potentiometer_t *pot){

	pot->pos = (float) (pot->rawPot - pot->cal.up)/pot->cal.convFactor;

}

