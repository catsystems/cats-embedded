/*
 * adc.h
 *
 *  Created on: Jan 7, 2021
 *      Author: Luca
 */

#ifndef INC_DRIVERS_ADC_H_
#define INC_DRIVERS_ADC_H_

#define ADC_NUM_CHANNELS 4

#include "stm32l4xx_hal.h"

typedef enum {
  ADC_BATTERY = 0,
  ADC_PYRO1,
  ADC_PYRO2,
  ADC_PYRO3
} adc_channels_e;

void adc_init();
uint32_t adc_get(adc_channels_e channel);

extern ADC_HandleTypeDef hadc1;

#endif /* INC_DRIVERS_ADC_H_ */
