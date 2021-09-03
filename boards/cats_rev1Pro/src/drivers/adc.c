/*
 * adc.c
 *
 *  Created on: 7 Jan 2021
 *      Author: Luca
 */

#include "drivers/adc.h"

static uint32_t adc_value[ADC_NUM_CHANNELS];

void adc_init() {
  HAL_ADC_Stop_DMA(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc_value, ADC_NUM_CHANNELS);
}

uint32_t adc_get(adc_channels_e channel) {
  if ((adc_value[0] | adc_value[1]) == 0) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, adc_value, ADC_NUM_CHANNELS);
  }

  if ((channel < 0) || (channel > (ADC_NUM_CHANNELS - 1))) return 0;

  return adc_value[channel];
}
