/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "drivers/adc.hpp"
#include "target.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static uint32_t adc_value[ADC_NUM_CHANNELS];

void adc_init() { HAL_ADC_Start_DMA(&ADC_HANDLE, adc_value, ADC_NUM_CHANNELS); }

uint32_t adc_get(adc_channels_e channel) {
  // If data of the first and last adc channel is frozen, reset the hardware
  if ((adc_value[0] | adc_value[ADC_BATTERY]) == 0) {
    HAL_ADC_Stop_DMA(&ADC_HANDLE);
    HAL_ADC_Start_DMA(&ADC_HANDLE, adc_value, ADC_NUM_CHANNELS);
  }

  if ((channel < 0) || (channel > (ADC_NUM_CHANNELS - 1))) {
    return 0;
  }

  return adc_value[channel];
}
