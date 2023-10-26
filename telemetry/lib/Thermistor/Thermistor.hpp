/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cmath>
#include <cstdint>

#include <stm32g0xx_hal.h>

template <uint32_t B>
class Thermistor {
 public:
  explicit Thermistor(ADC_HandleTypeDef *adc) : adcHandle(adc) { HAL_ADC_Start_DMA(adc, &rawAdcValue, 1); }

  float getTemperature() {
    float resistance = 10000.0F / ((65535.0F / static_cast<float>(static_cast<uint16_t>(rawAdcValue))) - 1.0F);

    float steinhart = logf(resistance / 10000.0F);  // ln(R/R0)

    steinhart /= static_cast<float>(B);     // 1/B * ln(R/R0)
    steinhart += 1.0F / (25.0F + 273.15F);  //
    steinhart = 1.0F / steinhart;           // Invert
    steinhart -= 273.15F;                   // convert to C

    return steinhart;
  }

 private:
  ADC_HandleTypeDef *adcHandle;
  uint32_t rawAdcValue{0};
};
