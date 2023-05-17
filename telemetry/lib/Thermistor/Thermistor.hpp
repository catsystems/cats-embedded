/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <stdint.h>
#include <stm32g0xx_hal.h>
#include <cmath>

template <uint32_t B>
class Thermistor {
 public:
  Thermistor(ADC_HandleTypeDef *adc) : adcHandle(adc) { HAL_ADC_Start_DMA(adc, &rawAdcValue, 1); }

  float getTemperature() {
    float resistance = 10000.0f / ((65535.0f / static_cast<float>((uint16_t)rawAdcValue)) - 1.0f);

    float steinhart = logf(resistance / 10000.0f);  // ln(R/R0)

    steinhart /= static_cast<float>(B);     // 1/B * ln(R/R0)
    steinhart += 1.0f / (25.0f + 273.15f);  //
    steinhart = 1.0 / steinhart;            // Invert
    steinhart -= 273.15;                    // convert to C

    return steinhart;
  }

 private:
  ADC_HandleTypeDef *adcHandle;
  uint32_t rawAdcValue;
};
