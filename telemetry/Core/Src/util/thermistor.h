
#pragma once

#include <cmath>

template <uint32_t B> class Thermistor {
public:
  Thermistor(ADC_HandleTypeDef *adc) : adcHandle(adc) {
    HAL_ADC_Start_DMA(adc, &rawAdcValue, 1);
  }

  float getTemperature() {

    float resistance =
        10000.0f /
        ((65535.0f / static_cast<float>((uint16_t)rawAdcValue)) - 1.0f);

    float steinhart = logf(resistance / 10000.0f); // ln(R/R0)

    steinhart /= static_cast<float>(B);    // 1/B * ln(R/R0)
    steinhart += 1.0f / (25.0f + 273.15f); //
    steinhart = 1.0 / steinhart;           // Invert
    steinhart -= 273.15;                   // convert to C

    return steinhart;
  }

private:
  ADC_HandleTypeDef *adcHandle;
  uint32_t rawAdcValue;
};
