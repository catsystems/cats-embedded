/*
 * buzzer.c
 *
 *  Created on: 24 Dec 2020
 *      Author: Luca
 */

// THIS IS A SOUND LIBRARY, WITH THIS YOU CAN PLAY MUSIC!!

#include "drivers/buzzer.h"
#include "cmsis_os2.h"

void buzzer_beep(BUZ *dev, uint32_t duration) {
  dev->end_time = osKernelGetTickCount() + duration;
  buzzer_start(dev);
}

// Set the volume between 0 and 100
void buzzer_set_volume(BUZ *dev, uint16_t volume) {
  if (volume > 100) volume = 100;

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (dev->arr / 200) * volume;  // set the pulse duration
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(dev->timer, &sConfigOC, dev->channel);

  // Start the pwm channel again if it should be running
  if (dev->started) HAL_TIM_PWM_Start(dev->timer, dev->channel);

  dev->volume = volume;
}

void buzzer_set_freq(BUZ *dev, float frequency) {
  // FREQ = CORE_FREQ / ((AAR+1) * (PSC+1))
  float core_freq = HAL_RCC_GetHCLKFreq();
  float psc = 1;
  if (frequency < 2000) psc = 2;
  if (frequency < 500) psc = 3;

  dev->arr = (uint16_t)(core_freq / (frequency * psc + frequency)) - 1;

  // Update timer period
  dev->timer->Init.Period = dev->arr;
  HAL_TIM_PWM_Init(dev->timer);

  // Update pulse as the freq changed
  buzzer_set_volume(dev, dev->volume);
}

void buzzer_start(BUZ *dev) {
  dev->started = 1;
  HAL_TIM_PWM_Start(dev->timer, dev->channel);  // start pwm generation
}

void buzzer_stop(BUZ *dev) {
  dev->started = 0;
  HAL_TIM_PWM_Stop(dev->timer, dev->channel);  // start pwm generation
}

// Stops the buzzer when time ran out
// Returns 1 if buzzer is still running
uint8_t buzzer_update(BUZ *dev) {
  if (dev->started && (dev->end_time > osKernelGetTickCount()))
    return 1;
  else if (dev->started)
    buzzer_stop(dev);
  return 0;
}
