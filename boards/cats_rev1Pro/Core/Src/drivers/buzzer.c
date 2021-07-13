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
  dev->start = 1;
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
  if (dev->started)
    HAL_TIM_PWM_Start(dev->timer, dev->channel);
  else
    HAL_TIM_PWM_Stop(dev->timer, dev->channel);

  dev->volume = volume;
}

// Set buzzer frequency between 200 and 10kHz

void buzzer_set_freq(BUZ *dev, uint32_t frequency) {
  // FREQ = CORE_FREQ / ((AAR+1) * (PSC+1))
  uint32_t core_freq = HAL_RCC_GetHCLKFreq();
  uint32_t psc = 1;
  // guards
  if (frequency > 10000)
    frequency = 10000;
  else if (frequency < 200)
    frequency = 200;

  if (frequency < 500)
    psc = 3;
  else if (frequency < 2000)
    psc = 2;

  dev->arr = (uint16_t)(core_freq / (frequency * psc + frequency)) - 1;

  // Update timer period
  dev->timer->Init.Period = dev->arr;
  HAL_TIM_PWM_Init(dev->timer);

  // Update pulse as the freq changed
  buzzer_set_volume(dev, dev->volume);
}

// Starts pwm timer
void buzzer_start(BUZ *dev) {
  dev->started = 1;
  HAL_TIM_PWM_Start(dev->timer, dev->channel);  // start pwm generation
}

// Stops pwm timer
void buzzer_stop(BUZ *dev) {
  dev->started = 0;
  HAL_TIM_PWM_Stop(dev->timer, dev->channel);  // stop pwm generation
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

// Starts and Stops the buzzer
// Returns 1 if buzzer is running
uint8_t buzzer_update(BUZ *dev) {
  if (dev->start) {
    buzzer_start(dev);
    dev->start = 0;
  }
  if (dev->started && (dev->end_time > osKernelGetTickCount()))
    return 1;
  else if (dev->started)
    buzzer_stop(dev);
  return 0;
}
