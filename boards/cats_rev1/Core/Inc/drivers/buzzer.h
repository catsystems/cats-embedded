/*
 * buzzer.h
 *
 *  Created on: Dec 24, 2020
 *      Author: Luca
 */

#ifndef INC_DRIVERS_BUZZER_H_
#define INC_DRIVERS_BUZZER_H_

#include "stm32l4xx_hal.h"

/** Exported Types **/

typedef struct buzzer_dev {
  // Hardware Configuration
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint16_t arr;
  uint8_t started;
  uint8_t start;
  uint16_t volume;
  uint32_t end_time;
} BUZ;

/** Exported Functions **/

void buzzer_set_volume(BUZ *dev, uint16_t volume);
void buzzer_beep(BUZ *dev, uint32_t duration);
void buzzer_set_freq(BUZ *dev, float frequency);
void buzzer_start(BUZ *dev);
void buzzer_stop(BUZ *dev);
uint8_t buzzer_update(BUZ *dev);

#endif /* INC_DRIVERS_BUZZER_H_ */
