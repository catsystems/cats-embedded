/*
 * buzzer.h
 *
 *  Created on: Dec 24, 2020
 *      Author: Luca
 */

#ifndef INC_DRIVERS_BUZZER_H_
#define INC_DRIVERS_BUZZER_H_

#include "stm32l4xx_hal.h"

#define BUZZER_INIT()                                                      \
  {                                                                        \
    .timer = &htim15, .channel = TIM_CHANNEL_2, .arr = 4000, .started = 0, \
    .volume = 100                                                          \
  }

typedef struct buzzer_dev {
  // Hardware Configuration
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint16_t arr;

  uint8_t started;
  uint16_t volume;
  uint32_t end_time;

} BUZ;

extern TIM_HandleTypeDef htim15;

void buzzer_set_volume(struct buzzer_dev *dev, uint16_t volume);
void buzzer_beep(struct buzzer_dev *dev, uint32_t duration);
void buzzer_set_freq(struct buzzer_dev *dev, float frequency);
void buzzer_start(struct buzzer_dev *dev);
void buzzer_stop(struct buzzer_dev *dev);
uint8_t buzzer_update(struct buzzer_dev *dev);

#endif /* INC_DRIVERS_BUZZER_H_ */
