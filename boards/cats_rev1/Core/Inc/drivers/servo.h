/*
 * servo.h
 *
 *  Created on: 5 Jan 2021
 *      Author: Luca
 */

#pragma once

#include "stm32l4xx_hal.h"

typedef struct servo_dev {
  // Hardware Configuration
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  uint32_t pulse;
  int8_t started;
} SERVO;

void servo_set_position(SERVO *dev, uint16_t angle);
void servo_set_onoff(SERVO *dev, uint16_t status);
void servo_start(SERVO *dev);
void servo_stop(SERVO *dev);
