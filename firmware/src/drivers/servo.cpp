/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "drivers/servo.h"
#include "cmsis_os2.h"

static void set_pwm(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t pulse) {
  TIM_OC_InitTypeDef sConfigOC;
  HAL_TIM_PWM_Stop(timer, channel);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;  // set the pulse duration
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, channel);
  HAL_TIM_PWM_Start(timer, channel);
}

void servo_start(SERVO *dev) {
  dev->started = 1;
  set_pwm(dev->timer, dev->channel, dev->pulse);
}

void servo_stop(SERVO *dev) {
  dev->started = 0;
  HAL_TIM_PWM_Stop(dev->timer, dev->channel);
}

// Set Servo angle from 0 to 180 degree
void servo_set_position(SERVO *dev, uint16_t angle) {
  if (angle > 180) angle = 180;
  dev->pulse = (14000 / 180) * angle + 8000;
  if (dev->started) set_pwm(dev->timer, dev->channel, dev->pulse);
}

void servo_set_onoff(SERVO *dev, bool status) {
  if (status) {
    dev->pulse = 199999;
  } else {
    dev->pulse = 0;
  }
  set_pwm(dev->timer, dev->channel, dev->pulse);
}
