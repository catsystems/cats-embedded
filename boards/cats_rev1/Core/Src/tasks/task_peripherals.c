/*
 * task_peripherals.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "config/globals.h"
#include "util/log.h"
#include "tasks/task_peripherals.h"
#include "main.h"

/** Private Constants **/

static const int_fast8_t PERIPHERALS_SAMPLING_FREQ = 10;

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void task_peripherals(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  flight_fsm_t fsm_state = {.flight_state = MOVING};

  uint8_t parachute_fired = 0;

  chute_type_t chute_type;

  /* Read from Config the Chute Type */
  chute_type.stages = 1;
  chute_type.stage_type_1 = SERVO_1_TRIGGER;
  chute_type.servo_angle_1 = 0;
  chute_type.servo_angle_2 = 0;

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / PERIPHERALS_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;
    fsm_state = global_flight_state;

    if (fsm_state.flight_state == APOGEE && fsm_state.state_changed) {
      if (chute_type.stages == 1) {
        switch (chute_type.stage_type_1) {
          case SERVO_1_TRIGGER:
            servo_set_position(&SERVO1, chute_type.servo_angle_1);
            break;
          case SERVO_2_TRIGGER:
            servo_set_position(&SERVO2, chute_type.servo_angle_2);
            break;
          case SERVO_1_2_TRIGGER:
            servo_set_position(&SERVO1, chute_type.servo_angle_1);
            servo_set_position(&SERVO2, chute_type.servo_angle_2);
            break;
          case PYRO_1_TRIGGER:
            HAL_GPIO_WritePin(GPIOA, PYRO_1_Pin, GPIO_PIN_SET);
            break;
          case PYRO_2_TRIGGER:
            HAL_GPIO_WritePin(GPIOA, PYRO_2_Pin, GPIO_PIN_SET);
            break;
          case PYRO_3_TRIGGER:
            HAL_GPIO_WritePin(GPIOA, PYRO_3_Pin, GPIO_PIN_SET);
            break;
          case ALL_PYROS_TRIGGER:
            HAL_GPIO_WritePin(GPIOA, PYRO_1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, PYRO_2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, PYRO_3_Pin, GPIO_PIN_SET);
            break;
          default:
            break;
        }
      }
    }

    /* TODO: Check if we have actually triggered the Parachute */

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
