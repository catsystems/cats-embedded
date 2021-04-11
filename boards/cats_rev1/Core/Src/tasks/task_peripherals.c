/*
 * task_peripherals.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "config/globals.h"
#include "util/types.h"
#include "util/log.h"
#include "tasks/task_peripherals.h"
#include "util/recorder.h"
#include "main.h"

/** Private Constants **/

const uint32_t EVENT_QUEUE_SIZE = 16;

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void task_peripherals(void* argument) {
  //  flight_fsm_t fsm_state = {.flight_state = MOVING};
  //
  //  uint8_t parachute_fired = 0;
  //  uint8_t trigger_parachute = 0;
  //  chute_type_t chute_type;
  //
  //  /* Read from Config the Chute Type */
  //  chute_type.stages = 1;
  //  chute_type.stage_type_1 = SERVO_1_TRIGGER;
  //  chute_type.servo_angle_1 = 0;
  //  chute_type.servo_angle_2 = 0;

  cats_event_e curr_event;
  while (true) {
    if (osMessageQueueGet(event_queue, &curr_event, NULL, osWaitForever) ==
        osOK) {
      peripheral_out_t* output_list = event_output_map[curr_event].output_list;
      for (uint32_t i = 0; i < event_output_map[curr_event].num_outputs; ++i) {
        timestamp_t curr_ts = osKernelGetTickCount();
        /* get the actuator function */
        peripheral_out_fp curr_fp = output_list[i].func_ptr;
        if (curr_fp != NULL) {
          log_warn("EXECUTING EVENT: %d, output_idx: %lu", curr_event, i);
          /* call the actuator function */
          curr_fp();
          event_info_t event_info = {
              .ts = curr_ts, .event = curr_event, .output_idx = i};
          record(EVENT_INFO, &event_info);
          uint32_t curr_delay = output_list[i].delay_ms;
          /* sleep if you need to */
          if (curr_delay > 0) {
            /* Be careful when setting these delays, right now the peripheral
             * task will sleep until this delay is finished, meaning that it
             * won't read from the queue until the delay is done. If you set too
             * long of a delay you might execute the next event too late, even
             * if the event came at the right time.
             *
             * Right now this is considered a feature and not a bug since we
             * assume the users know what they are doing when setting up these
             * delays. */
            osDelay(curr_delay);
          }
        }
      }
    }
  }

  //  while (1) {
  //    tick_count += tick_update;
  //    fsm_state = global_flight_state;
  //
  //    /* Check If we need to Trigger Parachute */
  //    if (fsm_state.flight_state == APOGEE && fsm_state.state_changed) {
  //      trigger_parachute = 1;
  //    }

  //    if (trigger_parachute == 1) {
  //      if (chute_type.stages == 1) {
  //        switch (chute_type.stage_type_1) {
  //          case SERVO_1_TRIGGER:
  //            servo_set_position(&SERVO1, chute_type.servo_angle_1);
  //            break;
  //          case SERVO_2_TRIGGER:
  //            servo_set_position(&SERVO2, chute_type.servo_angle_2);
  //            break;
  //          case SERVO_1_2_TRIGGER:
  //            servo_set_position(&SERVO1, chute_type.servo_angle_1);
  //            servo_set_position(&SERVO2, chute_type.servo_angle_2);
  //            break;
  //          case PYRO_1_TRIGGER:
  //            HAL_GPIO_WritePin(GPIOC, PYRO_1_Pin, GPIO_PIN_SET);
  //            break;
  //          case PYRO_2_TRIGGER:
  //            HAL_GPIO_WritePin(GPIOC, PYRO_2_Pin, GPIO_PIN_SET);
  //            break;
  //          case PYRO_3_TRIGGER:
  //            HAL_GPIO_WritePin(GPIOC, PYRO_3_Pin, GPIO_PIN_SET);
  //            break;
  //          case ALL_PYROS_TRIGGER:
  //            HAL_GPIO_WritePin(GPIOC, PYRO_1_Pin, GPIO_PIN_SET);
  //            HAL_GPIO_WritePin(GPIOC, PYRO_2_Pin, GPIO_PIN_SET);
  //            HAL_GPIO_WritePin(GPIOC, PYRO_3_Pin, GPIO_PIN_SET);
  //            break;
  //          default:
  //            break;
  //        }
  //      }
  //    }
  //
  //    /* Check if we have actually triggered the Parachute */
  //    if (chute_type.stages == 1) {
  //      switch (chute_type.stage_type_1) {
  //        case SERVO_1_TRIGGER:
  //          /* Can we Read the Servo Angle?*/
  //          break;
  //        case SERVO_2_TRIGGER:
  //          /* Can we Read the Servo Angle?*/
  //          break;
  //        case SERVO_1_2_TRIGGER:
  //          /* Can we Read the Servo Angle?*/
  //          break;
  //        case PYRO_1_TRIGGER:
  //          /* Read Pyro Voltage */
  //          break;
  //        case PYRO_2_TRIGGER:
  //          /* Read Pyro Voltage */
  //          break;
  //        case PYRO_3_TRIGGER:
  //          /* Read Pyro Voltage */
  //          break;
  //        case ALL_PYROS_TRIGGER:
  //          /* Read Pyro Voltage */
  //          break;
  //        default:
  //          break;
  //      }
  //    }

  // osDelayUntil(tick_count);
  //  }
}

osStatus_t trigger_event(cats_event_e ev) {
  log_warn("Event %d added to the queue", ev);
  /* TODO: check if timeout should be 0 here */
  return osMessageQueuePut(event_queue, &ev, 0U, 0U);
}

/** Private Function Definitions **/
