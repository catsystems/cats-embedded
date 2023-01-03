/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "tasks/task_health_monitor.h"
#include "cmsis_os.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "drivers/adc.h"
#include "util/actions.h"
#include "util/battery.h"
#include "util/buzzer_handler.h"
#include "util/log.h"
#include "util/task_util.h"

#include "tasks/task_simulator.h"
#include "tasks/task_usb_communicator.h"

/** Private Constants **/

/** Private Function Declarations **/
static void check_high_current_channels();

static void init_communication();

SET_TASK_PARAMS(task_usb_communicator, 512)

/** Exported Function Definitions **/

[[noreturn]] void task_health_monitor(__attribute__((unused)) void *argument) {
  // an increase of 1 on the timer means 10 ms
  uint32_t ready_timer = 0;
  uint32_t pyro_check_timer = 0;
  uint32_t voltage_logging_timer = 0;
  flight_fsm_e old_fsm_state = MOVING;
  battery_level_e old_level = BATTERY_OK;

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (1) {
    if (global_usb_detection == true && usb_communication_complete == false) {
      init_communication();
    }

    // Check battery level
    battery_level_e level = battery_level();
    bool level_changed = (old_level != level);

    if ((level == BATTERY_CRIT) && level_changed) {
      clear_error(CATS_ERR_BAT_LOW);
      add_error(CATS_ERR_BAT_CRITICAL);
    } else if ((level == BATTERY_LOW) && level_changed) {
      clear_error(CATS_ERR_BAT_CRITICAL);
      add_error(CATS_ERR_BAT_LOW);
    } else if (level_changed) {
      clear_error(CATS_ERR_BAT_LOW);
      clear_error(CATS_ERR_BAT_CRITICAL);
    }

    /* Record voltage information with 1Hz. */
    if (++voltage_logging_timer >= 100) {
      voltage_logging_timer = 0;
      uint16_t voltage = battery_voltage_short();
      record(osKernelGetTickCount(), VOLTAGE_INFO, &voltage);
    }

    old_level = battery_level();

    // Periodically check pyros channels as long as we are on the ground
    if ((global_flight_state.flight_state < THRUSTING) && (pyro_check_timer >= 200)) {
      check_high_current_channels();
      pyro_check_timer = 0;
    } else {
      ++pyro_check_timer;
    }

    // Beep out ready buzzer
    if ((global_flight_state.flight_state == READY) && (ready_timer >= 500)) {
      buzzer_queue_status(CATS_BUZZ_READY);
      ready_timer = 0;
    } else if (global_flight_state.flight_state == READY) {
      ++ready_timer;
    }

    // Beep out transitions from moving to ready and back
    if (global_flight_state.flight_state == READY && (global_flight_state.flight_state != old_fsm_state))
      buzzer_queue_status(CATS_BUZZ_CHANGED_READY);
    if (global_flight_state.flight_state == MOVING && (global_flight_state.flight_state != old_fsm_state))
      buzzer_queue_status(CATS_BUZZ_CHANGED_MOVING);

    // Update the buzzer
    buzzer_handler_update();

    old_fsm_state = global_flight_state.flight_state;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
static void check_high_current_channels() {
  bool error_encountered = false;
  // Loop over all events
  for (int ev_idx = 0; ev_idx < NUM_EVENTS; ev_idx++) {
    uint16_t nr_actions = cc_get_num_actions((cats_event_e)ev_idx);
    // If an action is mapped to the event
    if (nr_actions > 0) {
      // Loop over all actions
      for (uint16_t act_idx = 0; act_idx < nr_actions; act_idx++) {
        config_action_t action;
        if (cc_get_action((cats_event_e)ev_idx, act_idx, &action) == true) {
          switch (action.action_idx) {
            case ACT_HIGH_CURRENT_ONE:
              if (adc_get(ADC_PYRO1) < 500) {
                add_error(CATS_ERR_NO_PYRO);
                error_encountered = true;
              }
              break;
            case ACT_HIGH_CURRENT_TWO:
              if (adc_get(ADC_PYRO2) < 500) {
                add_error(CATS_ERR_NO_PYRO);
                error_encountered = true;
              }
              break;
            case ACT_HIGH_CURRENT_THREE:
              if (adc_get(ADC_PYRO3) < 500) {
                add_error(CATS_ERR_NO_PYRO);
                error_encountered = true;
              }
              break;
            default:
              break;
          }
        }
      }
    }
  }
  if (error_encountered == false) clear_error(CATS_ERR_NO_PYRO);
}

static void init_communication() {
  osThreadNew(task_usb_communicator, nullptr, &task_usb_communicator_attributes);
  usb_communication_complete = true;
}
