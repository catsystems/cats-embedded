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

#include "tasks/task_peripherals.h"
#include "cmsis_os.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "flash/recorder.h"
#include "util/actions.h"
#include "util/enum_str_maps.h"
#include "util/log.h"
#include "util/types.h"

/** Private Constants **/

const uint32_t EVENT_QUEUE_SIZE = 16;

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_peripherals(__attribute__((unused)) void* argument) {
  cats_event_e curr_event;
  while (true) {
    if (osMessageQueueGet(event_queue, &curr_event, nullptr, osWaitForever) == osOK) {
      /* Start Timer if the Config says so */
      for (uint32_t i = 0; i < NUM_TIMERS; i++) {
        if ((ev_timers[i].timer_id != nullptr) && (curr_event == ev_timers[i].timer_init_event)) {
          if (osTimerStart(ev_timers[i].timer_id, ev_timers[i].timer_duration_ticks) != osOK) {
            log_warn("Starting TIMER %lu with event %lu failed.", i, curr_event);
          }
        }
      }

#ifdef USE_PCHANNEL_SAFETY_LOCK
      /* Arm the pyro channels when going into ready */
      if (curr_event >= EV_READY) {
        HAL_GPIO_WritePin(PYRO_EN_GPIO_Port, PYRO_EN_Pin, GPIO_PIN_SET);
      }
      /* Disarm the pyro channels when going into moving */
      else if (curr_event == EV_MOVING) {
        HAL_GPIO_WritePin(PYRO_EN_GPIO_Port, PYRO_EN_Pin, GPIO_PIN_RESET);
      }
#endif
      peripheral_act_t* action_list = event_action_map[curr_event].action_list;
      uint8_t num_actions = event_action_map[curr_event].num_actions;
      for (uint32_t i = 0; i < num_actions; ++i) {
        timestamp_t curr_ts = osKernelGetTickCount();
        /* get the actuator function */
        peripheral_act_fp curr_fp = action_table[action_list[i].action];
        if (curr_fp != nullptr) {
          log_error("EXECUTING EVENT: %s, ACTION: %s, ACTION_ARG: %u", event_map[curr_event],
                    action_map[action_list[i].action], action_list[i].action_arg);
          /* call the actuator function */
          curr_fp(action_list[i].action_arg);
          event_info_t event_info = {.event = curr_event, .action = action_list[i]};
          record(curr_ts, EVENT_INFO, &event_info);
        }
      }
      if (num_actions == 0) {
        log_error("EXECUTING EVENT: %s, ACTION: %s", event_map[curr_event], action_map[action_list[0].action]);
        timestamp_t curr_ts = osKernelGetTickCount();
        event_info_t event_info = {.event = curr_event, .action = {ACT_NO_OP}};
        record(curr_ts, EVENT_INFO, &event_info);
      }
    }
  }
}

osStatus_t trigger_event(cats_event_e ev) {
  log_warn("Event %lu added to the queue", ev);
  /* TODO: check if timeout should be 0 here */
  return osMessageQueuePut(event_queue, &ev, 0U, 10U);
}

/** Private Function Definitions **/
