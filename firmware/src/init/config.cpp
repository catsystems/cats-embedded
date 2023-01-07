/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "config.h"

#include "config/cats_config.h"
#include "config/globals.h"
#include "drivers/servo.h"
#include "util/actions.h"
#include "util/log.h"

#include "tasks/task_peripherals.h"

static void create_event_map();
static void init_timers();

void load_and_set_config() {
  HAL_Delay(100);
  cc_init();
  cc_load();
  log_info("Config initialization complete.");

  HAL_Delay(100);
  create_event_map();
  init_timers();

  HAL_Delay(100);
  servo_set_position(&SERVO1, global_cats_config.initial_servo_position[0]);
  servo_set_position(&SERVO2, global_cats_config.initial_servo_position[1]);

  servo_start(&SERVO1);
  servo_start(&SERVO2);
}

static void create_event_map() {
  /* number of event types + 0th element */
  event_action_map = (event_action_map_elem_t *)(pvPortMalloc(NUM_EVENTS * sizeof(event_action_map_elem_t)));
  if (event_action_map == nullptr) {
    // TODO: set some error, beep!!
    log_raw("Could not allocate memory for event_action_map!");
    return;
  }
  memset(event_action_map, 0, NUM_EVENTS * sizeof(event_action_map_elem_t));

  uint16_t nr_actions;
  config_action_t action;
  // Loop over all events
  for (int ev_idx = 0; ev_idx < NUM_EVENTS; ev_idx++) {
    nr_actions = cc_get_num_actions((cats_event_e)(ev_idx));
    // If an action is mapped to the event
    if (nr_actions > 0) {
      event_action_map[ev_idx].num_actions = nr_actions;
      event_action_map[ev_idx].action_list = (peripheral_act_t *)(pvPortMalloc(nr_actions * sizeof(peripheral_act_t)));
      if (event_action_map[ev_idx].action_list == nullptr) {
        // TODO: set some error, beep!!
        log_raw("Could not allocate memory for actions in event_action_map[%d].action_list!", ev_idx);
        return;
      }
      memset(event_action_map[ev_idx].action_list, 0, nr_actions * sizeof(peripheral_act_t));

      // Loop over all actions
      for (uint16_t act_idx = 0; act_idx < nr_actions; act_idx++) {
        if (cc_get_action((cats_event_e)(ev_idx), act_idx, &action)) {
          event_action_map[ev_idx].action_list[act_idx].action = static_cast<action_function_e>(action.action_idx);
          event_action_map[ev_idx].action_list[act_idx].action_arg = action.arg;
        } else {
          // If we cannot find the action in the config, set the num of actions to the last successfully found action
          event_action_map[ev_idx].num_actions = act_idx;
          break;
        }
      }
    }
  }
}

static void init_timers() {
  /* Init timers */
  for (uint32_t i = 0; i < NUM_TIMERS; i++) {
    if (global_cats_config.timers[i].duration > 0) {
      ev_timers[i].timer_init_event = (cats_event_e)global_cats_config.timers[i].start_event;
      ev_timers[i].execute_event = (cats_event_e)global_cats_config.timers[i].trigger_event;
      ev_timers[i].timer_duration_ticks = global_cats_config.timers[i].duration;
    }
  }

  /* Create Timers */
  for (uint32_t i = 0; i < NUM_TIMERS; i++) {
    if (global_cats_config.timers[i].duration > 0) {
      ev_timers[i].timer_id =
          osTimerNew((osTimerFunc_t)trigger_event, osTimerOnce, (void *)ev_timers[i].execute_event, nullptr);
    }
  }
}
