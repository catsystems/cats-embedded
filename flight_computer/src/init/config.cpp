/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "config.hpp"

#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "drivers/servo.hpp"
#include "util/actions.hpp"
#include "util/log.h"

#include "tasks/task_peripherals.hpp"

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
}

static void create_event_map() {
  /* number of event types + 0th element */
  event_action_map = static_cast<event_action_map_elem_t *>(pvPortMalloc(NUM_EVENTS * sizeof(event_action_map_elem_t)));
  if (event_action_map == nullptr) {
    // TODO: set some error, beep!!
    log_raw("Could not allocate memory for event_action_map!");
    return;
  }
  memset(event_action_map, 0, NUM_EVENTS * sizeof(event_action_map_elem_t));

  uint16_t nr_actions{0};
  config_action_t action{};
  // Loop over all events
  for (int ev_idx = 0; ev_idx < NUM_EVENTS; ev_idx++) {
    nr_actions = cc_get_num_actions(static_cast<cats_event_e>(ev_idx));
    // If an action is mapped to the event
    if (nr_actions > 0) {
      event_action_map[ev_idx].num_actions = nr_actions;
      event_action_map[ev_idx].action_list =
          static_cast<peripheral_act_t *>(pvPortMalloc(nr_actions * sizeof(peripheral_act_t)));
      if (event_action_map[ev_idx].action_list == nullptr) {
        // TODO: set some error, beep!!
        log_raw("Could not allocate memory for actions in event_action_map[%d].action_list!", ev_idx);
        return;
      }
      memset(event_action_map[ev_idx].action_list, 0, nr_actions * sizeof(peripheral_act_t));

      // Loop over all actions
      for (uint16_t act_idx = 0; act_idx < nr_actions; act_idx++) {
        if (cc_get_action(static_cast<cats_event_e>(ev_idx), act_idx, &action)) {
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
      ev_timers[i].timer_init_event = static_cast<cats_event_e>(global_cats_config.timers[i].start_event);
      ev_timers[i].execute_event = static_cast<cats_event_e>(global_cats_config.timers[i].trigger_event);
      ev_timers[i].timer_duration_ticks = global_cats_config.timers[i].duration;
    }
  }

  /* Create Timers */
  for (uint32_t i = 0; i < NUM_TIMERS; i++) {
    if (global_cats_config.timers[i].duration > 0) {
      // NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast)
      ev_timers[i].timer_id = osTimerNew(reinterpret_cast<osTimerFunc_t>(trigger_event), osTimerOnce,
                                         reinterpret_cast<void *>(ev_timers[i].execute_event), nullptr);
      // NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast)
    }
  }
}
