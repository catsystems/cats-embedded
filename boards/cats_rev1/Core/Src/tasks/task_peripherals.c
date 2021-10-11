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
_Noreturn void task_peripherals(__attribute__((unused)) void* argument) {
  cats_event_e curr_event;
  while (true) {
    if (osMessageQueueGet(event_queue, &curr_event, NULL, osWaitForever) == osOK) {
      /* Start Timer if the Config says so */
      for (uint32_t i = 0; i < num_timers; i++) {
        if (curr_event == ev_timers[i].timer_init_event) {
          osTimerStart(ev_timers[i].timer_id, ev_timers[i].timer_duration_ticks);
        }
      }
      /* start Mach timer if needed */
      if (curr_event == mach_timer.timer_init_event) {
        if (mach_timer.timer_duration_ticks > 0) {
          osTimerStart(mach_timer.timer_id, mach_timer.timer_duration_ticks);
        }
      }
      peripheral_act_t* action_list = event_action_map[curr_event].action_list;
      for (uint32_t i = 0; i < event_action_map[curr_event].num_actions; ++i) {
        timestamp_t curr_ts = osKernelGetTickCount();
        /* get the actuator function */
        peripheral_act_fp curr_fp = action_list[i].func_ptr;
        if (curr_fp != NULL) {
          log_warn("EXECUTING EVENT: %d, action_idx: %lu", curr_event, i);
          /* call the actuator function */
          curr_fp(action_list[i].func_arg);
          event_info_t event_info = {.ts = curr_ts, .event = curr_event, .action_idx = i};
          record(EVENT_INFO, &event_info);
        }
      }
    }
  }
}

osStatus_t trigger_event(cats_event_e ev) {
  log_warn("Event %d added to the queue", ev);
  /* TODO: check if timeout should be 0 here */
  return osMessageQueuePut(event_queue, &ev, 0U, 0U);
}

/** Private Function Definitions **/
