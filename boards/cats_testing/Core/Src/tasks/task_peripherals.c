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
  cats_event_e curr_event;

  /* Create Timers */
  for (uint32_t i = 0; i < num_timers; i++) {
    ev_timers[i].timer_id = osTimerNew((void*)trigger_event, osTimerOnce, (void*)ev_timers[i].execute_event, NULL);
  }

  while (true) {
    if (osMessageQueueGet(event_queue, &curr_event, NULL, osWaitForever) == osOK) {
      /* Start Timer if the Config says so */
      for (uint32_t i = 0; i < num_timers; i++) {
        if (curr_event == ev_timers[i].timer_init_event) {
          osTimerStart(ev_timers[i].timer_id, ev_timers[i].timer_duration);
        }
      }
      peripheral_out_t* output_list = event_output_map[curr_event].output_list;
      for (uint32_t i = 0; i < event_output_map[curr_event].num_outputs; ++i) {
        timestamp_t curr_ts = osKernelGetTickCount();
        /* get the actuator function */
        peripheral_out_fp curr_fp = output_list[i].func_ptr;
        if (curr_fp != NULL) {
          log_warn("EXECUTING EVENT: %d, output_idx: %lu", curr_event, i);
          /* call the actuator function */
          curr_fp(output_list[i].func_arg);
          event_info_t event_info = {.ts = curr_ts, .event = curr_event, .output_idx = i};
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
}

osStatus_t trigger_event(cats_event_e ev) {
  log_warn("Event %d added to the queue", ev);
  /* TODO: check if timeout should be 0 here */
  return osMessageQueuePut(event_queue, &ev, 0U, 0U);
}

/** Private Function Definitions **/
