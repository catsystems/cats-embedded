/*
 * task_health_monitor.c
 *
 *  Created on: 28 Apr 2021
 *      Author: Luca
 */

#include "cmsis_os.h"
#include "tasks/task_health_monitor.h"
#include "config/globals.h"
#include "util/battery.h"
#include "util/buzzer_handler.h"
#include "config/cats_config.h"

/** Private Constants **/

/** Private Function Declarations **/

/** Exported Function Definitions **/

_Noreturn void task_health_monitor(__attribute__((unused)) void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  uint32_t alive_timer = 0;
  flight_fsm_e old_fsm_state = MOVING;

  while (1) {
    battery_level_e level = battery_level();
    if (level == BATTERY_CRIT)
      ;  // TODO add the error
    else if (level == BATTERY_LOW)
      ;  // TODO add the error
    else
      ;  // TODO clear error

    if (global_flight_state.flight_state == IDLE && alive_timer >= 600) {
      buzzer_queue_status(CATS_BUZZ_READY);
      alive_timer = 0;
    } else if (global_flight_state.flight_state == IDLE) {
      alive_timer++;
    }

    if (global_flight_state.flight_state == IDLE && (global_flight_state.flight_state != old_fsm_state))
      buzzer_queue_status(CATS_BUZZ_CHANGED_READY);
    if (global_flight_state.flight_state == MOVING && (global_flight_state.flight_state != old_fsm_state))
      buzzer_queue_status(CATS_BUZZ_CHANGED_MOVING);

    buzzer_handler_update();
    old_fsm_state = global_flight_state.flight_state;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
