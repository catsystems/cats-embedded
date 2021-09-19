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
  static uint8_t print_buffer[256];
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

    uint32_t len = fifo_get_length(&usb_output_fifo);
    if (len) {
      fifo_read_bytes(&usb_output_fifo, print_buffer, len);
      CDC_Transmit_FS(print_buffer, len);
    }

    buzzer_handler_update();
    old_fsm_state = global_flight_state.flight_state;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
