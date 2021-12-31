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
#include "tasks/task_receiver.h"
#include "drivers/sbus.h"
#include "config/globals.h"

#include <string.h>

#define FAILSAFE_TIME 250

_Noreturn void task_receiver(__attribute__((unused)) void *argument) {
  static receiver_data_t receiver_data = {0};
  uint8_t armed = 0;
  uint32_t failsafe_timer = 0;

  /* initialize data variables */
  sbus_init();

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / RECEIVER_SAMPLING_FREQ;
  while (1) {
    sbus_update(&receiver_data);

    if (!receiver_data.failsafe) {
      failsafe_timer = 0;
    } else if (failsafe_timer < FAILSAFE_TIME) {
      failsafe_timer++;
    }

    if (receiver_data.ch[4] > 2000 && receiver_data.ch[7] > 2000 && !receiver_data.failsafe) {
      dt_telemetry_trigger.set_waiting = 1;
      armed = 1;
    } else if (((receiver_data.ch[4] < 2000 || receiver_data.ch[7] < 2000) && !receiver_data.failsafe) ||
               (receiver_data.failsafe && failsafe_timer >= FAILSAFE_TIME)) {
      dt_telemetry_trigger.set_waiting = 0;
      armed = 0;
    }

    if (receiver_data.ch[5] > 2000 && !receiver_data.failsafe && armed) {
      dt_telemetry_trigger.set_drogue = 1;
    }

    if (receiver_data.ch[6] > 2000 && !receiver_data.failsafe && armed) {
      dt_telemetry_trigger.set_main = 1;
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
