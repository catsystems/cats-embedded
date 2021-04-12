/*
 * task_receiver.c
 *
 *  Created on: 10 Apr 2021
 *      Author: Luca
 */

#include "cmsis_os.h"
#include "tasks/task_receiver.h"
#include "drivers/sbus.h"
#include "config/globals.h"

#include <string.h>

#define FAILSAFE_TIME 250

void task_receiver(void *argument) {
  uint32_t tick_count, tick_update;

  static receiver_data_t receiver_data = {0};
  uint8_t armed = 0;
  uint32_t failsafe_timer = 0;

  /* initialize data variables */
  sbus_init();
  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / RECEIVER_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;
    sbus_update(&receiver_data);

    if (!receiver_data.failsafe) {
      failsafe_timer = 0;
    } else if (failsafe_timer < FAILSAFE_TIME) {
      failsafe_timer++;
    }

    if (receiver_data.ch[4] > 2000 && receiver_data.ch[7] > 2000 &&
        !receiver_data.failsafe) {
      dt_telemetry_trigger.set_waiting = 1;
      armed = 1;
    } else if (((receiver_data.ch[4] < 2000 || receiver_data.ch[7] < 2000) &&
                !receiver_data.failsafe) ||
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

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
