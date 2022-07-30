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

#include "tasks.h"

#include "config/cats_config.h"
#include "util/log.h"
#include "util/task_util.h"

#include "tasks/task_flight_fsm.h"
#include "tasks/task_health_monitor.h"
#include "tasks/task_peripherals.h"
#include "tasks/task_preprocessing.h"
#include "tasks/task_recorder.h"
#include "tasks/task_sensor_read.h"
#include "tasks/task_state_est.h"
#ifdef CATS_VEGA
#include "tasks/task_telemetry.h"
#endif
/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_sensor_read, 512)
/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_preprocessing, 512)

/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_state_est, 512)
SET_TASK_PARAMS(task_health_monitor, 256)

/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_flight_fsm, 512)
SET_TASK_PARAMS(task_peripherals, 256)
/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_recorder, 1024)

#ifdef CATS_VEGA
SET_TASK_PARAMS(task_telemetry, 512)
#endif

void init_tasks() {
  switch (global_cats_config.config.boot_state) {
    case CATS_FLIGHT: {
#if (configUSE_TRACE_FACILITY == 1)
      baro_channel = xTraceRegisterString("Baro Channel");
      flash_channel = xTraceRegisterString("Flash Channel");
#endif

      // TODO: Check rec_queue for validity here
      rec_queue = osMessageQueueNew(REC_QUEUE_SIZE, sizeof(rec_elem_t), NULL);
      rec_cmd_queue = osMessageQueueNew(REC_CMD_QUEUE_SIZE, sizeof(rec_cmd_type_e), NULL);
      event_queue = osMessageQueueNew(EVENT_QUEUE_SIZE, sizeof(cats_event_e), NULL);
#if (configUSE_TRACE_FACILITY == 1)
      vTraceSetQueueName(rec_queue, "Recorder Queue");
#endif

      osThreadNew(task_recorder, NULL, &task_recorder_attributes);

      osThreadNew(task_sensor_read, NULL, &task_sensor_read_attributes);

      osThreadNew(task_preprocessing, NULL, &task_preprocessing_attributes);

      osThreadNew(task_flight_fsm, NULL, &task_flight_fsm_attributes);

      osThreadNew(task_peripherals, NULL, &task_peripherals_attributes);

      osThreadNew(task_state_est, NULL, &task_state_est_attributes);

      osThreadNew(task_health_monitor, NULL, &task_health_monitor_attributes);
#ifdef CATS_VEGA
      osThreadNew(task_telemetry, NULL, &task_telemetry_attributes);
#endif

    } break;
    case CATS_CONFIG:
      break;
    case CATS_TIMER:
    case CATS_DROP:
      break;
    default:
      log_fatal("Wrong boot state!");
  }
}