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

#include "util/error_handler.h"
#include "util/recorder.h"
#include "util/log.h"

#include <stdint.h>

void error_handler(cats_error_e err) {
  /* TODO: see if it makes sense to log from which task the error came from */
  // osThreadId_t task_id = osThreadGetId();
  if (err != CATS_ERR_OK) {
    log_error("Encountered error 0x%x", err);
    // uint32_t ts = osKernelGetTickCount();
    //    error_info_t error_info = {.ts = ts, .error = err};
    //    record(ERROR_INFO, &error_info);
  }
}

