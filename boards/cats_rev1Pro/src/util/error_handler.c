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
#include "flash/recorder.h"
#include "util/log.h"

static uint32_t errors = 0;

void add_error(cats_error_e err) {
  if (err != CATS_ERR_OK) {
    if (errors != (errors | err)) {
      log_error("Encountered error 0x%x", err);

      errors |= err;

      error_info_t error_info = {.error = errors};
      record(osKernelGetTickCount(), ERROR_INFO, &error_info);
    }
  }
}

void clear_error(cats_error_e err) {
  if (errors & err) {
    errors &= ~err;

    error_info_t error_info = {.error = errors};
    record(osKernelGetTickCount(), ERROR_INFO, &error_info);
  }
}

//TODO: use __builtin_popcount(errors)
uint32_t get_error_count() {
  uint32_t count = 0;
  const uint32_t mask = 0x00000001;
  for (int i = 0; i < 32; i++) {
    if (errors & (mask << i)) count++;
  }
  return count;
}

cats_error_e get_error_by_priority(uint32_t id) {
  uint32_t count = 0;
  const uint32_t mask = 0x00000001;
  for (int i = 31; i >= 0; i--) {
    if (errors & (mask << i)) {
      count++;
    }
    if (count == (id + 1)) return (cats_error_e)(mask << i);
  }
  return CATS_ERR_OK;
}

bool get_error_by_tag(cats_error_e err) {
  if ((errors & err) != 0) {
    /* return true if error is present */
    return true;
  } else {
    /* return false if error is not present */
    return false;
  }
}
