/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "util/enum_str_maps.hpp"
#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "util/log.h"

#include <cstdio>

const char* const fsm_map[8] = {"INVALID", "MOVING", "READY", "THRUSTING", "COASTING", "DROGUE", "MAIN", "TOUCHDOWN"};

const char* const event_map[9] = {
    "MOVING", "READY", "LIFTOFF", "MAX_V", "APOGEE", "MAIN_DEPLOYMENT", "TOUCHDOWN", "CUSTOM_1", "CUSTOM_2",
};

const char* const action_map[8] = {
    "NONE", "DELAY", "HC_ONE", "HC_TWO", "LL_ONE", "SERVO_ONE", "SERVO_TWO", "RECORDER",
};

const char* const battery_map[3] = {"LI-ION", "LI-PO", "ALKALINE"};

// Filled later depending on tick frequency
char* recorder_speed_map[NUM_REC_SPEEDS] = {};

const char* const on_off_map[2] = {
    "OFF",
    "ON",
};

void init_recorder_speed_map() {
  for (uint32_t i = 0; i < NUM_REC_SPEEDS; ++i) {
    recorder_speed_map[i] = (char*)pvPortMalloc(14 * sizeof(char));
    if (recorder_speed_map[i] == nullptr) {
      log_raw("Could not allocate memory for recorder_speed_map[%lu]!", i);
      return;
    }

    memset(recorder_speed_map[i], 0, 14 * sizeof(char));
    snprintf(recorder_speed_map[i], 14, "%.4gHz", (double)CONTROL_SAMPLING_FREQ / (i + 1));
  }
}
