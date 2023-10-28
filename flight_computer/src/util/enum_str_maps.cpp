/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "util/enum_str_maps.hpp"
#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "util/log.h"

#include <array>
#include <cstdio>

// Filled later depending on tick frequency
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::array<char*, NUM_REC_SPEEDS> recorder_speed_map = {};

void init_recorder_speed_map() {
  for (uint32_t i = 0; i < NUM_REC_SPEEDS; ++i) {
    recorder_speed_map[i] = static_cast<char*>(pvPortMalloc(14 * sizeof(char)));
    if (recorder_speed_map[i] == nullptr) {
      log_raw("Could not allocate memory for recorder_speed_map[%lu]!", i);
      return;
    }

    memset(recorder_speed_map[i], 0, 14 * sizeof(char));
    snprintf(recorder_speed_map[i], 14, "%.4gHz",
             static_cast<double>(CONTROL_SAMPLING_FREQ) / static_cast<double>(i + 1));
  }
}
