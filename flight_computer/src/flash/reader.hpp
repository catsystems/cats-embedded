/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "cmsis_os.h"
#include "recorder.hpp"
#include "util/types.hpp"

namespace reader {

void dump_recording(uint16_t flight_num);
void parse_recording(uint16_t flight_num, rec_entry_type_e filter_mask);

void print_stats_and_cfg(uint16_t flight_num);

}  // namespace reader
