/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <arm_math.h>
#include <cstdint>

struct gnss_time_t {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} __attribute__((packed));

struct gnss_position_t {
  float32_t lat;
  float32_t lon;
  uint8_t sats;
} __attribute__((packed));

struct gnss_data_t {
  gnss_position_t position;
  gnss_time_t time;
};
