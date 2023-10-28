/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

// clang-format off
 enum cats_error_e: uint32_t {
  CATS_ERR_OK =             0,
  CATS_ERR_NON_USER_CFG =   1U << 0U,   // 0x01
  CATS_ERR_NO_PYRO =        1U << 1U,   // 0x02
  CATS_ERR_LOG_FULL =       1U << 2U,   // 0x04
  CATS_ERR_BAT_LOW =        1U << 3U,   // 0x08
  CATS_ERR_BAT_CRITICAL =   1U << 4U,   // 0x10
  CATS_ERR_IMU_0 =          1U << 5U,   // 0x20
  CATS_ERR_IMU_1 =          1U << 6U,   // 0x40
  CATS_ERR_IMU_2 =          1U << 7U,   // 0x80
  CATS_ERR_BARO_0 =         1U << 8U,   // 0x100
  CATS_ERR_BARO_1 =         1U << 9U,   // 0x200
  CATS_ERR_BARO_2 =         1U << 10U,   // 0x400
  CATS_ERR_MAG =            1U << 11U,   // 0x800
  CATS_ERR_ACC =            1U << 12U,   // 0x1000
  CATS_ERR_FILTER_ACC =     1U << 13U,   // 0x2000
  CATS_ERR_FILTER_HEIGHT =  1U << 14U,   // 0x4000
  CATS_ERR_HARD_FAULT =     1U << 15U,   // 0x8000
  CATS_ERR_TELEMETRY_HOT =  1U << 16U,   // 0x10000
  CATS_ERR_CALIB =          1U << 17U,   // 0x20000
};
// clang-format on

void add_error(cats_error_e err);
void clear_error(cats_error_e err);
uint32_t get_error_count();
cats_error_e get_error_by_priority(uint32_t id);
/* Returns true if the error is present */
bool get_error_by_tag(cats_error_e err);
