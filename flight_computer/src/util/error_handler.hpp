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

#pragma once

#include <cstdint>

// clang-format off
 enum cats_error_e: uint32_t {
  CATS_ERR_OK =             0,
  CATS_ERR_NON_USER_CFG =   1 << 0,   // 0x01
  CATS_ERR_NO_PYRO =        1 << 1,   // 0x02
  CATS_ERR_LOG_FULL =       1 << 2,   // 0x04
  CATS_ERR_BAT_LOW =        1 << 3,   // 0x08
  CATS_ERR_BAT_CRITICAL =   1 << 4,   // 0x10
  CATS_ERR_IMU_0 =          1 << 5,   // 0x20
  CATS_ERR_IMU_1 =          1 << 6,   // 0x40
  CATS_ERR_IMU_2 =          1 << 7,   // 0x80
  CATS_ERR_BARO_0 =         1 << 8,   // 0x100
  CATS_ERR_BARO_1 =         1 << 9,   // 0x200
  CATS_ERR_BARO_2 =         1 << 10,   // 0x400
  CATS_ERR_MAG =            1 << 11,   // 0x800
  CATS_ERR_ACC =            1 << 12,   // 0x1000
  CATS_ERR_FILTER_ACC =     1 << 13,   // 0x2000
  CATS_ERR_FILTER_HEIGHT =  1 << 14,   // 0x4000
  CATS_ERR_HARD_FAULT =     1 << 15,   // 0x8000
  CATS_ERR_TELEMETRY_HOT =  1 << 16,   // 0x10000
};
// clang-format on

void add_error(cats_error_e err);
void clear_error(cats_error_e err);
uint32_t get_error_count();
cats_error_e get_error_by_priority(uint32_t id);
/* Returns true if the error is present */
bool get_error_by_tag(cats_error_e err);
