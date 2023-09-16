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

#include <array>
#include <span>
#include <type_traits>

#include "config/cats_config.hpp"

using EnumToStrMap = std::span<const char *const>;

/**
 * @brief Prints the string mapping of a given enum value.
 *
 * @tparam T - enum type
 * @param enum_val - enum value
 * @param map - enum to string map
 * @return string value of the given enumt, prints "Unknown" if the passed enum value is out of bounds.
 */
template <typename T>
constexpr const char *GetStr(T enum_val, EnumToStrMap map) {
  static_assert(std::is_enum<T>::value, "Must be an enum!");
  const auto map_idx = std::underlying_type_t<T>(enum_val);
  if (map_idx < 0 || map_idx >= map.size()) {
    return "Unknown";
  }
  return map[map_idx];
}

inline constexpr std::array<const char *, 8> fsm_map{"INVALID",  "CALIBRATING", "READY", "THRUSTING",
                                                     "COASTING", "DROGUE",      "MAIN",  "TOUCHDOWN"};

inline constexpr std::array<const char *, NUM_EVENTS> event_map{
    "CALIBRATE", "READY", "LIFTOFF", "MAX_V", "APOGEE", "MAIN_DEPLOYMENT", "TOUCHDOWN", "CUSTOM_1", "CUSTOM_2",
};

inline constexpr std::array<const char *, 8> action_map{
    "NONE", "DELAY", "HC_ONE", "HC_TWO", "LL_ONE", "SERVO_ONE", "SERVO_TWO", "RECORDER",
};

inline constexpr std::array<const char *, 2> on_off_map{
    "OFF",
    "ON",
};

inline constexpr std::array<const char *, 3> battery_map{"LI-ION", "LI-PO", "ALKALINE"};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern std::array<char *, NUM_REC_SPEEDS> recorder_speed_map;

void init_recorder_speed_map();
