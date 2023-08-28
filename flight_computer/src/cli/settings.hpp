/*
 * This file was part of Cleanflight and Betaflight.
 * https://github.com/betaflight/betaflight
 * It is modified for the CATS Flight Software.
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

#include "config/cats_config.hpp"
#include "util/enum_str_maps.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

enum lookup_table_index_e { TABLE_EVENTS = 0, TABLE_ACTIONS, TABLE_POWER, TABLE_SPEEDS, TABLE_BATTERY };

inline constexpr uint8_t VALUE_TYPE_OFFSET = 0;
inline constexpr uint8_t VALUE_SECTION_OFFSET = 3;
inline constexpr uint8_t VALUE_MODE_OFFSET = 5;

enum cli_value_flag_e {
  // value type, bits 0-2
  VAR_UINT8 = (0U << VALUE_TYPE_OFFSET),
  VAR_INT8 = (1U << VALUE_TYPE_OFFSET),
  VAR_UINT16 = (2U << VALUE_TYPE_OFFSET),
  VAR_INT16 = (3U << VALUE_TYPE_OFFSET),
  VAR_UINT32 = (4U << VALUE_TYPE_OFFSET),

  // value mode, bits 5-7
  MODE_DIRECT = (0U << VALUE_MODE_OFFSET),
  MODE_LOOKUP = (1U << VALUE_MODE_OFFSET),
  MODE_ARRAY = (2U << VALUE_MODE_OFFSET),
  MODE_BITSET = (3U << VALUE_MODE_OFFSET),
  MODE_STRING = (4U << VALUE_MODE_OFFSET),
};

inline constexpr uint32_t VALUE_TYPE_MASK = 0x07;
inline constexpr uint32_t VALUE_SECTION_MASK = 0x18;
inline constexpr uint32_t VALUE_MODE_MASK = 0xE0;

struct cli_minmax_config_t {
  const int16_t min;
  const int16_t max;
};

struct cli_minmax_unsigned_config_t {
  const uint16_t min;
  const uint16_t max;
};

struct cli_lookup_table_config_t {
  const lookup_table_index_e table_index;
};

struct cli_array_length_config_t {
  const uint8_t length;
};

struct cli_string_length_config_t {
  const uint8_t min_length;
  const uint8_t max_length;
  const uint8_t flags;
};

inline constexpr uint32_t STRING_FLAGS_NONE = 0;
inline constexpr uint32_t STRING_FLAGS_WRITEONCE = 1U << 0U;

union cli_value_config_u {
  cli_lookup_table_config_t lookup;              // used for MODE_LOOKUP excl. VAR_UINT32
  cli_minmax_config_t minmax;                    // used for MODE_DIRECT with signed parameters
  cli_minmax_unsigned_config_t minmax_unsigned;  // used for MODE_DIRECT with unsigned parameters
  cli_array_length_config_t array;               // used for MODE_ARRAY
  cli_string_length_config_t string;             // used for MODE_STRING
  uint8_t bitpos;                                // used for MODE_BITSET
  uint32_t u32_max;                              // used for MODE_DIRECT with VAR_UINT32
};

struct cli_value_t;
using callback_f = void (*)(const struct cli_value_t *);

struct cli_value_t {
  const char *name;
  const uint8_t type;  // see cli_value_flag_e
  const cli_value_config_u config;
  uint16_t member_offset;
  callback_f cb;
} __attribute__((packed));

inline constexpr std::array<EnumToStrMap, 5> lookup_tables{event_map, action_map, on_off_map, recorder_speed_map,
                                                           battery_map};

extern const uint16_t value_table_entry_count;

extern const cli_value_t value_table[];

void *get_cats_config_member_ptr(const cats_config_t *cfg, const cli_value_t *var);

void print_cats_config(const char *cmd_name, const cats_config_t *cfg, bool print_limits);
