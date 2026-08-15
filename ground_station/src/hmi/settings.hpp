/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "config.hpp"
#include "utils.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

enum settings_type_e {
  STRING = 0,
  TOGGLE = 1,
  NUMBER = 2,
  BUTTON = 3,
};

enum settings_button_action_e {
  BUTTON_ACTION_NONE = 0,
  BUTTON_ACTION_USB_STORAGE,
  BUTTON_ACTION_START_BOOTLOADER,
};

struct settings_min_max_t {
  int16_t min;
  int16_t max;
};

union settings_limits_u {
  uint32_t stringLength;
  uint32_t lookup;
  settings_min_max_t minmax;
  settings_button_action_e buttonAction;
};

struct device_settings_t {
  const char* name;
  const char* description1;
  const char* description2;
  settings_type_e type;
  settings_limits_u config;

  void* dataPtr;
};

enum lookup_table_index_e {
  TABLE_MODE = 0,
  TABLE_UNIT,
  TABLE_LOGGING,
  TABLE_STARTUP_ANIMATION,
};

const char* const mode_map[2] = {
    "SINGLE",
    "DUAL",
};

const char* const unit_map[2] = {
    "METRIC",
    "IMPERIAL",
};

const char* const logging_map[2] = {
    "DOWN",
    "NEVER",
};

const char* const startup_animation_map[2] = {
    "OFF",
    "ON",
};

struct lookup_table_entry_t {
  const char* const* values;
  const uint8_t value_count;
};

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookup_table_entry_t lookup_tables[] = {
    LOOKUP_TABLE_ENTRY(mode_map),
    LOOKUP_TABLE_ENTRY(unit_map),
    LOOKUP_TABLE_ENTRY(logging_map),
    LOOKUP_TABLE_ENTRY(startup_animation_map),
};

enum {
  kSettingPages = 3,
};

const char* const settingPageName[kSettingPages] = {"General", "Telemetry", "Other"};

extern const device_settings_t settingsTable[][4];

const uint16_t settingsTableValueCount[kSettingPages] = {4, 4, 3};
