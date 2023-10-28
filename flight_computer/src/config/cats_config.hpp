/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "target.hpp"
#include "util/types.hpp"

/* The system will reload the default config when the number changes */
constexpr uint32_t CONFIG_VERSION = 212U;

/* Number of supported recording speeds */
constexpr uint8_t NUM_REC_SPEEDS = 10;

struct cats_config_t {
  /* Needs to be in first position */
  uint32_t config_version{0};

  /* A bit mask that specifies which readings to log to the flash */
  uint32_t rec_mask{0};

  // Timers
  config_timer_t timers[NUM_TIMERS]{};
  // Event action map
  int16_t action_array[NUM_EVENTS][16]{};  // 8 (16/2) actions for each event
  int16_t initial_servo_position[2]{};

  config_telemetry_t telemetry_settings{};
  control_settings_t control_settings{};
  uint8_t buzzer_volume{0};
  battery_type_e battery_type{LI_ION};
  uint8_t rec_speed_idx{0};  // == inverse recording rate - 1
  /* Testing Mode */
  bool enable_testing_mode{false};
  bool is_set_by_user{false};
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern cats_config_t global_cats_config;

/** cats config initialization **/
void cc_init();
void cc_defaults(bool use_default_outputs, bool set_by_user);

/** persistence functions - return true on success **/
bool cc_load();
bool cc_save();
bool cc_format_save();

/** action map functions **/
uint16_t cc_get_num_actions(cats_event_e event);
bool cc_get_action(cats_event_e event, uint16_t act_idx, config_action_t* action);
