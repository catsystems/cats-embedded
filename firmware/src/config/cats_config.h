/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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
#include "target.h"
#include "util/types.h"

/* The system will reload the default config when the number changes */
/* Config version 2 / Minor 4 */
#define CONFIG_VERSION 204

/* Number supported recording speeds */
#define NUM_REC_SPEEDS 10

struct cats_config_t {
  /* Needs to be in first position */
  uint32_t config_version;

  control_settings_t control_settings;
  /* A bit mask that specifies which readings to log to the flash */
  uint32_t rec_mask;

  // Timers
  config_timer_t timers[NUM_TIMERS];
  // Event action map
  int16_t action_array[NUM_EVENTS][16];  // 8 (16/2) actions for each event
  int16_t initial_servo_position[2];
  uint8_t rec_speed_idx;  // == inverse recording rate - 1

  config_telemetry_t telemetry_settings;
};

union cats_config_u {
  cats_config_t config;
  uint32_t config_array[sizeof(cats_config_t) / sizeof(uint32_t)];
};

extern cats_config_u global_cats_config;

/** cats config initialization **/
void cc_init();
void cc_defaults(bool use_default_outputs);

/** persistence functions - return true on success **/
bool cc_load();
bool cc_save();
bool cc_format_save();

/** action map functions **/
uint16_t cc_get_num_actions(cats_event_e event);
bool cc_get_action(cats_event_e event, uint16_t act_idx, config_action_t* action);
