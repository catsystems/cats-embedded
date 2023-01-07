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

#include "config/cats_config.h"
#include "flash/lfs_custom.h"
#include "lfs.h"
#include "util/actions.h"
#include "util/enum_str_maps.h"
#include "util/error_handler.h"

const cats_config_t DEFAULT_CONFIG = {
    .config_version = CONFIG_VERSION,
    .rec_mask = UINT32_MAX,
    .timers = {},
    .action_array =
        {// EV_MOVING
         {},
         // EV_READY
         {ACT_SET_RECORDER_STATE, REC_FILL_QUEUE},
         // EV_LIFTOFF
         {ACT_SET_RECORDER_STATE, REC_WRITE_TO_FLASH},
         // EV_MAX_V
         {},
         // EV_APOGEE
         {ACT_HIGH_CURRENT_ONE, 1},
         // EV_MAIN_DEPLOYMENT
         {ACT_HIGH_CURRENT_TWO, 1},
         // EV_TOUCHDOWN
         {ACT_SET_RECORDER_STATE, REC_OFF}},
    .initial_servo_position = {0, 0},
    .telemetry_settings =
        {
            .link_phrase = {},
            .power_level = 20,
            .adaptive_power = OFF,
        },
    .control_settings =
        {
            .liftoff_acc_threshold = 35,
            .liftoff_detection_agl = 50,
            .main_altitude = 200,
        },
    .rec_speed_idx = 0,
    /* Assume that when the user starts the board for the first time the default config will be considered theirs. */
    .is_set_by_user = true};

cats_config_t global_cats_config = {};

lfs_file_t config_file;

/** cats config initialization **/

void cc_init() {
  /* Fill lookup_table_speeds with the string representation of the available speeds. The speeds are placed in the array
   * in descending order and are dependent on CONTROL_SAMPLING_FREQ. */
  init_recorder_speed_map();
}

void cc_defaults(bool use_default_outputs, bool set_by_user) {
  memcpy(&global_cats_config, &DEFAULT_CONFIG, sizeof(global_cats_config));
  /* Remove APOGEE & POST_APOGEE actions */
  if (!use_default_outputs) {
    memset(global_cats_config.action_array[EV_APOGEE], 0, sizeof(global_cats_config.action_array[EV_APOGEE]));
    memset(global_cats_config.action_array[EV_MAIN_DEPLOYMENT], 0,
           sizeof(global_cats_config.action_array[EV_MAIN_DEPLOYMENT]));
  }

  global_cats_config.is_set_by_user = set_by_user;
}

/** persistence functions **/

bool cc_load() {
  bool ret = true;
  ret &= lfs_file_open(&lfs, &config_file, "config", LFS_O_RDONLY | LFS_O_CREAT) >= 0;
  ret &=
      lfs_file_read(&lfs, &config_file, &global_cats_config, sizeof(global_cats_config)) == sizeof(global_cats_config);
  ret &= lfs_file_close(&lfs, &config_file) >= 0;

  /* Check if the read config makes sense */
  if (global_cats_config.config_version != CONFIG_VERSION) {
    log_error("Configuration changed or error in config!");
    cc_defaults(true, false);
    ret &= cc_save();
    if (ret) {
      /* If the default config was successfully saved, emit a CATS_ERR_NON_USER_CFG error */
      add_error(CATS_ERR_NON_USER_CFG);
    }
  }

  if (!global_cats_config.is_set_by_user) {
    log_warn("Configuration not set by user!");
  }

  return ret;
}

bool cc_format_save() {
  /* TODO: format lfs and try saving. */
  return cc_save();
}

bool cc_save() {
  lfs_file_open(&lfs, &config_file, "config", LFS_O_WRONLY | LFS_O_CREAT);
  lfs_file_rewind(&lfs, &config_file);
  /* Config saving is successful if the entire global_cats_config struct is written. */
  const bool ret =
      lfs_file_write(&lfs, &config_file, &global_cats_config, sizeof(global_cats_config)) == sizeof(global_cats_config);
  lfs_file_close(&lfs, &config_file);
  if (ret == false) {
    log_error("Error while saving configuration file!");
  }
  return ret;
}

/**
 * Returns the number of actions configured for an event
 * @param event -
 * @return number of actions mapped to event
 */
uint16_t cc_get_num_actions(cats_event_e event) {
  uint16_t i = 0;
  uint16_t nr_actions;
  if (event > (NUM_EVENTS - 1)) return 0;
  // Count the number of entries
  while ((global_cats_config.action_array[event][i] != 0) && (i < 16)) {
    i += 2;
  }
  nr_actions = i / 2;
  return nr_actions;
}

/**
 * Returns the action parameters
 * @param event
 * @param id
 * @param action
 * @return
 */
bool cc_get_action(cats_event_e event, uint16_t act_idx, config_action_t* action) {
  if ((action == nullptr) || (cc_get_num_actions(event) < (act_idx + 1))) return false;

  int16_t idx = global_cats_config.action_array[event][act_idx * 2];
  int16_t arg = global_cats_config.action_array[event][act_idx * 2 + 1];

  if (idx > 0 && idx <= NUM_ACTION_FUNCTIONS) {
    action->action_idx = idx;
    action->arg = arg;
    return true;
  }
  return false;
}
