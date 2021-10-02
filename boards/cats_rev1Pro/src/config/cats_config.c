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

#include "config/cats_config.h"
#include "util/log.h"
#include "eeprom_emul.h"
#include "util/actions.h"

const cats_config_u DEFAULT_CONFIG = {
    .config.boot_state = CATS_FLIGHT,
    .config.control_settings.main_altitude = 150,
    .config.control_settings.liftoff_acc_threshold = 1500,
    .config.timers[0].duration = 0,
    .config.timers[0].start_event = 0,
    .config.timers[0].end_event = 0,
    .config.timers[1].duration = 0,
    .config.timers[1].start_event = 0,
    .config.timers[1].end_event = 0,
    .config.timers[2].duration = 0,
    .config.timers[2].start_event = 0,
    .config.timers[2].end_event = 0,
    .config.timers[3].duration = 0,
    .config.timers[3].start_event = 0,
    .config.timers[3].end_event = 0,
    .config.action_array[EV_IDLE][0] = ACT_SET_RECORDER_STATE,
    .config.action_array[EV_IDLE][1] = REC_FILL_QUEUE,
    .config.action_array[EV_LIFTOFF][0] = ACT_SET_RECORDER_STATE,
    .config.action_array[EV_LIFTOFF][1] = REC_WRITE_TO_FLASH,
    .config.action_array[EV_TOUCHDOWN][0] = ACT_SET_RECORDER_STATE,
    .config.action_array[EV_TOUCHDOWN][1] = REC_OFF,
    .config.initial_servo_position[0] = 0,
    .config.initial_servo_position[1] = 0,
};

cats_config_u global_cats_config = {};

const uint32_t CATS_STATUS_SECTOR = 1;

/** cats config initialization **/

void cc_init() {
  HAL_FLASH_Unlock();
  EE_Status ee_status = EE_Init(EE_FORCED_ERASE);
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
  osDelay(5);
  HAL_FLASH_Lock();
}

void cc_defaults() { memcpy(&global_cats_config, &DEFAULT_CONFIG, sizeof(global_cats_config)); }

/** persistence functions **/

void cc_load() {
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    EE_ReadVariable32bits(i + 1, &global_cats_config.config_array[i]);
  }
}

bool cc_format_save(){
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  EE_Status ee_status = EE_Format(EE_FORCED_ERASE);
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
  return cc_save();
}

bool cc_save() {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  // loop through all elements of the config
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    uint32_t tmp;
    EE_ReadVariable32bits(i + 1, &tmp);
    // Compare the value from ram to the value from flash
    if (tmp != global_cats_config.config_array[i]) {
      // If different override the value in flash
      EE_Status ee_status;
      int errors = 0;
      do {
        ee_status = EE_WriteVariable32bits(i + 1, global_cats_config.config_array[i]);
        errors++;
      } while((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR && errors < 5);
      if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
      // If writing failed 5 times, stop and return an error
      if (errors == 5) {
        HAL_FLASH_Lock();
        return false;
      }
    }
  }
  HAL_FLASH_Lock();
  return true;
}

/**
 * Returns the number of actions configured for an event
 * @param event -
 * @return number of actions mapped to event
 */
uint16_t cc_get_num_actions(cats_event_e event) {
  uint16_t i = 0;
  uint16_t nr_actions;
  if (event > (NUM_EVENTS-1)) return 0;
  // Count the number of entries
  while((global_cats_config.config.action_array[event][i] != 0) && (i < 16)) {
    i+=2;
  }
  nr_actions = i/2;
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
  if ((action == NULL) || (cc_get_num_actions(event) < (act_idx + 1))) return false;

  int16_t idx = global_cats_config.config.action_array[event][act_idx*2];
  int16_t arg = global_cats_config.config.action_array[event][act_idx*2+1];

  if(idx > 0 && idx <= NUM_ACTION_FUNCTIONS){
    action->action_idx = idx;
    action->arg = arg;
    return true;
  }
  return false;
}
/** debug functions **/

void cc_print() {
  static const char* BOOT_STATE_STRING[] = {"CATS_INVALID", "CATS_IDLE", "CATS_CONFIG",
                                            "CATS_TIMER",   "CATS_DROP", "CATS_FLIGHT"};
  log_info("Boot State: %s", BOOT_STATE_STRING[global_cats_config.config.boot_state]);
}
