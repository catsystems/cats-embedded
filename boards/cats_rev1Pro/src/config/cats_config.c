//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "drivers/w25q.h"
#include "util/log.h"
#include "drivers/eeprom_emul.h"

typedef struct {
  /* Last sector where task_recorder wrote the data; The next sector will be
   * first checked if it's empty and if so, the next flight recorder
   * log will be recorded starting from that sector */
  uint16_t last_recorded_sector;
  /* Total number of currently logged flights on the flash chip. This value
   * represents the length of the array stored in sector #1 that holds the
   * starting sectors of separate flight logs. */
  uint16_t num_recorded_flights;
  /* TODO: don't create a static array here */
  uint16_t last_sectors_of_flight_recordings[32];

  flight_fsm_e last_fsm_state[32];
  float max_altitude[32];
  float max_velocity[32];
  float max_acceleration[32];
} cats_status_t;

const cats_config_u DEFAULT_CONFIG = {
    .config.boot_state = CATS_FLIGHT,
    .config.control_settings.main_altitude = 150,
    .config.control_settings.liftoff_acc_threshold = 1500,
    .config.timers[0].duration = 1000,
    .config.timers[0].start_event = 0,
    .config.timers[0].end_event = 0,
    .config.timers[1].duration = 1000,
    .config.timers[1].start_event = 0,
    .config.timers[1].end_event = 0,
    .config.timers[2].duration = 1000,
    .config.timers[2].start_event = 0,
    .config.timers[2].end_event = 0,
    .config.timers[3].duration = 1000,
    .config.timers[3].start_event = 0,
    .config.timers[3].end_event = 0,
};

cats_config_u global_cats_config = {};
cats_status_t global_cats_status = {};

const uint32_t CATS_STATUS_SECTOR = 1;

/** cats config initialization **/

void cc_init() {
  HAL_FLASH_Unlock();
  EE_Status ee_status = EE_Init(EE_FORCED_ERASE);
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
}

void cc_defaults() { memcpy(&global_cats_config, &DEFAULT_CONFIG, sizeof(global_cats_config)); }

/** persistence functions **/

void cc_load() {
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    EE_ReadVariable32bits(i + 1, &global_cats_config.config_array[i]);
  }
}

void cc_save() {
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    uint32_t tmp;
    EE_ReadVariable32bits(i + 1, &tmp);
    if (tmp != global_cats_config.config_array[i]) {
      EE_Status ee_status = EE_WriteVariable32bits(i + 1, global_cats_config.config_array[i]);
      if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
    }
  }
}

/**
 * Returns the number of actions configured for an event
 * @param event -
 * @return number of actions mapped to event
 */
int16_t cc_get_action_number(cats_event_e event) {
  int i = 0;
  bool found = false;
  int16_t nr_actions;

  do {
    // Loop until we find the event or until we run out of config to read from
    cats_event_e tmp = global_cats_config.config.action_array[i];

    // Check for event, stop immediately when config is empty
    if (tmp > 0) {
      if (tmp == event) {
        found = true;
      }
    } else
      return 0;

    // Read the number of actions from the array
    nr_actions = global_cats_config.config.action_array[i + 1];
    // Increment the action array pointer to the next action, return when number is 0
    if (nr_actions > 0)
      i = i + (2 * nr_actions) + 2;
    else
      return 0;
  } while (i < 128 && found == false);

  return nr_actions;
}

/**
 * Returns the action parameters
 * @param event
 * @param id
 * @param action
 * @return
 */
bool cc_get_action(cats_event_e event, int16_t id, config_action_t* action) {
  if (cc_get_action_number(event) < (id + 1)) return false;
  int i = 0;
  do {
    // Loop until we find the event
    if (global_cats_config.config.action_array[i] == event) {
      action->action_pointer = global_cats_config.config.action_array[i + (2 * (id + 1))];
      action->arg = global_cats_config.config.action_array[i + (2 * (id + 1)) + 1];
      return true;
    }

    // Read the number of actions from the array
    int nr_actions = global_cats_config.config.action_array[i + 1];

    // Increment the action array pointer to the next action, return when number is 0
    i += (2 * nr_actions) + 2;

  } while (i < 128);
  return false;
}
/** debug functions **/

void cc_print() {
  static const char* BOOT_STATE_STRING[] = {"CATS_INVALID", "CATS_IDLE", "CATS_CONFIG",
                                            "CATS_TIMER",   "CATS_DROP", "CATS_FLIGHT"};
  log_info("Boot State: %s", BOOT_STATE_STRING[global_cats_config.config.boot_state]);
}