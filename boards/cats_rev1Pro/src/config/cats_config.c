//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "util/log.h"
#include "drivers/eeprom_emul.h"

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

/** debug functions **/

void cc_print() {
  static const char *BOOT_STATE_STRING[] = {"CATS_INVALID", "CATS_IDLE", "CATS_CONFIG",
                                            "CATS_TIMER",   "CATS_DROP", "CATS_FLIGHT"};
  log_info("Boot State: %s", BOOT_STATE_STRING[global_cats_config.config.boot_state]);
}