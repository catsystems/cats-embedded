//
// Created by stoja on 21.12.20.
//

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "util/types.h"

/* Exported types */

typedef enum {
  CATS_INVALID,
  CATS_IDLE,
  CATS_CONFIG,
  CATS_TIMER,
  CATS_DROP,
  CATS_FLIGHT,
  CATS_HEHE = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
} cats_boot_state;

typedef struct {
  /* State according to /concepts/v1/cats_fsm.jpg */
  cats_boot_state boot_state;

  control_settings_t control_settings;
  /* A bit mask that specifies which readings to log to the flash */
  uint32_t recorder_mask;

  // Timers
  config_timer_t timers[8];
  // Event action map
  config_event_actions_t event_actions[9];
} cats_config_t;

typedef union {
  cats_config_t config;
  uint32_t config_array[sizeof(cats_config_t) / sizeof(uint32_t)];
} cats_config_u;

extern cats_config_u global_cats_config;

extern const uint32_t CATS_STATUS_SECTOR;

/** cats config initialization **/
void cc_init();
void cc_defaults();

/** persistence functions **/
void cc_load();
void cc_save();

/** debug functions **/
void cc_print();