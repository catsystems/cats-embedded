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

  //Timers
  config_timer_t timers[4];
  // Event action map
  config_event_actions_t event_actions[9];
} cats_config_t;

typedef union {
  cats_config_t config;
  uint32_t config_array[sizeof(cats_config_t)/sizeof(uint32_t)];
} cats_config_u;

extern cats_config_u global_cats_config;

extern const uint32_t CATS_CONFIG_SECTOR;
extern const uint32_t CATS_STATUS_SECTOR;

/** cats config initialization **/
void cc_init();
void cc_defaults();

/** persistence functions **/
void cc_load();
void cc_save();

/** debug functions **/
void cc_print();

/** cats state initialization **/
void cs_init(uint16_t last_recorded_sector, uint16_t num_recorded_flights);
void cs_clear();

/** accessor functions **/

uint16_t cs_get_last_recorded_sector();
void cs_set_last_recorded_sector(uint16_t last_recorded_sector);

uint16_t cs_get_num_recorded_flights();
void cs_set_num_recorded_flights(uint16_t last_recorded_sector);

uint16_t cs_get_last_sector_of_flight(uint16_t flight_idx);

void cs_set_max_altitude(float altitude);
void cs_set_max_acceleration(float acceleration);
void cs_set_max_velocity(float velocity);
void cs_set_flight_phase(flight_fsm_e state);

float cs_get_max_acceleration(uint32_t flight);
float cs_get_max_altitude(uint32_t flight);
float cs_get_max_velocity(uint32_t flight);

/** persistence functions **/
void cs_load();
void cs_save();

/** debug functions **/
void cs_print();

