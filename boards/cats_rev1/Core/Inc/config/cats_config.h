//
// Created by stoja on 21.12.20.
//

#ifndef CATS_CONFIG_H_
#define CATS_CONFIG_H_

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

extern const uint32_t CATS_CONFIG_SECTOR;
extern const uint32_t CATS_STATUS_SECTOR;

/** cats config initialization **/
void cc_init(cats_boot_state boot_state, bool clear_flash,
             uint32_t recorder_mask);
void cc_clear();

/** accessor functions **/

cats_boot_state cc_get_boot_state();
void cc_set_boot_state(cats_boot_state boot_state);

bool cc_get_clear_flash();
void cc_set_clear_flash(bool clear_flash);

/* Control Settings accessor */
control_settings_t cc_get_control_settings();
void cc_set_apogee_timer(float apogee_timer);
void cc_set_second_stage_timer(float second_stage_timer);
void cc_set_liftoff_acc_threshold(float liftoff_acc_threshold);
float cc_get_apogee_timer();
float cc_get_second_stage_timer();
float cc_get_liftoff_acc_threshold();

uint32_t cc_get_recorder_mask();
void cc_set_recorder_mask(uint32_t recorder_mask);

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

#endif  // CATS_CONFIG_H_
