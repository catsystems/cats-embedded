//
// Created by stoja on 21.12.20.
//

#ifndef CATS_CONFIG_H_
#define CATS_CONFIG_H_

#include <stdint.h>

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

/** cats config initialization **/
void cc_init(uint16_t last_recorded_sector, uint16_t num_recorded_flights,
             cats_boot_state boot_state);
void cc_clear();

/** accessor functions **/

uint16_t cc_get_last_recorded_sector();
void cc_set_last_recorded_sector(uint16_t last_recorded_sector);

uint16_t cc_get_num_recorded_flights();
void cc_set_num_recorded_flights(uint16_t last_recorded_sector);

cats_boot_state cc_get_boot_state();
void cc_set_boot_state(cats_boot_state boot_state);

/** persistence functions **/
void cc_load();
void cc_save();

/** debug functions **/
void cc_print();

#endif  // CATS_CONFIG_H_
