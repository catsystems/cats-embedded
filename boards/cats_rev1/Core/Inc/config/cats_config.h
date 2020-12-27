//
// Created by stoja on 21.12.20.
//

#ifndef CATS_CONFIG_H_
#define CATS_CONFIG_H_

#include <stdint.h>

/** cats config initialization **/
void cc_init(uint16_t last_recorded_sector);
void cc_clear();

/** accessor functions **/

uint16_t cc_get_last_recorded_sector();
void cc_set_last_recorded_sector(uint16_t last_recorded_sector);

/** persistence functions **/
void cc_load();
void cc_save();

/** debug functions **/
void cc_print();

#endif  // CATS_CONFIG_H_
