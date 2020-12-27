//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "drivers/w25qxx.h"
#include "util/log.h"
#include <string.h>

typedef struct {
  /* Last sector where task_recorder wrote the data; The next sector will be
   * first checked if it's empty and if so, the next flight recorder
   * log will be recorded starting from that sector */
  uint16_t last_recorded_sector;
} cats_config_t;

cats_config_t global_cats_config = {0};

/** cats config initialization **/

void cc_init(uint16_t last_recorded_sector) {
  global_cats_config.last_recorded_sector = last_recorded_sector;
}

void cc_clear() { memset(&global_cats_config, 0, sizeof(global_cats_config)); }

/** accessor functions **/

uint16_t cc_get_last_recorded_sector() {
  return global_cats_config.last_recorded_sector;
}
void cc_set_last_recorded_sector(uint16_t last_recorded_sector) {
  global_cats_config.last_recorded_sector = last_recorded_sector;
}

/** persistence functions **/

void cc_load() {
  /* TODO: global_cats_config can't be larger than sector size */
  w25qxx_read_sector((uint8_t *)(&global_cats_config), 0, 0,
                     sizeof(global_cats_config));
}

void cc_save() {
  /* erase sector before writing to it */
  w25qxx_erase_sector(0);
  /* TODO: global_cats_config can't be larger than sector size */
  w25qxx_write_sector((uint8_t *)(&global_cats_config), 0, 0,
                      sizeof(global_cats_config));
}

/** debug functions **/

void cc_print() {
  log_info("Config: Last recorded sector: %u",
           global_cats_config.last_recorded_sector);
}