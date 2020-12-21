//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "drivers/w25qxx.h"
#include "util.h"
#include <string.h>

cats_config_t global_cats_config = {0};

void fill_config(float f1, float f2, uint32_t i1, const char *msg) {
  global_cats_config.f1 = f1;
  global_cats_config.f2 = f2;
  global_cats_config.i1 = i1;
  memset(global_cats_config.msg, '\0', sizeof(global_cats_config.msg));
  if (msg != NULL) {
    strncpy(global_cats_config.msg, msg, sizeof(global_cats_config.msg));
  }
}

void load_config() {
  /* TODO: global_cats_config can't be larger than sector size */
  W25qxx_ReadSector((uint8_t *)(&global_cats_config), 0, 0,
                    sizeof(global_cats_config));
}

void save_config() {
  /* erase sector before writing to it */
  W25qxx_EraseSector(0);
  /* TODO: global_cats_config can't be larger than sector size */
  W25qxx_WriteSector((uint8_t *)(&global_cats_config), 0, 0,
                     sizeof(global_cats_config));
}

void print_config() {
  UsbPrint("Config: f1: %f, f2: %f, i1: %d, msg: %s\n", global_cats_config.f1,
           global_cats_config.f2, global_cats_config.i1,
           global_cats_config.msg);
}