//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "drivers/w25qxx.h"
#include "util.h"
#include <string.h>

typedef struct {
  int i1;
  float f1;
  float f2;
  char msg[20];
} cats_config_t;

static cats_config_t global_cats_config = {0};

void cc_init(float f1, float f2, uint32_t i1, const char *msg) {
  global_cats_config.f1 = f1;
  global_cats_config.f2 = f2;
  global_cats_config.i1 = i1;
  /* TODO: change this string copying if needed later */
  memset(global_cats_config.msg, '\0', sizeof(global_cats_config.msg));
  if (msg != NULL) {
    strncpy(global_cats_config.msg, msg, sizeof(global_cats_config.msg));
  }
}

void cc_clear() { memset(&global_cats_config, 0, sizeof(global_cats_config)); }

/* accessor functions */
float cc_get_f1() { return global_cats_config.f1; }
void cc_set_f1(float f1) { global_cats_config.f1 = f1; }

float cc_get_f2() { return global_cats_config.f2; }
void cc_set_f2(float f2) { global_cats_config.f2 = f2; }

uint32_t cc_get_i1() { return global_cats_config.i1; }
void cc_set_i1(uint32_t i1) { global_cats_config.i1 = i1; }

const char *cc_get_msg() { return global_cats_config.msg; }
void cc_set_msg(const char *msg) {
  memset(global_cats_config.msg, '\0', sizeof(global_cats_config.msg));
  if (msg != NULL) {
    strncpy(global_cats_config.msg, msg, sizeof(global_cats_config.msg));
  }
}

void cc_load() {
  /* TODO: global_cats_config can't be larger than sector size */
  W25qxx_ReadSector((uint8_t *)(&global_cats_config), 0, 0,
                    sizeof(global_cats_config));
}

void cc_save() {
  /* erase sector before writing to it */
  W25qxx_EraseSector(0);
  /* TODO: global_cats_config can't be larger than sector size */
  W25qxx_WriteSector((uint8_t *)(&global_cats_config), 0, 0,
                     sizeof(global_cats_config));
}

void cc_print() {
  UsbPrint("Config: f1: %f, f2: %f, i1: %d, msg: %s\n", global_cats_config.f1,
           global_cats_config.f2, global_cats_config.i1,
           global_cats_config.msg);
}