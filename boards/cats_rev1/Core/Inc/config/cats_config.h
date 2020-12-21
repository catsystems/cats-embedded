//
// Created by stoja on 21.12.20.
//

#ifndef CATS_CONFIG_H_
#define CATS_CONFIG_H_

#include "stdint.h"

typedef struct {
  int i1;
  float f1;
  float f2;
  char msg[20];
} cats_config_t;

extern cats_config_t global_cats_config;

void fill_config(float f1, float f2, uint32_t i1, const char *msg);
void load_config();
void save_config();
void print_config();

#endif  // CATS_CONFIG_H_
