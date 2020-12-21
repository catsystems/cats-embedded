//
// Created by stoja on 21.12.20.
//

#ifndef CATS_CONFIG_H_
#define CATS_CONFIG_H_

#include <stdint.h>

/* cats config initialization */
void cc_init(float f1, float f2, uint32_t i1, const char *msg);
void cc_clear();

/* accessor functions */
float cc_get_f1();
void cc_set_f1(float f1);

float cc_get_f2();
void cc_set_f2(float f2);

uint32_t cc_get_i1();
void cc_set_i1(uint32_t i1);

const char *cc_get_msg();
void cc_set_msg(const char *msg);

/* persistence functions */
void cc_load();
void cc_save();

/* debug functions */
void cc_print();

#endif  // CATS_CONFIG_H_
