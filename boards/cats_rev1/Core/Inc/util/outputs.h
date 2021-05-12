/*
 * outputs.h
 *
 *  Created on: Apr 12, 2021
 *      Author: Luca
 */

#pragma once

#include "config/globals.h"

#define NUM_OUTPUT_FUNCTIONS 16

extern const peripheral_out_fp output_table[NUM_OUTPUT_FUNCTIONS];

typedef enum {
  OUT_NO_OP = 0,
  OUT_HIGH_CURRENT_ONE,
  OUT_HIGH_CURRENT_TWO,
  OUT_HIGH_CURRENT_THREE,
  OUT_HIGH_CURRENT_FOUR,
  OUT_HIGH_CURRENT_FIVE,
  OUT_HIGH_CURRENT_SIX,
  OUT_LOW_LEVEL_ONE,
  OUT_LOW_LEVEL_TWO,
  OUT_LOW_LEVEL_THREE,
  OUT_LOW_LEVEL_FOUR,
  OUT_SERVO_ONE,
  OUT_SERVO_TWO,
  OUT_SERVO_THREE,
  OUT_SERVO_FOUR,
  RECORDER_STATE,
} output_function_list_e;
