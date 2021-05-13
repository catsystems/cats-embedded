/*
 * actions.h
 *
 *  Created on: Apr 12, 2021
 *      Author: Luca
 */

#pragma once

#include "util/types.h"

#define NUM_ACTION_FUNCTIONS 17

extern const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS];

typedef enum {
  ACT_NO_OP = 0,
  ACT_OS_DELAY,
  ACT_HIGH_CURRENT_ONE,
  ACT_HIGH_CURRENT_TWO,
  ACT_HIGH_CURRENT_THREE,
  ACT_HIGH_CURRENT_FOUR,
  ACT_HIGH_CURRENT_FIVE,
  ACT_HIGH_CURRENT_SIX,
  ACT_LOW_LEVEL_ONE,
  ACT_LOW_LEVEL_TWO,
  ACT_LOW_LEVEL_THREE,
  ACT_LOW_LEVEL_FOUR,
  ACT_SERVO_ONE,
  ACT_SERVO_TWO,
  ACT_SERVO_THREE,
  ACT_SERVO_FOUR,
  ACT_SET_RECORDER_STATE,
} action_function_list_e;
