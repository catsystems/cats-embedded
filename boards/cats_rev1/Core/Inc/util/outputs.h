/*
 * outputs.h
 *
 *  Created on: Apr 12, 2021
 *      Author: Luca
 */

#ifndef INC_UTIL_OUTPUTS_H_
#define INC_UTIL_OUTPUTS_H_

#include "config/globals.h"

extern const bool (*output_table[15])(int16_t);

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
} output_function_list;

#endif /* INC_UTIL_OUTPUTS_H_ */
