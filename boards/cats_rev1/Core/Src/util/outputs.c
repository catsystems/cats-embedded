/*
 * outputs.c
 *
 *  Created on: 12 Apr 2021
 *      Author: Luca
 */

#include "util/outputs.h"
#include "config/globals.h"
#include "drivers/servo.h"

bool no_output_function(int16_t bummer);
bool high_current_channel_one(int16_t state);
bool high_current_channel_two(int16_t state);
bool high_current_channel_three(int16_t state);
bool high_current_channel_four(int16_t state);  // reserved for later use
bool high_current_channel_five(int16_t state);  // reserved for later use
bool high_current_channel_six(int16_t state);   // reserved for later use

bool low_level_channel_one(int16_t state);
bool low_level_channel_two(int16_t state);
bool low_level_channel_three(int16_t state);  // reserved for later use
bool low_level_channel_four(int16_t state);   // reserved for later use

bool servo_channel_one(int16_t angle);
bool servo_channel_two(int16_t angle);
bool servo_channel_three(int16_t angle);  // reserved for later use
bool servo_channel_four(int16_t angle);   // reserved for later use

const bool (*output_table[15])(int16_t) = {
    no_output_function,        high_current_channel_one,
    high_current_channel_two,  high_current_channel_three,
    high_current_channel_four, high_current_channel_five,
    high_current_channel_six,  low_level_channel_one,
    low_level_channel_two,     low_level_channel_three,
    low_level_channel_four,    servo_channel_one,
    servo_channel_two,         servo_channel_three,
    servo_channel_four,
};

bool no_output_function(int16_t bummer) {
  // Sucks to be here...
  // it seems like someone didn't configure the outputs right
}

// High current outputs for pyros, valves etc.

bool high_current_channel_one(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_1_GPIO_Port, PYRO_1_Pin, state);
    return true;
  } else
    return false;
}

bool high_current_channel_two(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_2_GPIO_Port, PYRO_2_Pin, state);
    return true;
  } else
    return false;
}

bool high_current_channel_three(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  } else
    return false;
}

bool high_current_channel_four(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  } else
    return false;
}

bool high_current_channel_five(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  } else
    return false;
}

bool high_current_channel_six(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  } else
    return false;
}

// Low level (3.3V) outputs

bool low_level_channel_one(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, state);
    return true;
  } else
    return false;
}

bool low_level_channel_two(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(GPIO_2_GPIO_Port, GPIO_2_Pin, state);
    return true;
  } else
    return false;
}

bool low_level_channel_three(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, state);
    return true;
  } else
    return false;
}

bool low_level_channel_four(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, state);
    return true;
  } else
    return false;
}

// Servo Outputs

bool servo_channel_one(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO1, angle);
    return true;
  } else
    return false;
}

bool servo_channel_two(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO2, angle);
    return true;
  } else
    return false;
}

bool servo_channel_three(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  } else
    return false;
}

bool servo_channel_four(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  } else
    return false;
}
