/*
 * outputs.c
 *
 *  Created on: 12 Apr 2021
 *      Author: Luca
 */

#include "util/outputs.h"

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

bool set_recorder_state(int16_t state);

const peripheral_out_fp output_table[NUM_OUTPUT_FUNCTIONS] = {
    no_output_function,        high_current_channel_one,
    high_current_channel_two,  high_current_channel_three,
    high_current_channel_four, high_current_channel_five,
    high_current_channel_six,  low_level_channel_one,
    low_level_channel_two,     low_level_channel_three,
    low_level_channel_four,    servo_channel_one,
    servo_channel_two,         servo_channel_three,
    servo_channel_four,        set_recorder_state};

bool no_output_function(int16_t bummer) {
  // Sucks to be here...
  // it seems like someone didn't configure the outputs right
  return 0;
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

// Same as servo 1 but digital output
bool low_level_channel_three(int16_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO1, state);
    return true;
  } else
    return false;
}

// Same as servo 2 but digital output
bool low_level_channel_four(int16_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO2, state);
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

/* TODO check if mutex should be used here */
bool set_recorder_state(int16_t state) {
  recorder_status_e rec_status = (recorder_status_e)state;
  /* TODO: add a boundary value for rec_status_e enum -> REC_END = 0xfff..*/
  if (rec_status >= REC_OFF && rec_status <= REC_WRITE_TO_FLASH) {
    global_recorder_status = rec_status;
    return true;
  }
  return false;
}