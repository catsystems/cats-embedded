/*
 * actions.c
 *
 *  Created on: 12 Apr 2021
 *      Author: Luca
 */

#include "util/actions.h"
#include "config/cats_config.h"

#include "drivers/servo.h"

bool no_action_function(int16_t bummer);

bool os_delay(int16_t centiseconds);

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

const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS] = {no_action_function,         os_delay,
                                                              high_current_channel_one,   high_current_channel_two,
                                                              high_current_channel_three, high_current_channel_four,
                                                              high_current_channel_five,  high_current_channel_six,
                                                              low_level_channel_one,      low_level_channel_two,
                                                              low_level_channel_three,    low_level_channel_four,
                                                              servo_channel_one,          servo_channel_two,
                                                              servo_channel_three,        servo_channel_four,
                                                              set_recorder_state};

bool no_action_function(int16_t bummer) {
  // Sucks to be here...
  // it seems like someone didn't configure the actions right
  return false;
}

/* Be careful when setting these delays, right now the peripheral
 * task will sleep until this delay is finished, meaning that it
 * won't read from the queue until the delay is done. If you set too
 * long of a delay you might execute the next event too late, even
 * if the event came at the right time.
 *
 * Right now this is considered a feature and not a bug since we
 * assume the users know what they are doing when setting up these
 * delays. */
bool os_delay(int16_t centiseconds) {
  if (centiseconds > 0) {
    /* convert to milliseconds */
    osDelay(centiseconds * 10);
    return true;
  }
  return false;
}

// High current outputs for pyros, valves etc.

bool high_current_channel_one(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_1_GPIO_Port, PYRO_1_Pin, state);
    return true;
  }
  return false;
}

bool high_current_channel_two(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_2_GPIO_Port, PYRO_2_Pin, state);
    return true;
  }
  return false;
}

bool high_current_channel_three(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  }
  return false;
}

bool high_current_channel_four(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  }
  return false;
}

bool high_current_channel_five(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  }
  return false;
}

bool high_current_channel_six(int16_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin, state);
    return true;
  }
  return false;
}

// Low level (3.3V) outputs

bool low_level_channel_one(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(GPIO_1_GPIO_Port, GPIO_1_Pin, state);
    return true;
  }
  return false;
}

bool low_level_channel_two(int16_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(GPIO_2_GPIO_Port, GPIO_2_Pin, state);
    return true;
  }
  return false;
}

// Same as servo 1 but digital output
bool low_level_channel_three(int16_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO1, state);
    return true;
  }
  return false;
}

// Same as servo 2 but digital output
bool low_level_channel_four(int16_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO2, state);
    return true;
  }
  return false;
}

// Servo Outputs

bool servo_channel_one(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

bool servo_channel_two(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO2, angle);
    return true;
  }
  return false;
}

bool servo_channel_three(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

bool servo_channel_four(int16_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

/* TODO check if mutex should be used here */
bool set_recorder_state(int16_t state) {
  recorder_status_e rec_status = (recorder_status_e)state;
  /* TODO: add a boundary value for rec_status_e enum -> REC_END = 0xfff..*/

  if (rec_status >= REC_OFF && rec_status <= REC_WRITE_TO_FLASH) {
    if (state == REC_WRITE_TO_FLASH) {
      cs_set_num_recorded_flights(cs_get_num_recorded_flights() + 1);
      cs_save();
    }
    global_recorder_status = rec_status;

    return true;
  }
  return false;
}
