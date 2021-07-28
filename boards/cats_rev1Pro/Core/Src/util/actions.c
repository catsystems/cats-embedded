/*
 * actions.c
 *
 *  Created on: 12 Apr 2021
 *      Author: Luca
 */

#include "util/actions.h"

#include "config/cats_config.h"
#include "config/globals.h"
#include "drivers/servo.h"

bool no_action_function(__attribute__((unused)) int32_t bummer);

bool os_delay(int32_t ticks);

bool high_current_channel_one(int32_t state);
bool high_current_channel_two(int32_t state);
bool high_current_channel_three(int32_t state);
bool high_current_channel_four(int32_t state);  // reserved for later use
bool high_current_channel_five(int32_t state);  // reserved for later use
bool high_current_channel_six(int32_t state);   // reserved for later use

bool low_level_channel_one(int32_t state);
bool low_level_channel_two(int32_t state);
bool low_level_channel_three(int32_t state);  // reserved for later use
bool low_level_channel_four(int32_t state);   // reserved for later use

bool servo_channel_one(int32_t angle);
bool servo_channel_two(int32_t angle);
bool servo_channel_three(int32_t angle);  // reserved for later use
bool servo_channel_four(int32_t angle);   // reserved for later use


const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS] = {no_action_function,         os_delay,
                                                              high_current_channel_one,   high_current_channel_two,
                                                              high_current_channel_three, high_current_channel_four,
                                                              high_current_channel_five,  high_current_channel_six,
                                                              low_level_channel_one,      low_level_channel_two,
                                                              low_level_channel_three,    low_level_channel_four,
                                                              servo_channel_one,          servo_channel_two,
                                                              servo_channel_three,        servo_channel_four,
                                                              set_recorder_state};

bool no_action_function(__attribute__((unused)) int32_t bummer) {
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
bool os_delay(int32_t ticks) {
  if (ticks > 0) {
    osDelay((uint32_t)ticks);
    return true;
  }
  return false;
}

// High current outputs for pyros, valves etc.

bool high_current_channel_one(int32_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool high_current_channel_two(int32_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool high_current_channel_three(int32_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO3_GPIO_Port, PYRO3_Pin, (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool high_current_channel_four(int32_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin,  (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool high_current_channel_five(int32_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin,  (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool high_current_channel_six(int32_t state) {
  if (state == 0 || state == 1) {
    // HAL_GPIO_WritePin(PYRO_3_GPIO_Port, PYRO_3_Pin,  (GPIO_PinState)state);
    return true;
  }
  return false;
}

// Low level (3.3V) outputs

bool low_level_channel_one(int32_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, (GPIO_PinState)state);
    return true;
  }
  return false;
}

bool low_level_channel_two(int32_t state) {
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, (GPIO_PinState)state);
    return true;
  }
  return false;
}

// Same as servo 1 but digital output
bool low_level_channel_three(int32_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO1, state);
    return true;
  }
  return false;
}

// Same as servo 2 but digital output
bool low_level_channel_four(int32_t state) {
  if (state == 0 || state == 1) {
    servo_set_onoff(&SERVO2, (bool)state);
    return true;
  }
  return false;
}

// Servo Outputs

bool servo_channel_one(int32_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

bool servo_channel_two(int32_t angle) {
  if (angle >= 0 && angle <= 180) {
    servo_set_position(&SERVO2, angle);
    return true;
  }
  return false;
}

bool servo_channel_three(int32_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

bool servo_channel_four(int32_t angle) {
  if (angle >= 0 && angle <= 180) {
    // servo_set_position(&SERVO1, angle);
    return true;
  }
  return false;
}

/* TODO check if mutex should be used here */
bool set_recorder_state(int32_t state) {
  volatile recorder_status_e rec_status = (recorder_status_e)state;
  /* TODO: add a boundary value for rec_status_e enum -> REC_END = 0xfff..*/
  // TODO: max flights of 32
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
