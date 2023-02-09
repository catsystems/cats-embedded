/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "util/actions.h"

#include "config/cats_config.h"
#include "config/globals.h"
#include "drivers/servo.h"
#include "flash/lfs_custom.h"
#include "target.h"

extern driver::Servo* global_servo1;
extern driver::Servo* global_servo2;

bool no_action_function(__attribute__((unused)) int16_t bummer);

bool os_delay(int16_t ticks);

bool high_current_channel_one(int16_t state);
bool high_current_channel_two(int16_t state);

bool low_level_channel_one(int16_t state);

bool servo_channel_one(int16_t angle);
bool servo_channel_two(int16_t angle);

const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS] = {
    no_action_function,    os_delay,          high_current_channel_one, high_current_channel_two,
    low_level_channel_one, servo_channel_one, servo_channel_two,        set_recorder_state};

bool no_action_function(__attribute__((unused)) int16_t bummer) {
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
bool os_delay(int16_t ticks) {
  if (ticks > 0) {
    osDelay(static_cast<uint32_t>(ticks));
    return true;
  }
  return false;
}

// High current outputs for pyros, valves etc.
bool high_current_channel_one(int16_t state) {
#if NUM_PYRO > 0
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO1_GPIO_Port, PYRO1_Pin, (GPIO_PinState)state);
    return true;
  }
#endif
  return false;
}

bool high_current_channel_two(int16_t state) {
#if NUM_PYRO > 1
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(PYRO2_GPIO_Port, PYRO2_Pin, (GPIO_PinState)state);
    return true;
  }
#endif
  return false;
}

// Low level (3.3V) outputs

bool low_level_channel_one(int16_t state) {
#if NUM_LOW_LEVEL_IO > 0
  if (state == 0 || state == 1) {
    HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, (GPIO_PinState)state);
    return true;
  }
#endif
  return false;
}

// Servo Outputs

bool servo_channel_one(int16_t position) {
  if (global_servo1 == nullptr) {
    return false;
  }
  if (position >= 0 && position <= 1000) {
    global_servo1->SetPosition(position);
    return true;
  }
  return false;
}

bool servo_channel_two(int16_t position) {
  if (global_servo2 == nullptr) {
    return false;
  }
  if (position >= 0 && position <= 1000) {
    global_servo2->SetPosition(position);
    return true;
  }
  return false;
}

/* TODO check if mutex should be used here */
bool set_recorder_state(int16_t state) {
  volatile recorder_status_e new_rec_state = (recorder_status_e)state;
  if (new_rec_state < REC_OFF || new_rec_state > REC_WRITE_TO_FLASH) {
    return false;
  }

  rec_cmd_type_e rec_cmd = REC_CMD_INVALID;

  log_info("Changing recorder state from %u to %u", global_recorder_status, new_rec_state);
  switch (global_recorder_status) {
    case REC_OFF:
      if (new_rec_state == REC_FILL_QUEUE) {
        rec_cmd = REC_CMD_FILL_Q;
      } else if (new_rec_state == REC_WRITE_TO_FLASH) {
        rec_cmd = REC_CMD_WRITE;
      }
      break;
    case REC_FILL_QUEUE:
      if (new_rec_state == REC_OFF) {
        rec_cmd = REC_CMD_FILL_Q_STOP;
      } else if (new_rec_state == REC_WRITE_TO_FLASH) {
        rec_cmd = REC_CMD_WRITE;
      }
      break;
    case REC_WRITE_TO_FLASH:
      if ((new_rec_state == REC_OFF) || (new_rec_state == REC_FILL_QUEUE)) {
        rec_cmd = REC_CMD_WRITE_STOP;
        osStatus_t ret = osMessageQueuePut(rec_cmd_queue, &rec_cmd, 0U, 10U);
        if (ret != osOK) {
          log_error("Inserting an element to the recorder command queue failed! Error: %d", ret);
        }
        rec_cmd = REC_CMD_INVALID;
        if (new_rec_state == REC_FILL_QUEUE) {
          rec_cmd = REC_CMD_FILL_Q;
        }
      }
      break;
    default:
      log_error("Invalid global recorder status!");
      return false;
  }

  global_recorder_status = new_rec_state;

  if (rec_cmd != REC_CMD_INVALID) {
    osStatus_t ret = osMessageQueuePut(rec_cmd_queue, &rec_cmd, 0U, 10U);
    if (ret != osOK) {
      log_error("Inserting an element to the recorder command queue failed! Error: %d", ret);
    }
  }

  return true;
}
