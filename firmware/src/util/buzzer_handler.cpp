/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "util/buzzer_handler.h"

#include "config/globals.h"
#include "control/data_processing.h"
#include "util/error_handler.h"
#include "util/types.h"

// TODO right now this file only supports errors, should be updated to support all buzzer things

#define BUZZER_COMMAND_MAX_LENGTH 9

#define BUZZER_SHORT_BEEP  100
#define BUZZER_LONG_BEEP   400
#define BUZZER_SHORT_PAUSE 200
#define BUZZER_LONG_PAUSE  1000

#define BUZZER_MAX_STATUS_QUEUE 5

uint8_t status_buzzer(buzzer_status_e status);

// Never buzz bellow 1000Hz and above 3400Hz!
const uint32_t pitch_lookup[8] = {
    2349,  // D A
    2489,  // D# B
    2637,  // E C
    2793,  // F D
    2959,  // F# E
    3135,  // G F
    1200,  // Error G
    2217,  // C#  H
};

const char cats_error_codes[16][BUZZER_COMMAND_MAX_LENGTH] = {
    "gGgGgG",  // Error start
    "G",       // no config
    "GG",      // no pyro detected
    "GGG",     // log full
    "ggGG",    // usb connected
    "gG",      // battery low
    "ggG",     // battery critical
    "gGG",     // imu0 error
    "gGG",     // imu1 error
    "gGG",     // imu2 error
    "gGGG",    // baro0 error
    "gGGG",    // baro1 error
    "gGGG",    // baro2 error
    "gGGGG",   // filter error
    "ggggg"    // hard fault
};

const char cats_status_codes[5][BUZZER_COMMAND_MAX_LENGTH] = {
    " ",
    "caef",  // bootup
    "aa",    // ready
    "Eca",   // ready -> moving
    "ace",   // moving -> ready
};

static uint32_t status_queue_index = 0;
static uint32_t status_queue_elements = 0;
static buzzer_status_e status_queue[BUZZER_MAX_STATUS_QUEUE];
uint8_t error_buzzer(cats_error_e error);

bool buzzer_queue_status(buzzer_status_e status) {
  if (status_queue_elements < BUZZER_MAX_STATUS_QUEUE) {
    status_queue[(status_queue_index + status_queue_elements) % 5] = status;
    ++status_queue_elements;
    return true;
  }
  return false;
}

void buzzer_handler_update() {
  if (status_queue_elements > 0) {
    if (status_buzzer(status_queue[status_queue_index]) == 0) {
      status_queue_index = (status_queue_index + 1) % BUZZER_MAX_STATUS_QUEUE;
      status_queue_elements--;
    }
  }
  // else if(get_error_count()>0 || error_started == 1){
  //   error_started = error_buzzer(log2_32(get_error_by_priority(0)));
  // }
  buzzer_update(&BUZZER);
}

uint8_t status_buzzer(buzzer_status_e status) {
  static uint32_t timeout = 0;
  static int32_t i = 0;
  static uint8_t started = 0;
  static buzzer_status_e static_status = CATS_BUZZ_NONE;

  // In case of wrong input return
  if ((status > CATS_BUZZ_CHANGED_READY) || (status < 0)) return 0;

  // Only load new status state when old one is finished
  if ((status != static_status) && (started == 0)) {
    static_status = status;
  }

  if (status) {
    started = 1;
  }

  if ((timeout < osKernelGetTickCount()) && started) {
    uint32_t duration = 0;

    if (cats_status_codes[static_status][i] && (i < BUZZER_COMMAND_MAX_LENGTH)) {
      char pitch = cats_status_codes[static_status][i];
      if (pitch >= 'A' && pitch <= 'H') {
        buzzer_set_freq(&BUZZER, pitch_lookup[pitch - 'A']);
        duration = BUZZER_LONG_BEEP;
      } else if (pitch >= 'a' && pitch <= 'h') {
        buzzer_set_freq(&BUZZER, pitch_lookup[pitch - 'a']);
        duration = BUZZER_SHORT_BEEP;
      }
      buzzer_beep(&BUZZER, duration);
      timeout = osKernelGetTickCount() + duration + BUZZER_SHORT_PAUSE;
      i++;
    } else {
      timeout = osKernelGetTickCount() + BUZZER_LONG_PAUSE;
      i = 0;
      started = 0;
    }
  }
  return started;
}

uint8_t error_buzzer(cats_error_e error) {
  static uint32_t timeout = 0;
  static int32_t i = 0;
  static uint8_t stage = 0;
  static uint8_t error_started = 0;
  static cats_error_e static_error = CATS_ERR_OK;

  // In case of wrong input/corruption throw hard fault error
  if ((error > CATS_ERR_HARD_FAULT) || (error < 0)) error = CATS_ERR_HARD_FAULT;

  // Only load new error state when old one is finished
  if ((error != static_error) && (error_started == 0)) {
    static_error = error;
  }

  if (error) error_started = 1;

  if ((timeout < osKernelGetTickCount()) && error_started) {
    buzzer_set_freq(&BUZZER, pitch_lookup[6]);
    uint32_t duration = 0;
    if (stage == 0) {  // Beep error start
      if (i < 6) {
        if (cats_error_codes[0][i] == 'g')
          duration = BUZZER_SHORT_BEEP;
        else
          duration = BUZZER_LONG_BEEP;
        buzzer_beep(&BUZZER, duration);
        timeout = osKernelGetTickCount() + duration + BUZZER_SHORT_PAUSE;
        i++;
      } else {
        timeout = osKernelGetTickCount() + BUZZER_LONG_PAUSE;
        stage = 1;
        i = 0;
      }
    } else {  // Beep error code
      if (cats_error_codes[static_error][i] && (i < BUZZER_COMMAND_MAX_LENGTH)) {
        if (cats_error_codes[static_error][i] == 'g')
          duration = BUZZER_SHORT_BEEP;
        else
          duration = BUZZER_LONG_BEEP;
        buzzer_beep(&BUZZER, duration);
        timeout = osKernelGetTickCount() + duration + BUZZER_SHORT_PAUSE;
        i++;
      } else {
        timeout = osKernelGetTickCount() + BUZZER_LONG_PAUSE;
        stage = 0;
        i = 0;
        error_started = 0;
      }
    }
  }
  return error_started;
}
