/*
 * buzzer_handler.c
 *
 *  Created on: 6 Jan 2021
 *      Author: Luca
 */

#include "util/types.h"
#include "config/globals.h"
#include "util/buzzer_handler.h"

// TODO right now this file only supports errors, should be updated to support
// all buzzer things

const char cats_error_codes[11][BUZZER_COMMAND_MAX_LENGTH] = {
    ".-.-.-",  // Error start
    "-",       // no config
    "--",      // no pyro detected
    "---",     // log full
    "..--",    // usb connected
    ".-",      // battery low
    "..-",     // battery critical
    ".--",     // imu error
    ".---",    // baro error
    ".----",   // filter error
    "....."    // hard fault
};

void error_buzzer(cats_error_e error) {
  static uint32_t timeout = 0;
  static int32_t i = 0;
  static uint8_t stage = 0;
  static uint8_t error_started = 0;
  static cats_error_e static_error = CATS_ERROR_OK;

  // In case of wrong input/corruption throw hard fault error
  if ((error > CATS_ERROR_HARD_FAULT) || (error < 0))
    error = CATS_ERROR_HARD_FAULT;

  // Only load new error state when old one is finished
  if ((error != static_error) && (error_started == 0)) {
    static_error = error;
  }

  if (error) error_started = 1;
  // TODO [Luca] @Nemanja should I use a switch case here just for clarity?
  // (stage 0 and 1, start of error code then error code)
  if ((timeout < osKernelGetTickCount()) && error_started) {
    uint32_t duration;
    if (stage == 0) {  // Beep error start
      if (i < 6) {
        if (cats_error_codes[0][i] == '.')
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
      if (cats_error_codes[static_error][i] &&
          (i < BUZZER_COMMAND_MAX_LENGTH)) {
        if (cats_error_codes[static_error][i] == '.')
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
}
