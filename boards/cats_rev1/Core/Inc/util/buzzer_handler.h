/*
 * buzzer_handler.h
 *
 *  Created on: 6 Jan 2021
 *      Author: Luca
 */

#ifndef INC_UTIL_BUZZER_HANDLER_H_
#define INC_UTIL_BUZZER_HANDLER_H_

#define BUZZER_COMMAND_MAX_LENGTH 7

static const char cats_error_codes[11][BUZZER_COMMAND_MAX_LENGTH] = {
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

#define BUZZER_SHORT_BEEP  100
#define BUZZER_LONG_BEEP   400
#define BUZZER_SHORT_PAUSE 300
#define BUZZER_LONG_PAUSE  1000

void error_buzzer(cats_error_e error);

#endif /* INC_UTIL_BUZZER_HANDLER_H_ */
