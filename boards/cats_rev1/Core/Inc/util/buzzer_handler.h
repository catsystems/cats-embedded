/*
 * buzzer_handler.h
 *
 *  Created on: 6 Jan 2021
 *      Author: Luca
 */

#pragma once

#define BUZZER_COMMAND_MAX_LENGTH 9

extern const char cats_error_codes[11][BUZZER_COMMAND_MAX_LENGTH];

#define BUZZER_SHORT_BEEP  100
#define BUZZER_LONG_BEEP   400
#define BUZZER_SHORT_PAUSE 200
#define BUZZER_LONG_PAUSE  1000

void buzzer_handler_update();
bool buzzer_queue_status(buzzer_status_e status);
