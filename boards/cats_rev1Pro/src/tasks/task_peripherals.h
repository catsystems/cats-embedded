/*
 * task_peripherals.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#pragma once

#include "config/globals.h"
#include "cmsis_os.h"

extern const uint32_t EVENT_QUEUE_SIZE;

_Noreturn void task_peripherals(void *argument);

osStatus_t trigger_event(cats_event_e ev);
