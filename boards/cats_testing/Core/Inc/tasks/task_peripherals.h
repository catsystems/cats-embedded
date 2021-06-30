/*
 * task_peripherals.h
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#ifndef INC_TASKS_TASK_PERIPHERALS_H_
#define INC_TASKS_TASK_PERIPHERALS_H_

#include "config/globals.h"
#include "cmsis_os.h"

extern const uint32_t EVENT_QUEUE_SIZE;

void task_peripherals(void *argument);

osStatus_t trigger_event(cats_event_e ev);

#endif /* INC_TASKS_TASK_PERIPHERALS_H_ */
