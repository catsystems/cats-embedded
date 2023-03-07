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

#include "cmsis_os.h"
#include "log.h"
#include "target.h"

#include <cstdio>

#define STACK_OVERFLOW_PRINT_BUF_SZ 100

#ifdef CATS_DEBUG
/**
 * This function is called when a stack overflow is detected by FreeRTOS. The CATS implementation of this function just
 * prints the name of the task which generated the overflow via USB (if available) and blinks the red LED.
 *
 * The function will loop forever.
 */
void vApplicationStackOverflowHook(TaskHandle_t task_handle, char *task_name) {
  while (true) {
    /* Toggle the red LED. */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

    /* Todo: Fix this log_raw, it doesn't work */
    log_raw("Stack overflow detected in %s...\n", task_name);

    /* osDelay doesn't work here because this is an interrupt. */
    HAL_Delay(1000);
  }
}
#endif
