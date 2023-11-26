/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "cmsis_os.h"
#include "log.h"
#include "target.hpp"

#include <cstdio>

constexpr uint16_t STACK_OVERFLOW_PRINT_BUF_SZ = 100;

#ifdef CATS_DEV
/**
 * This function is called when a stack overflow is detected by FreeRTOS. The CATS implementation of this function just
 * prints the name of the task which generated the overflow via USB (if available) and blinks the red LED.
 *
 * The function will loop forever.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask [[maybe_unused]], char* pcTaskName) {
  while (true) {
    /* Toggle the red LED. */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

    /* Todo: Fix this log_raw, it doesn't work */
    log_raw("Stack overflow detected in %s...\n", pcTaskName);

    /* osDelay doesn't work here because this is an interrupt. */
    HAL_Delay(1000);
  }
}
#endif
