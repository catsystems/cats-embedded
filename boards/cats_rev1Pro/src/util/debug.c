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

#include "cmsis_os.h"
#include "usbd_cdc_if.h"

#include "util/debug.h"

#include <stdio.h>

#define STACK_OVERFLOW_PRINT_BUF_SZ 100

extern USBD_HandleTypeDef hUsbDeviceFS;

#ifdef CATS_DEBUG
/**
 * This function is called when a stack overflow is detected by FreeRTOS. The CATS implementation of this function just
 * prints the name of the task which generated the overflow via USB (if available) and blinks the red LED.
 *
 * The function will loop forever.
 */
_Noreturn void vApplicationStackOverflowHook(TaskHandle_t task_handle, char *task_name) {
  static char print_buf[STACK_OVERFLOW_PRINT_BUF_SZ] = {};
  uint32_t msg_len = snprintf(print_buf, sizeof(print_buf), "Stack overflow detected in %s...\n", task_name);

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  while (1) {
    /* Toggle the red LED. */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

    if (hcdc != NULL) {
      /* Force the TxState to 0 because someone might have been writing to the USB before, but we don't care anymore. */
      hcdc->TxState = 0;
      CDC_Transmit_FS((uint8_t *)print_buf, msg_len);
    }

    /* osDelay doesn't work here because this is an interrupt. */
    HAL_Delay(1000);
  }
}
#endif
