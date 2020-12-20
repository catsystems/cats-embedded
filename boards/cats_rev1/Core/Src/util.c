/*
 * util.c
 *
 *  Created on: Feb 24, 2020
 *      Author: stoja
 */

#include "util.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/** USB DEBUGGING SECTION **/

#ifdef DEBUG
osMutexId_t print_mutex;
char print_buffer[PRINT_BUFFER_LEN];
#endif

uint8_t UsbPrint(const char *format, ...) {
  uint8_t ret = 1;
#ifdef DEBUG
  // TODO Add mutex!
  // if (osMutexAcquire(print_mutex, 0U) == osOK) {
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
  va_end(argptr);
  ret = CDC_Transmit_FS((uint8_t *)print_buffer, strlen(print_buffer));
  osMutexRelease(print_mutex);
  osDelay(2);
  //}
#endif
  return ret;
}
