//
// Created by stoja on 13.05.21.
//

#pragma once

#include <stdint.h>

typedef enum {
  CATS_ERR_OK = 0,
  CATS_ERR_NO_CONFIG = 0x01,
  CATS_ERR_NO_PYRO = 0x02,
  CATS_ERR_LOG_FULL = 0x04,
  CATS_ERR_USB_CONNECTED = 0x08,
  CATS_ERR_BAT_LOW = 0x10,
  CATS_ERR_BAT_CRITICAL = 0x20,
  CATS_ERR_IMU = 0x40,
  CATS_ERR_BARO = 0x80,
  CATS_ERR_FILTER = 0x100,
  CATS_ERR_HARD_FAULT = 0x200
} cats_error_e;

void error_handler(cats_error_e err);