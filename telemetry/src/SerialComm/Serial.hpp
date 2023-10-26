/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "stm32g0xx_hal.h"

void start_serial();

template <uint32_t N>
class Serial {
 public:
  explicit Serial(UART_HandleTypeDef *handle) : uart(handle) { HAL_UART_Receive_DMA(uart, buffer, N); }

  uint32_t available() {
    volatile uint32_t head = N - uart->hdmarx->Instance->CNDTR;
    if (tail > head) {
      return (head + N) - tail;
    }
    return head - tail;
  }

  uint8_t read() {
    uint8_t tmp = buffer[tail];
    ++tail;
    tail = tail % N;
    return tmp;
  }

 private:
  volatile uint32_t tail{0};
  UART_HandleTypeDef *uart;
  uint8_t buffer[N]{};
};
