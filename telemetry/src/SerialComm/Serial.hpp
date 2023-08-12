/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include "stm32g0xx_hal.h"

void start_serial();

template <uint32_t N>
class Serial {
 public:
  explicit Serial(UART_HandleTypeDef *handle) : tail(0U), uart(handle), buffer{} {
    HAL_UART_Receive_DMA(uart, buffer, N);
  }

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
  volatile uint32_t tail;
  UART_HandleTypeDef *uart;
  uint8_t buffer[N];
};
