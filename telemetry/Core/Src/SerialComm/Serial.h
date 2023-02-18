#pragma once

#include "stm32g0xx_hal.h"

void start_serial();

template <uint32_t N> class Serial {
public:
  Serial(UART_HandleTypeDef *handle) : tail(0U), uart(handle), buffer{}{
    HAL_UART_Receive_DMA(uart, buffer, N);
  }

  uint32_t available() {
    uint32_t head = N - uart->hdmarx->Instance->CNDTR;
    if (tail > head)
      return (head + N) - tail;
    else
      return head - tail;
  }

  uint8_t read() {
    uint8_t tmp = buffer[tail];
    ++tail;
    tail = tail % N;
    return tmp;
  }

private:
  uint32_t tail;
  UART_HandleTypeDef *uart;
  uint8_t buffer[N];
};
