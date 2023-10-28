/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "stm32f4xx.h"

#include <cstdint>

struct UART_BUS {
  UART_HandleTypeDef* const uart_handle;
  uint8_t initialized;
  bool busy;
};

uint8_t uart_transmit_receive(UART_BUS* bus, uint8_t* tx_buf, uint16_t tx_size, uint8_t* rx_buf, uint16_t rx_size);
uint8_t uart_transmit(UART_BUS* bus, uint8_t* tx_buf, uint16_t tx_size);
uint8_t uart_receive(UART_BUS* bus, uint8_t* rx_buf, uint16_t rx_size);
void uart_init(UART_BUS* bus);

inline constexpr uint8_t MAX_INSTANCES = 10;
inline constexpr uint8_t UART_TIMEOUT = 5;
