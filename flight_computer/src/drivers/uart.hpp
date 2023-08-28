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
