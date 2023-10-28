/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "uart.hpp"
#include "target.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static uint16_t instance_count = 0;

static UART_BUS *buses[MAX_INSTANCES];
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

void uart_init(UART_BUS *bus) {
  buses[instance_count] = bus;
  instance_count++;
}

uint8_t uart_transmit_receive(UART_BUS *bus, uint8_t *tx_buf, uint16_t tx_size, uint8_t *rx_buf, uint16_t rx_size) {
  if (bus->busy) {
    return 0;
  }
  HAL_UART_Transmit(bus->uart_handle, tx_buf, tx_size, UART_TIMEOUT);
  HAL_UART_Receive(bus->uart_handle, rx_buf, rx_size, UART_TIMEOUT);
  return 1;
}

uint8_t uart_transmit(UART_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) {
    return 0;
  }
  return HAL_UART_Transmit(&huart2, tx_buf, tx_size, UART_TIMEOUT);
}

uint8_t uart_receive(UART_BUS *bus, uint8_t *rx_buf, uint16_t rx_size) {
  if (bus->busy) {
    return 0;
  }
  return HAL_UART_Receive(&huart2, rx_buf, rx_size, UART_TIMEOUT);
}
