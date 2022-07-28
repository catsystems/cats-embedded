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

#include "uart.h"
#include "target.h"

static uint16_t instance_count = 0;

static UART_BUS *buses[MAX_INSTANCES];

void uart_init(UART_BUS *bus) {
  buses[instance_count] = bus;
  instance_count++;
}

 uint8_t uart_transmit_receive(UART_BUS *bus, uint8_t *tx_buf, uint16_t tx_size, uint8_t *rx_buf,
                                    uint16_t rx_size) {
  if (bus->busy) return 0;
  HAL_UART_Transmit(bus->uart_handle, tx_buf, tx_size, UART_TIMEOUT);
  HAL_UART_Receive(bus->uart_handle, rx_buf, rx_size, UART_TIMEOUT);
  return 1;
}

 uint8_t uart_transmit(UART_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) return 0;
  return HAL_UART_Transmit(&huart2, tx_buf, tx_size, UART_TIMEOUT);
}

 uint8_t uart_receive(UART_BUS *bus, uint8_t *rx_buf, uint16_t rx_size) {
  if (bus->busy) return 0;
  return HAL_UART_Receive(&huart2, rx_buf, rx_size, UART_TIMEOUT);
}
