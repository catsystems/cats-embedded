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

#pragma once

#include "cmsis_os.h"
#include "util/types.h"
#include "target.h"

typedef enum cs_type {
  LOW_ACTIVE = GPIO_PIN_RESET,
  HIGH_ACTIVE = GPIO_PIN_SET,
} cs_type_e;

typedef struct spi_bus {
  GPIO_TypeDef* const cs_port;
  uint16_t cs_pin;
  cs_type_e cs_type;
  SPI_HandleTypeDef* const spi_handle;
  uint8_t initialized;
  bool busy;
} SPI_BUS;

uint8_t spi_transmit_receive(SPI_BUS* bus, uint8_t* tx_buf, uint16_t tx_size, uint8_t* rx_buf, uint16_t rx_size);
uint8_t spi_transmit(SPI_BUS* bus, uint8_t* tx_buf, uint16_t tx_size);
uint8_t spi_receive(SPI_BUS* bus, uint8_t* rx_buf, uint16_t rx_size);
void spi_init(SPI_BUS* bus);

#define MAX_INSTANCES 10
#define SPI_TIMEOUT   5
