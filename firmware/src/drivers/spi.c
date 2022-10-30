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

#include "drivers/spi.h"

static uint16_t instance_count = 0;

static SPI_BUS *buses[MAX_INSTANCES];

void spi_init(SPI_BUS *bus) {
  buses[instance_count] = bus;
  instance_count++;
}

 uint8_t spi_transmit_receive(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size, uint8_t *rx_buf,
                                    uint16_t rx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(bus->cs_type));
  HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
  HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(!bus->cs_type));
  return 1;
}

 uint8_t spi_transmit(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(bus->cs_type));
  HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(!bus->cs_type));
  return 1;
}

 uint8_t spi_receive(SPI_BUS *bus, uint8_t *rx_buf, uint16_t rx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(bus->cs_type));
  HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(!bus->cs_type));
  return 1;
}

 uint8_t spi_transmit_it(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) return 0;
  // Toggle CS to make device active
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, (GPIO_PinState)(bus->cs_type));
  bus->busy = true;
  HAL_SPI_Transmit_IT(bus->spi_handle, tx_buf, tx_size);
  return 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < instance_count; i++) {
    if (buses[i]->busy) {
      HAL_GPIO_WritePin(buses[i]->cs_port, buses[i]->cs_pin, (GPIO_PinState)(!buses[i]->cs_type));
      buses[i]->busy = false;
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < instance_count; i++) {
    if (buses[i]->busy) {
      HAL_GPIO_WritePin(buses[i]->cs_port, buses[i]->cs_pin, (GPIO_PinState)(!buses[i]->cs_type));
      buses[i]->busy = false;
    }
  }
}
