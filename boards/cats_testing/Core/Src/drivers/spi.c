/*
 * spi.c
 *
 *  Created on: 8 Apr 2021
 *      Author: Luca
 */

#include "drivers/spi.h"
#include <stdlib.h>

static uint16_t instance_count = 0;

static SPI_BUS *buses[MAX_INSTANCES];

void spi_init(SPI_BUS *bus) {
  buses[instance_count] = bus;
  instance_count++;
}

inline uint8_t spi_transmit_receive(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size, uint8_t *rx_buf,
                                    uint16_t rx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
  HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  return 1;
}

inline uint8_t spi_transmit(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  return 1;
}

inline uint8_t spi_receive(SPI_BUS *bus, uint8_t *rx_buf, uint16_t rx_size) {
  if (bus->busy) return 0;
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  return 1;
}

inline uint8_t spi_transmit_it(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (bus->busy) return 0;
  // Toggle CS to make device active
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  bus->busy = true;
  HAL_SPI_Transmit_IT(bus->spi_handle, tx_buf, tx_size);
  return 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < instance_count; i++) {
    if (buses[i]->busy) {
      HAL_GPIO_WritePin(buses[i]->cs_port, buses[i]->cs_pin, !buses[i]->cs_type);
      buses[i]->busy = false;
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  for (int i = 0; i < instance_count; i++) {
    if (buses[i]->busy) {
      HAL_GPIO_WritePin(buses[i]->cs_port, buses[i]->cs_pin, !buses[i]->cs_type);
      buses[i]->busy = false;
    }
  }
}
