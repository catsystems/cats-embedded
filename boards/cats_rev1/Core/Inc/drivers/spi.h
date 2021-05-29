/*
 * spi.h
 *
 *  Created on: 8 Apr 2021
 *      Author: Luca
 */

#pragma once

#include "cmsis_os.h"
#include "util/types.h"
#include "stm32l4xx_hal.h"

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

#define MAX_INSTANCES 10
#define SPI_TIMEOUT   20
