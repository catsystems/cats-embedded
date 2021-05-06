/*
 * spi.h
 *
 *  Created on: 8 Apr 2021
 *      Author: Luca
 */

#ifndef INC_DRIVERS_SPI_H_
#define INC_DRIVERS_SPI_H_

#include "cmsis_os.h"
#include "util/types.h"
#include "stm32l4xx_hal.h"

typedef enum spi_type {
  SPI_NORMAL = 1,
  SPI_IT = 2,
  SPI_DMA = 3,
} spi_type_e;

typedef enum cs_type {
  LOW_ACTIVE = 0,
  HIGH_ACTIVE = 1,
} cs_type_e;

typedef struct instances {
  TaskHandle_t xTaskToNotify;
  uint8_t busy;
} task_origin;

typedef struct spi_bus {
  GPIO_TypeDef* const cs_port;
  uint16_t cs_pin;
  cs_type_e cs_type;
  spi_type_e spi_type;
  SPI_HandleTypeDef* const spi_handle;
  task_origin* origin;
  uint8_t initialized;
} SPI_BUS;

uint8_t spi_transmit_receive(SPI_BUS* bus, uint8_t* tx_buf, uint16_t tx_size,
                             uint8_t* rx_buf, uint16_t rx_size);
uint8_t spi_transmit(SPI_BUS* bus, uint8_t* tx_buf, uint16_t tx_size);
uint8_t spi_receive(SPI_BUS* bus, uint8_t* rx_buf, uint16_t rx_size);

#define MAX_INSTANCES 10
#define SPI_TIMEOUT   20

#endif /* INC_DRIVERS_SPI_H_ */
