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

enum e_spi_type {
  SPI_NORMAL = 1,
  SPI_IT = 2,
  SPI_DMA = 3,
};

enum e_cs_type {
  LOW_ACTIVE = 0,
  HIGH_ACTIVE = 1,
};

typedef struct instances {
  TaskHandle_t xTaskToNotify;
  uint8_t busy;
} task_origin;

typedef struct spi_bus {
  GPIO_TypeDef* const cs_port;
  uint16_t cs_pin;
  enum e_cs_type cs_type;
  enum e_spi_type spi_type;
  SPI_HandleTypeDef* const spi_handle;
  task_origin* origin;
  uint8_t initialized;
} SPI_BUS;

uint8_t spi_transmit_receive(SPI_BUS* bus, uint8_t* transmit, uint16_t tsize,
                             uint8_t* receive, uint16_t rsize);

#define MAX_INSTANCES 10
#define SPI_TIMEOUT   20

#endif /* INC_DRIVERS_SPI_H_ */
