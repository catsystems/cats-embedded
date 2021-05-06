/*
 * spi.c
 *
 *  Created on: 8 Apr 2021
 *      Author: Luca
 */

#include "drivers/spi.h"
#include <stdlib.h>

static uint16_t instance_count = 0;

// TODO dynamically allocate the origin array pointers
static task_origin *origin_parray[MAX_INSTANCES];

static void spi_init(SPI_BUS *bus) {
  // Allocate the task origin struct
  origin_parray[instance_count] = (task_origin *)malloc(sizeof(task_origin));

  // Save the task_origin pointer in object
  bus->origin = origin_parray[instance_count];
  instance_count++;
  bus->initialized = 1;
}

// A task blocking, but non blocking freertos spi tx_buf rx_buf function
// Dropin replacement for HAL_SPI_TransmitReceive
uint8_t spi_transmit_receive(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size,
                             uint8_t *rx_buf, uint16_t rx_size) {
  if (!bus->initialized) spi_init(bus);
  bus->origin->xTaskToNotify = xTaskGetCurrentTaskHandle();
  // Toggle CS to make device active
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  if (bus->spi_type == SPI_NORMAL) {
    HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
    HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  } else if (bus->spi_type == SPI_IT) {
    bus->origin->busy = 1;
    HAL_SPI_Transmit_IT(bus->spi_handle, tx_buf, tx_size);
    // Wait for transmission to end without blocking
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    if (!notification) {
      // Timed out
      bus->origin->busy = 0;
      HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
      return 0;
    }
    bus->origin->busy = 1;
    HAL_SPI_Receive_IT(bus->spi_handle, rx_buf, rx_size);
    notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
    if (!notification) {
      bus->origin->busy = 0;
      return 0;  // Timed out
    }
  } else if (bus->spi_type == SPI_DMA) {
    bus->origin->busy = 1;
    HAL_SPI_Transmit_DMA(bus->spi_handle, tx_buf, tx_size);
    // Wait for transmission to end without blocking
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    if (!notification) {
      // Timed out
      bus->origin->busy = 0;
      HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
      return 0;
    }
    bus->origin->busy = 1;
    HAL_SPI_Receive_DMA(bus->spi_handle, rx_buf, rx_size);
    notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
    if (!notification) {
      bus->origin->busy = 0;
      return 0;  // Timed out
    }
  }
  return 1;
}

uint8_t spi_transmit(SPI_BUS *bus, uint8_t *tx_buf, uint16_t tx_size) {
  if (!bus->initialized) spi_init(bus);
  bus->origin->xTaskToNotify = xTaskGetCurrentTaskHandle();
  // Toggle CS to make device active
  // HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  if (bus->spi_type == SPI_NORMAL) {
    HAL_SPI_Transmit(bus->spi_handle, tx_buf, tx_size, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  } else if (bus->spi_type == SPI_IT) {
    bus->origin->busy = 1;
    HAL_SPI_Transmit_IT(bus->spi_handle, tx_buf, tx_size);
    // Wait for transmission to end without blocking
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    if (!notification) {
      // Timed out
      bus->origin->busy = 0;
      // HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
      return 0;
    }
  } else if (bus->spi_type == SPI_DMA) {
    bus->origin->busy = 1;
    HAL_SPI_Transmit_DMA(bus->spi_handle, tx_buf, tx_size);
    // Wait for transmission to end without blocking
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    if (!notification) {
      // Timed out
      bus->origin->busy = 0;
      // HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
      return 0;
    }
  }
  return 1;
}

uint8_t spi_receive(SPI_BUS *bus, uint8_t *rx_buf, uint16_t rx_size) {
  if (!bus->initialized) spi_init(bus);
  bus->origin->xTaskToNotify = xTaskGetCurrentTaskHandle();
  // Toggle CS to make device active
  HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, bus->cs_type);
  if (bus->spi_type == SPI_NORMAL) {
    HAL_SPI_Receive(bus->spi_handle, rx_buf, rx_size, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
  } else if (bus->spi_type == SPI_IT) {
    bus->origin->busy = 1;
    HAL_SPI_Receive_IT(bus->spi_handle, rx_buf, rx_size);
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
    if (!notification) {
      bus->origin->busy = 0;
      return 0;  // Timed out
    }
  } else if (bus->spi_type == SPI_DMA) {
    bus->origin->busy = 1;
    HAL_SPI_Receive_DMA(bus->spi_handle, rx_buf, rx_size);
    uint32_t notification = ulTaskNotifyTake(pdTRUE, SPI_TIMEOUT);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, !bus->cs_type);
    if (!notification) {
      bus->origin->busy = 0;
      return 0;  // Timed out
    }
  }
  return 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  for (int i = 0; i < instance_count; i++) {
    if (origin_parray[i]->busy) {
      vTaskNotifyGiveFromISR(origin_parray[i]->xTaskToNotify,
                             &xHigherPriorityTaskWoken);
      origin_parray[i]->busy = 0;
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  for (int i = 0; i < instance_count; i++) {
    if (origin_parray[i]->busy) {
      vTaskNotifyGiveFromISR(origin_parray[i]->xTaskToNotify,
                             &xHigherPriorityTaskWoken);
      origin_parray[i]->busy = 0;
    }
  }
}
