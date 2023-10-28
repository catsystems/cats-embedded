/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "cmsis_os.h"

struct fifo_t {
  uint32_t head;
  uint32_t tail;

  uint32_t used;
  uint32_t size;

  uint8_t *buf;

  volatile bool mutex;
};

/**
 * Initializes the fifo with an underlying buffer.
 * @param fifo
 * @param buf
 * @param size
 */
void fifo_init(fifo_t *fifo, uint8_t *buf, uint32_t size);

/**
 * Resets the fifo pointers; does not affect the underlying buffer.
 */
void fifo_reset(fifo_t *fifo);

/**
 * Get the number of occupied bytes in the fifo.
 */
uint32_t fifo_get_length(const fifo_t *fifo);

/**
 * Read a single byte from a fifo.
 * @param fifo
 * @param byte_ptr pointer to where the byte will be written
 * @return
 */
bool fifo_read_byte(fifo_t *fifo, uint8_t *byte_ptr);

/**
 * Write a single byte to a fifo.
 */
bool fifo_write_byte(fifo_t *fifo, uint8_t data);

/**
 * Read multiple bytes from a fifo.
 * @param fifo
 * @param data pointer to where the data will be written
 * @param count number of bytes to read
 * @return
 */
bool fifo_read(fifo_t *fifo, uint8_t *data, uint32_t count);

/**
 * Write multiple bytes to a fifo.
 * @param fifo
 * @param data bytes to be written
 * @param count number of bytes to write
 * @return
 */
bool fifo_write(fifo_t *fifo, const uint8_t *data, uint32_t count);

uint32_t fifo_read_until(fifo_t *fifo, uint8_t *data, uint8_t delimiter, uint32_t count);
