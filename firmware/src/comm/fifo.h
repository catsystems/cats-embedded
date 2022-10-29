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

typedef struct {
  uint32_t head;
  uint32_t tail;

  uint32_t used;
  uint32_t size;

  uint8_t *buf;

  volatile bool mutex;
} fifo_t;

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
