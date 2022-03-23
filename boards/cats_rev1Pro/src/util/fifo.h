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
#include <stdbool.h>

typedef struct {
  uint32_t head;
  uint32_t tail;
  uint32_t used;
  uint32_t size;
  uint8_t *data;
  //osSemaphoreId_t semaphore_id;
  volatile bool mutex;
} fifo_t;

void fifo_init(fifo_t *fifo, uint8_t *pdata, uint32_t size);
uint32_t fifo_get_length(fifo_t *fifo);
void fifo_flush(fifo_t *fifo);

uint8_t fifo_read(fifo_t *fifo);
bool fifo_write(fifo_t *fifo, uint8_t data);

bool fifo_read_bytes(fifo_t *fifo, uint8_t *data, uint32_t count);
uint32_t fifo_read_until(fifo_t *fifo, uint8_t *data, uint8_t delimiter, uint32_t count);

bool fifo_write_bytes(fifo_t *fifo, uint8_t *data, uint32_t count);

void fifo_write_str(fifo_t *fifo, const char *str);
