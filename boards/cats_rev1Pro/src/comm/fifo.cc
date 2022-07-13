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

#include "comm/fifo.h"

#include <cstring>

void fifo_init(fifo_t *fifo, uint8_t *buf, uint32_t size) {
  fifo->buf = buf;
  fifo->size = size;
  fifo->mutex = false;
  fifo_reset(fifo);
}

void fifo_reset(fifo_t *fifo) {
  if (fifo->mutex == false) {
    fifo->mutex = true;
    fifo->tail = 0;
    fifo->head = 0;
    fifo->used = 0;
    fifo->mutex = false;
  }
}

uint32_t fifo_get_length(const fifo_t *const fifo) { return fifo->used; }

bool fifo_read_byte(fifo_t *const fifo, uint8_t *byte_ptr) {
  if (fifo->mutex == false) {
    fifo->mutex = true;
    if (fifo->used == 0) {
      fifo->mutex = false;
      return false;
    }
    uint8_t data = fifo->buf[fifo->tail];
    fifo->tail = (fifo->tail + 1) % fifo->size;
    fifo->used--;
    fifo->mutex = false;
    *byte_ptr = data;
    return true;
  }
  return false;
}

bool fifo_write_byte(fifo_t *const fifo, uint8_t data) {
  if (fifo->mutex == false) {
    fifo->mutex = true;
    if (fifo->used >= fifo->size) {
      fifo->mutex = false;
      return false;
    }
    fifo->buf[fifo->head] = data;
    fifo->head = (fifo->head + 1) % fifo->size;
    fifo->used++;
    fifo->mutex = false;
    return true;
  }
  return false;
}

bool fifo_read(fifo_t *const fifo, uint8_t *data, uint32_t count) {
  if (fifo->mutex == false) {
    fifo->mutex = true;
    if (fifo->used < count) {
      fifo->mutex = false;
      return false;
    }
    if (fifo->tail + count > fifo->size) {
      uint32_t front = (fifo->tail + count) % fifo->size;
      uint32_t back = count - front;
      memcpy(&data[0], &fifo->buf[fifo->tail], back);
      memcpy(&data[back], &fifo->buf[0], front);
    } else {
      memcpy(&data[0], &fifo->buf[fifo->tail], count);
    }
    fifo->tail = (fifo->tail + count) % fifo->size;
    fifo->used -= count;
    fifo->mutex = false;
    return true;
  }
  return false;
}

bool fifo_write(fifo_t *const fifo, const uint8_t *data, uint32_t count) {
  // If there is not enough space return false
  if (fifo->mutex == false) {
    fifo->mutex = true;
    if ((fifo->size - fifo->used) < count) {
      fifo->mutex = false;
      return false;
    }
    if (count + fifo->head > fifo->size) {
      uint32_t front = fifo->head + count - fifo->size;
      uint32_t back = count - front;
      memcpy(&fifo->buf[fifo->head], data, back);
      memcpy(&fifo->buf[0], &data[back], front);
    } else {
      memcpy(&fifo->buf[fifo->head], data, count);
    }
    fifo->head = (fifo->head + count) % fifo->size;
    fifo->used += count;
    fifo->mutex = false;
    return true;
  }
  return false;
}

uint32_t fifo_read_until(fifo_t *const fifo, uint8_t *data, uint8_t delimiter, uint32_t count) {
  uint32_t max;
  bool found = false;
  if (count > fifo->used)
    max = fifo->used;
  else
    max = count;

  uint32_t i;
  for (i = 0; i < max; i++) {
    if (fifo->buf[(fifo->tail + i) % fifo->size] == delimiter) {
      found = true;
      break;
    }
  }

  if (found) {
    fifo_read(fifo, data, i);
    uint8_t dummy;
    fifo_read_byte(fifo, &dummy);
    return i;
  }
  return 0;
}
