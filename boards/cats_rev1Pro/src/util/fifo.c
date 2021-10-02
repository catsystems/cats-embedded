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

#include <string.h>
#include "util/fifo.h"

void fifo_init(fifo_t *fifo, uint8_t *pdata, uint32_t size) {
  fifo->data = pdata;
  fifo->size = size;
  fifo->semaphore_id = osSemaphoreNew(1, 1, NULL);
  fifo_flush(fifo);
}

void fifo_flush(fifo_t *fifo) {
  if (osSemaphoreAcquire(fifo->semaphore_id, 0U) == osOK) {
    fifo->tail = 0;
    fifo->head = 0;
    fifo->used = 0;
    osSemaphoreRelease(fifo->semaphore_id);
  }
}

uint32_t fifo_get_length(fifo_t *fifo) { return fifo->used; }

uint8_t fifo_read(fifo_t *fifo) {
  if (osSemaphoreAcquire(fifo->semaphore_id, 0U) == osOK) {
    if (fifo->used == 0) {
      osSemaphoreRelease(fifo->semaphore_id);
      return 0;
    }
    uint8_t data = fifo->data[fifo->tail];
    fifo->tail = (fifo->tail + 1) % fifo->size;
    fifo->used--;
    osSemaphoreRelease(fifo->semaphore_id);
    return data;
  }
  return 0;
}

bool fifo_write(fifo_t *fifo, uint8_t data) {
  if (osSemaphoreAcquire(fifo->semaphore_id, 0U) == osOK) {
    if (fifo->used >= fifo->size) {
      osSemaphoreRelease(fifo->semaphore_id);
      return false;
    }
    fifo->data[fifo->head] = data;
    fifo->head = (fifo->head + 1) % fifo->size;
    fifo->used++;
    osSemaphoreRelease(fifo->semaphore_id);
    return true;
  }
  return false;
}

bool fifo_read_bytes(fifo_t *fifo, uint8_t *data, uint32_t count) {
  if (osSemaphoreAcquire(fifo->semaphore_id, 0U) == osOK) {
    if (fifo->used < count) {
      osSemaphoreRelease(fifo->semaphore_id);
      return false;
    }
    if (fifo->tail + count > fifo->size) {
      uint32_t front = (fifo->tail + count) % fifo->size;
      uint32_t back = count - front;
      memcpy(&data[0], &fifo->data[fifo->tail], back);
      memcpy(&data[back], &fifo->data[0], front);
    } else {
      memcpy(&data[0], &fifo->data[fifo->tail], count);
    }
    fifo->tail = (fifo->tail + count) % fifo->size;
    fifo->used -= count;
    osSemaphoreRelease(fifo->semaphore_id);
    return true;
  }
  return false;
}

uint32_t fifo_read_until(fifo_t *fifo, uint8_t *data, uint8_t delimiter, uint32_t count) {
  uint32_t max;
  bool found = false;
  if (count > fifo->used)
    max = fifo->used;
  else
    max = count;

  uint32_t i;
  for (i = 0; i < max; i++) {
    if (fifo->data[(fifo->tail + i) % fifo->size] == delimiter) {
      found = true;
      break;
    }
  }

  if (found) {
    fifo_read_bytes(fifo, data, i);
    fifo_read(fifo);
    return i;
  }
  return 0;
}

bool fifo_write_bytes(fifo_t *fifo, uint8_t *data, uint32_t count) {
  // If there is not enough space return false
  if (osSemaphoreAcquire(fifo->semaphore_id, 0U) == osOK) {
    if ((fifo->size - fifo->used) < count) {
      osSemaphoreRelease(fifo->semaphore_id);
      return false;
    }
    if (count + fifo->head > fifo->size) {
      uint32_t front = fifo->head + count - fifo->size;
      uint32_t back = count - front;
      memcpy(&fifo->data[fifo->head], data, back);
      memcpy(&fifo->data[0], &data[back], front);
    } else {
      memcpy(&fifo->data[fifo->head], data, count);
    }
    fifo->head = (fifo->head + count) % fifo->size;
    fifo->used += count;
    osSemaphoreRelease(fifo->semaphore_id);
    return true;
  }
  return false;
}

void fifo_write_str(fifo_t *fifo, const char *str) {
  while (*str) {
    while (fifo_write(fifo, *str++) == false) {
      osDelay(1);
      str--;
    }
  }
}
