//
// Created by Luca on 20/05/2021.
//

#pragma once

#include "cmsis_os.h"
#include <stdbool.h>

typedef struct {
  uint32_t head;
  uint32_t tail;
  uint32_t used;
  uint32_t size;
  uint8_t *data;
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
