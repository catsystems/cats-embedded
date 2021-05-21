//
// Created by Luca on 20/05/2021.
//

#pragma once

#include "cmsis_os.h"
#include <stdbool.h>

enum{
  FIFO_SIZE = 512
};

typedef struct {
  uint32_t head;
  uint32_t tail;
  uint32_t used;
  uint8_t data[FIFO_SIZE];
} fifo_t;

uint32_t fifo_get_length(fifo_t *fifo);
void fifo_flush(fifo_t *fifo);

uint8_t fifo_read(fifo_t *fifo);
bool fifo_write(fifo_t *fifo, uint8_t data);

bool fifo_read_bytes(fifo_t *fifo, uint8_t* data, uint32_t count);
uint32_t fifo_read_until(fifo_t *fifo, uint8_t* data, uint8_t delimiter, uint32_t count);

bool fifo_write_bytes(fifo_t *fifo, uint8_t* data, uint32_t count);

