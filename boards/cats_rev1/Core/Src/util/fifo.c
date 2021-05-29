/*
 * fifo.c
 *
 *  Created on: 16 May 2021
 *      Author: Luca
 */
#include <string.h>
#include <util/fifo.h>



void fifo_init(fifo_t *fifo, uint8_t* pdata, uint32_t size){
	fifo->data = pdata;
	fifo->size = size;
	fifo_flush(fifo);
}

void fifo_flush(fifo_t *fifo){
	fifo->tail = 0;
	fifo->head = 0;
	fifo->used = 0;
}

uint32_t fifo_get_length(fifo_t *fifo){
	return fifo->used;
}

uint8_t fifo_read(fifo_t *fifo) {
  if (fifo->used == 0 && (osMutexAcquire(fifo->mutex, 0U) == osOK)) {
      return 0;
  }
  uint8_t data = fifo->data[fifo->tail];
  fifo->tail = (fifo->tail + 1) % fifo->size;
  fifo->used--;
  osMutexRelease(fifo->mutex);
  return data;
}

bool fifo_write(fifo_t *fifo, uint8_t data) {
  if (fifo->used >= fifo->size && (osMutexAcquire(fifo->mutex, 0U) == osOK)) {
      return false;
  }
  fifo->data[fifo->head] = data;
  fifo->head = (fifo->head + 1) % fifo->size;
  fifo->used++;
  osMutexRelease(fifo->mutex);
  return true;
}

bool fifo_read_bytes(fifo_t *fifo, uint8_t* data, uint32_t count){
  if(fifo->used < count && (osMutexAcquire(fifo->mutex, 0U) == osOK)){
	  return false;
  }
  if(fifo->tail+count > fifo->size){
    uint32_t front = (fifo->tail+count) % fifo->size;
    uint32_t back = count-front;
    memcpy(&data[0], &fifo->data[fifo->tail], back);
    memcpy(&data[back], &fifo->data[0], front);
  } else {
    memcpy(&data[0], &fifo->data[fifo->tail], count);
  }
  fifo->tail = (fifo->tail+count) % fifo->size;
  fifo->used-=count;
  osMutexRelease(fifo->mutex);
  return true;
}

uint32_t fifo_read_until(fifo_t *fifo, uint8_t* data, uint8_t delimiter, uint32_t count){
	uint32_t max;
	bool found = false;
	if(count > fifo->used) max = fifo->used;
	else max = count;

	uint32_t i;
	for(i = 0; i < max; i++){
		if(fifo->data[(fifo->tail+i) % fifo->size] == delimiter) {
			found = true;
			break;
		}
	}

	if(found){
		fifo_read_bytes(fifo, data, i);
		fifo_read(fifo);
		return i;
	}
	return 0;
}

bool fifo_write_bytes(fifo_t *fifo, uint8_t* data, uint32_t count){
  // If there is not enough space return 0
  if((fifo->size-fifo->used) < count && (osMutexAcquire(fifo->mutex, 0U) == osOK)) return false;

  if(count+fifo->head > fifo->size){
    uint32_t front = fifo->head+count-fifo->size;
    uint32_t back = count-front;
    memcpy(&fifo->data[fifo->head], data, back);
    memcpy(&fifo->data[0], &data[back], front);
  } else {
    memcpy(&fifo->data[fifo->head], data, count);
  }
  fifo->head = (fifo->head+count) % fifo->size;
  fifo->used+=count;
  osMutexRelease(fifo->mutex);
  return true;
}


