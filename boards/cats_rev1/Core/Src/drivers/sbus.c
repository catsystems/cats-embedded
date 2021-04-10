/*
 * sbus.c
 *
 *  Created on: 8 Apr 2021
 *      Author: Luca
 */

#include "drivers/sbus.h"
#include "config/globals.h"
#include "stm32l4xx_hal.h"
#include "util/types.h"

extern UART_HandleTypeDef huart1;

void _reset_buffer(uint8_t* buffer, uint32_t length);
void map_channels(receiver_data_t* data, uint8_t* _payload);
void receiver_reorder();

static uint8_t sbus_input_buffer[25];
static uint8_t receiver_data_raw[25];

void sbus_init() { HAL_UART_Receive_DMA(&huart1, sbus_input_buffer, 25); }

void sbus_update(receiver_data_t* data) {
  receiver_reorder();
  map_channels(data, receiver_data_raw + 1);
}

void receiver_reorder() {
  memcpy(receiver_data_raw, sbus_input_buffer, 24);
  for (int i = 0; i < 24; i++) {
    if (sbus_input_buffer[i] == 0 && sbus_input_buffer[i + 1] == 15) {
      memcpy(receiver_data_raw, &sbus_input_buffer[i + 1], 24 - i);
      memcpy(&receiver_data_raw[24 - i], &sbus_input_buffer, i + 1);
      break;
    }
  }
}

static uint16_t linear_fit(uint16_t value) {
  // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
  return (5 * value / 8) + 880;
}

void map_channels(receiver_data_t* data, uint8_t* _payload) {
  data->ch[0] = (uint16_t)((_payload[0] | _payload[1] << 8) & 0x07FF);
  data->ch[1] = (uint16_t)((_payload[1] >> 3 | _payload[2] << 5) & 0x07FF);
  data->ch[2] = (uint16_t)(
      (_payload[2] >> 6 | _payload[3] << 2 | _payload[4] << 10) & 0x07FF);
  data->ch[3] = (uint16_t)((_payload[4] >> 1 | _payload[5] << 7) & 0x07FF);
  data->ch[4] = (uint16_t)((_payload[5] >> 4 | _payload[6] << 4) & 0x07FF);
  data->ch[5] = (uint16_t)(
      (_payload[6] >> 7 | _payload[7] << 1 | _payload[8] << 9) & 0x07FF);
  data->ch[6] = (uint16_t)((_payload[8] >> 2 | _payload[9] << 6) & 0x07FF);
  data->ch[7] = (uint16_t)((_payload[9] >> 5 | _payload[10] << 3) & 0x07FF);
  data->ch[8] = (uint16_t)((_payload[11] | _payload[12] << 8) & 0x07FF);
  data->ch[9] = (uint16_t)((_payload[12] >> 3 | _payload[13] << 5) & 0x07FF);
  data->ch[10] = (uint16_t)(
      (_payload[13] >> 6 | _payload[14] << 2 | _payload[15] << 10) & 0x07FF);
  data->ch[11] = (uint16_t)((_payload[15] >> 1 | _payload[16] << 7) & 0x07FF);
  data->ch[12] = (uint16_t)((_payload[16] >> 4 | _payload[17] << 4) & 0x07FF);
  data->ch[13] = (uint16_t)(
      (_payload[17] >> 7 | _payload[18] << 1 | _payload[19] << 9) & 0x07FF);
  data->ch[14] = (uint16_t)((_payload[19] >> 2 | _payload[20] << 6) & 0x07FF);
  data->ch[15] = (uint16_t)((_payload[20] >> 5 | _payload[21] << 3) & 0x07FF);

  for (int i = 0; i < 16; i++) {
    if (data->ch[i] != 0) data->ch[i] = linear_fit(data->ch[i]);
  }
  data->failsafe = (_payload[22] & 0x08) ? 1 : 0;
  data->framelost = (_payload[22] & 0x04) ? 1 : 0;
}
