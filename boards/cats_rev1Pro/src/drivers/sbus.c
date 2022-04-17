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

#include "drivers/sbus.h"
#include "target.h"

extern UART_HandleTypeDef huart1;

void reset_buffer(uint8_t* buffer, uint32_t length);
void map_channels(receiver_data_t* data, const uint8_t* payload);
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

void map_channels(receiver_data_t* data, const uint8_t* payload) {
  data->ch[0] = (uint16_t)((payload[0] | payload[1] << 8) & 0x07FF);
  data->ch[1] = (uint16_t)((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
  data->ch[2] = (uint16_t)((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
  data->ch[3] = (uint16_t)((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
  data->ch[4] = (uint16_t)((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
  data->ch[5] = (uint16_t)((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
  data->ch[6] = (uint16_t)((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
  data->ch[7] = (uint16_t)((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
  data->ch[8] = (uint16_t)((payload[11] | payload[12] << 8) & 0x07FF);
  data->ch[9] = (uint16_t)((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
  data->ch[10] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
  data->ch[11] = (uint16_t)((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
  data->ch[12] = (uint16_t)((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
  data->ch[13] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
  data->ch[14] = (uint16_t)((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
  data->ch[15] = (uint16_t)((payload[20] >> 5 | payload[21] << 3) & 0x07FF);

  for (int i = 0; i < 16; i++) {
    if (data->ch[i] != 0) data->ch[i] = linear_fit(data->ch[i]);
  }
  data->failsafe = (payload[22] & 0x08) ? 1 : 0;
  data->framelost = (payload[22] & 0x04) ? 1 : 0;
}
