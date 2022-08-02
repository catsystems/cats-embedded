/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "tasks/task_telemetry.h"
#include "comm/fifo.h"
#include "comm/stream.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "util/log.h"
#include "util/telemetry_reg.h"
#include "util/crc.h"


static uint8_t uart_char;

#define UART_FIFO_SIZE 40

static uint8_t usb_fifo_in_buf[UART_FIFO_SIZE];
static fifo_t uart_fifo = {
    .head = 0, .tail = 0, .used = 0, .size = UART_FIFO_SIZE, .buf = usb_fifo_in_buf, .mutex = false};
static stream_t uart_stream = {.fifo = &uart_fifo, .timeout_msec = 1};

typedef enum {
  STATE_OP,
  STATE_LEN,
  STATE_DATA,
  STATE_CRC,
} state_e;

#define INDEX_OP 0
#define INDEX_LEN 1

void send_setting(uint8_t command, uint8_t value);
void parse(uint8_t op_code, const uint8_t* buffer, uint32_t length);
void send_link_phrase(uint8_t* phrase, uint32_t length);
void send_tx_payload(uint8_t* payload, uint32_t length);
bool check_valid_op_code(uint8_t op_code);
void send_enable();

typedef struct {
  uint8_t state : 3;
  uint8_t errors : 4;
  uint16_t timestamp : 15;
  int32_t lat : 22;
  int32_t lon : 22;
  int32_t altitude : 17;
  int16_t velocity : 10;
  uint16_t voltage : 8;
  uint16_t continuity : 3;
  // fill up to 16 bytes
  uint8_t : 0;  // sent
  uint8_t d1;   // dummy
  uint8_t d2;   // dummy
  uint8_t d3;   // dummy
} __attribute__((packed)) packed_tx_msg_t;

typedef struct {
  double lat;
  double lon;
  uint8_t sats;
} gnss_data_t;

gnss_data_t gnss_data;

void pack_tx_msg(packed_tx_msg_t* tx_payload) {
  /* TODO add state, error, voltage and continuity information */
  tx_payload->timestamp = osKernelGetTickCount() / 100;
  tx_payload->altitude =   (int32_t)global_estimation_data.height;
  tx_payload->velocity = (int16_t)global_estimation_data.velocity;
  tx_payload->lat = (int32_t)(gnss_data.lat * 10000);
  tx_payload->lon = (int32_t)(gnss_data.lon * 10000);
}

void parse_tx_msg(packed_tx_msg_t* tx_payload) {
  uint32_t ts = tx_payload->timestamp;
  int32_t altitude = tx_payload->altitude;
  int16_t velocity = tx_payload->velocity;
  int32_t lat = tx_payload->lat;
  int32_t lon = tx_payload->lon;
  log_raw("[TELE] ts: %lu alt: %ld vel: %d lat: %ld lon %ld", ts, altitude, velocity, lat, lon);
}

[[noreturn]] void task_telemetry(__attribute__((unused)) void* argument) {
  /* Give the telemetry hardware some time to initialize */
  osDelay(5000);

  /* Configure the telemetry MCU */
  send_setting(CMD_DIRECTION, global_cats_config.config.telemetry_settings.direction);
  osDelay(100);
  send_setting(CMD_POWER_LEVEL, global_cats_config.config.telemetry_settings.power_level);
  osDelay(100);
  /* Only start the telemetry when a link phrase is set */
  if (global_cats_config.config.telemetry_settings.link_phrase[0] != 0) {
    send_link_phrase(global_cats_config.config.telemetry_settings.link_phrase, 8);
    osDelay(100);
    send_enable();
  }

  /* Start the interrupt request for the UART */
  HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, (uint8_t*)&uart_char, 1);

  uint8_t uart_buffer[20];
  uint32_t uart_index = 0;
  state_e state = STATE_OP;
  bool valid_op = false;

  packed_tx_msg_t tx_payload = {};

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / TELEMETRY_SAMPLING_FREQ;
  while (1) {

    if(global_cats_config.config.telemetry_settings.direction == TX) {
      pack_tx_msg(&tx_payload);
      send_tx_payload((uint8_t*)&tx_payload, 16);
    }

    /* Check for data from the Telemetry MCU */
    while (stream_length(&uart_stream) > 1) {
      uint8_t ch;
      stream_read_byte(&uart_stream, &ch);
      switch(state){
        case STATE_OP:
          valid_op = check_valid_op_code(ch);
          if(valid_op){
            uart_buffer[INDEX_OP] = ch;
            state = STATE_LEN;
          }
          break;
        case STATE_LEN:
          if (ch <= 16){
            uart_buffer[INDEX_LEN] = ch;
            if(ch > 0){
              state = STATE_DATA;
            } else {
              state = STATE_CRC;
            }
          }
          break;
        case STATE_DATA:
          if((uart_buffer[1] - uart_index) > 0){
            uart_buffer[uart_index + 2] = ch;
            uart_index++;
          }
          if((uart_buffer[INDEX_LEN] - uart_index) == 0){
            state = STATE_CRC;
          }
          break;
        case STATE_CRC: {
          uint8_t crc;
          crc = crc8(uart_buffer, uart_index + 2);
          if (crc == ch) {
            parse(uart_buffer[INDEX_OP], &uart_buffer[2], uart_buffer[INDEX_LEN]);
          }
          uart_index = 0;
          state = STATE_OP;
        }
          break;
        default:
          break;
      }

    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

bool check_valid_op_code(uint8_t op_code) {
  /* TODO loop over all opcodes and check if it exists */
  if (op_code == CMD_GNSS_INFO || op_code == CMD_GNSS_LOC || op_code == CMD_RX || op_code == CMD_INFO ||
      op_code == CMD_GNSS_TIME) {
    return true;
  } else {
    return false;
  }
}

void parse(uint8_t op_code, const uint8_t* buffer, uint32_t length) {
  if (length < 1) return;

  if (op_code == CMD_RX) {
    packed_tx_msg_t rx_payload;
    memcpy(&rx_payload, buffer, length);
    parse_tx_msg(&rx_payload);
    //log_info("RX received");
  } else if (op_code == CMD_INFO) {
    //log_info("Link Info received");
  } else if (op_code == CMD_GNSS_LOC) {
    memcpy(&gnss_data.lat, buffer, 8);
    memcpy(&gnss_data.lon, &buffer[8], 8);
    //log_info("GNSS location received: %f %f",gnss_data.lat, gnss_data.lon);
  } else if (op_code == CMD_GNSS_INFO) {
    gnss_data.sats = buffer[0];
    //log_info("GNSS info received: %u", gnss_data.sats);
  } else if (op_code == CMD_GNSS_TIME) {
    //log_info("GNSS time received");
  } else {
    log_error("Unknown Op Code");
  }
}

void send_link_phrase(uint8_t* phrase, uint32_t length) {
  uint8_t out[11]; // 1 OP + 1 LEN + 8 DATA + 1 CRC
  out[0] = CMD_LINK_PHRASE;
  out[1] = (uint8_t)length;
  memcpy(&out[2], phrase, length);
  out[length+2] = crc8(out, length+2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, length+3, 2);
}

void send_setting(uint8_t command, uint8_t value) {
  uint8_t out[4]; // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = command;
  out[1] = 1;
  out[2] = value;
  out[3] = crc8(out, 3);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 4, 2);
}

void send_enable() {
  uint8_t out[3]; // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = CMD_ENABLE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void send_tx_payload(uint8_t* payload, uint32_t length) {
  uint8_t out[19]; // 1 OP + 1 LEN + 16 DATA + 1 CRC
  out[0] = CMD_TX;
  out[1] = (uint8_t)length;
  memcpy(&out[2], payload, length);
  out[length+2] = crc8(out, length+2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, length+3, 2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &TELEMETRY_UART_HANDLE) {
    uint8_t tmp = uart_char;
    HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, &uart_char, 1);
    stream_write_byte(&uart_stream, tmp);
  }
}
