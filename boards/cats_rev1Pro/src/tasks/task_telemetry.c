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

static uint8_t uart_char;

#define UART_FIFO_SIZE 40

static uint8_t usb_fifo_in_buf[UART_FIFO_SIZE];
static fifo_t uart_fifo = {
    .head = 0, .tail = 0, .used = 0, .size = UART_FIFO_SIZE, .buf = usb_fifo_in_buf, .mutex = false};
static stream_t uart_stream = {.fifo = &uart_fifo, .timeout_msec = 1};

void send_setting(uint8_t command, uint8_t value);
void parse(uint8_t op_code, const uint8_t* buffer, uint32_t length);
void send_link_phrase(uint8_t* phrase, uint32_t length);
void send_tx_payload(uint8_t* payload, uint32_t length);
bool check_valid_op_code(uint8_t op_code);
void send_enable();

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

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / TELEMETRY_SAMPLING_FREQ;
  while (1) {
    /* TODO add the payload to the tx_payload buffer */
    uint8_t tx_payload[16];

    uint32_t ts = osKernelGetTickCount();
    memcpy(&tx_payload[0], &ts, 4);

    send_tx_payload(tx_payload, 16);

    /* Check for data from the Telemetry MCU */
    while (stream_length(&uart_stream) > 1) {
      /* First read the Op Code */
      uint8_t op_code;
      do {
        stream_read_byte(&uart_stream, &op_code);
      } while (check_valid_op_code(op_code) == false);

      /* Read the data length */
      uint8_t length;
      if (stream_read_byte(&uart_stream, &length)) {
        /* Read the data */
        while ((length - uart_index) > 0) {
          if (stream_read_byte(&uart_stream, &uart_buffer[uart_index])) {
            uart_index++;
          }
          if (uart_index > 20) {
            uart_index = 0;
            break;
          }
        }
        parse(op_code, uart_buffer, length);
        uart_index = 0;
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

  /* TODO add all opcodes, store the data and make it look good :) */

  if (op_code == CMD_RX) {
    log_info("RX received");
  } else if (op_code == CMD_INFO) {
    log_info("Link Info received");
  } else if (op_code == CMD_GNSS_LOC) {
    log_info("GNSS location received");
  } else if (op_code == CMD_GNSS_INFO) {
    log_info("GNSS info received");
  } else if (op_code == CMD_GNSS_TIME) {
    log_info("GNSS time received");
  } else {
    log_error("Unknown Op Code");
  }
}

void send_link_phrase(uint8_t* phrase, uint32_t length) {
  uint8_t command = CMD_LINK_PHRASE;
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &command, 1, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, (uint8_t*)&length, 1, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, phrase, length, 2);
}

void send_setting(uint8_t command, uint8_t value) {
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &command, 1, 2);

  uint8_t length = 1;
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &length, 1, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &value, 1, 2);
}

void send_enable() {
  uint8_t command = CMD_ENABLE;
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &command, 1, 2);
  uint8_t length = 0;
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &length, 1, 2);
}

void send_tx_payload(uint8_t* payload, uint32_t length) {
  uint8_t command = CMD_TX;
  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, &command, 1, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, (uint8_t*)&length, 1, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, payload, length, 2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &TELEMETRY_UART_HANDLE) {
    uint8_t tmp = uart_char;
    HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, &uart_char, 1);
    stream_write_byte(&uart_stream, tmp);
  }
}
