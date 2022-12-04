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
#include "util/crc.h"
#include "util/gnss.h"
#include "util/log.h"
#include "util/telemetry_reg.h"

static uint8_t uart_char;

#define UART_FIFO_SIZE 40

static uint8_t usb_fifo_in_buf[UART_FIFO_SIZE];
static fifo_t uart_fifo = {
    .head = 0, .tail = 0, .used = 0, .size = UART_FIFO_SIZE, .buf = usb_fifo_in_buf, .mutex = false};
static stream_t uart_stream = {.fifo = &uart_fifo, .timeout_msec = 1};

enum state_e {
  STATE_OP,
  STATE_LEN,
  STATE_DATA,
  STATE_CRC,
};

#define INDEX_OP       0
#define INDEX_LEN      1
#define TELE_MAX_POWER 30

void send_setting(uint8_t command, uint8_t value);
bool parse(uint8_t op_code, const uint8_t* buffer, uint32_t length, gnss_data_t* gnss);
void send_link_phrase(uint8_t* phrase, uint32_t length);
void send_tx_payload(uint8_t* payload, uint32_t length);
bool check_valid_op_code(uint8_t op_code);
void send_enable();
void send_disable();

struct packed_tx_msg_t {
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
} __attribute__((packed));

static float amplifier_temperature = 0.0F;

void pack_tx_msg(uint32_t ts, gnss_data_t* gnss, packed_tx_msg_t* tx_payload) {
  /* TODO add state, error, voltage and continuity information */
  if (global_flight_state.flight_state == MOVING) {
    tx_payload->state = 0;
  } else if (global_flight_state.flight_state == READY) {
    tx_payload->state = 1;
  } else if (global_flight_state.flight_state == THRUSTING) {
    tx_payload->state = 2;
  } else if (global_flight_state.flight_state == COASTING) {
    tx_payload->state = 3;
  } else if (global_flight_state.flight_state == DROGUE) {
    tx_payload->state = 4;
  } else if (global_flight_state.flight_state == MAIN) {
    tx_payload->state = 5;
  } else if (global_flight_state.flight_state == TOUCHDOWN) {
    tx_payload->state = 6;
  }
  tx_payload->timestamp = ts / 100;
  tx_payload->altitude = (int32_t)global_estimation_data.height;
  tx_payload->velocity = (int16_t)global_estimation_data.velocity;
  tx_payload->lat = (int32_t)(gnss->position.lat * 10000);
  tx_payload->lon = (int32_t)(gnss->position.lon * 10000);
}

void parse_tx_msg(packed_tx_msg_t* rx_payload) {
  if (rx_payload->d1 == 0xAA && rx_payload->d2 == 0xBB && rx_payload->d3 == 0xCC) {
    global_arming_bool = true;
    log_info("ARM");
  } else {
    global_arming_bool = false;
  }
}

[[noreturn]] void task_telemetry(__attribute__((unused)) void* argument) {
  /* Give the telemetry hardware some time to initialize */
  osDelay(5000);
  // global_arming_bool = true;

  /* Configure the telemetry MCU */
  send_setting(CMD_DIRECTION, TX);
  osDelay(100);
  send_setting(CMD_POWER_LEVEL, global_cats_config.config.telemetry_settings.power_level);
  osDelay(100);
  send_setting(CMD_MODE, BIDIRECTIONAL);
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

  gnss_data_t gnss_data = {};
  bool gnss_position_received = false;

  /* local fsm enum */
  flight_fsm_e new_fsm_enum = MOVING;
  flight_fsm_e old_fsm_enum = MOVING;

  packed_tx_msg_t tx_payload = {};

  uint32_t uart_timeout = osKernelGetTickCount();

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / TELEMETRY_SAMPLING_FREQ;
  while (1) {
    pack_tx_msg(tick_count, &gnss_data, &tx_payload);
    send_tx_payload((uint8_t*)&tx_payload, 16);

    if ((tick_count - uart_timeout) > 60000) {
      uart_timeout = tick_count;
      HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, (uint8_t*)&uart_char, 1);
    }

    /* Check for data from the Telemetry MCU */
    while (stream_length(&uart_stream) > 1) {
      uint8_t ch;
      uart_timeout = tick_count;
      stream_read_byte(&uart_stream, &ch);
      switch (state) {
        case STATE_OP:
          valid_op = check_valid_op_code(ch);
          if (valid_op) {
            uart_buffer[INDEX_OP] = ch;
            state = STATE_LEN;
          }
          break;
        case STATE_LEN:
          if (ch <= 16) {
            uart_buffer[INDEX_LEN] = ch;
            if (ch > 0) {
              state = STATE_DATA;
            } else {
              state = STATE_CRC;
            }
          }
          break;
        case STATE_DATA:
          if ((uart_buffer[1] - uart_index) > 0) {
            uart_buffer[uart_index + 2] = ch;
            uart_index++;
          }
          if ((uart_buffer[INDEX_LEN] - uart_index) == 0) {
            state = STATE_CRC;
          }
          break;
        case STATE_CRC: {
          uint8_t crc;
          crc = crc8(uart_buffer, uart_index + 2);
          if (crc == ch) {
            gnss_position_received = parse(uart_buffer[INDEX_OP], &uart_buffer[2], uart_buffer[INDEX_LEN], &gnss_data);
          }
          uart_index = 0;
          state = STATE_OP;
        } break;
        default:
          break;
      }
    }

    /* Log GNSS data if we received it in this iteration. */
    if (gnss_position_received) {
      record(tick_count, GNSS_INFO, &(gnss_data.position));
      gnss_position_received = false;
    }

    new_fsm_enum = global_flight_state.flight_state;

    /* Log GNSS time when changing to THRUSTING. */
    if ((new_fsm_enum != old_fsm_enum) && (new_fsm_enum == THRUSTING)) {
      /* Time will be 0 if it was never received. */
      /* TODO: Keep track of the last timestamp when the GNSS time was received and add the difference between that and
       * current one to the GNSS time. This should be done when the date information is also sent via UART. */
      log_info("Logging GNSS Time: %02hu:%02hu:%02hu UTC", gnss_data.time.hour, gnss_data.time.min, gnss_data.time.sec);
      global_flight_stats.liftoff_time = gnss_data.time;
    }

    /* Go to high power mode if adaptive power is enabled */
    if (global_cats_config.config.telemetry_settings.adaptive_power == ON) {
      if ((new_fsm_enum != old_fsm_enum) && (new_fsm_enum == THRUSTING)) {
        send_setting(CMD_POWER_LEVEL, TELE_MAX_POWER);
      }
      if ((new_fsm_enum != old_fsm_enum) && (new_fsm_enum == TOUCHDOWN)) {
        send_setting(CMD_POWER_LEVEL, global_cats_config.config.telemetry_settings.power_level);
      }
    }

    old_fsm_enum = new_fsm_enum;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

bool check_valid_op_code(uint8_t op_code) {
  /* TODO loop over all opcodes and check if it exists */
  if (op_code == CMD_GNSS_INFO || op_code == CMD_GNSS_LOC || op_code == CMD_RX || op_code == CMD_INFO ||
      op_code == CMD_GNSS_TIME || op_code == CMD_TEMP_INFO) {
    return true;
  } else {
    return false;
  }
}

/**
 * Parse telemetry message.
 *
 * @param op_code [in]
 * @param buffer [in]
 * @param length [in]
 * @param gnss [out]
 * @return
 */
bool parse(uint8_t op_code, const uint8_t* buffer, uint32_t length, gnss_data_t* gnss) {
  if (length < 1) return false;

  bool gnss_position_received = false;

  if (op_code == CMD_RX) {
    packed_tx_msg_t rx_payload;
    memcpy(&rx_payload, buffer, length);
    parse_tx_msg(&rx_payload);
    // log_info("RX received");
  } else if (op_code == CMD_INFO) {
    // log_info("Link Info received");
  } else if (op_code == CMD_GNSS_LOC) {
    gnss_position_received = true;
    memcpy(&(gnss->position.lat), buffer, 4);
    memcpy(&(gnss->position.lon), &buffer[4], 4);
    log_info("[GNSS location]: LAT: %f, LON: %f", (double)gnss->position.lat, (double)gnss->position.lon);
  } else if (op_code == CMD_GNSS_INFO) {
    gnss_position_received = true;
    gnss->position.sats = buffer[0];
    log_info("[GNSS info]: sats: %u", gnss->position.sats);
  } else if (op_code == CMD_GNSS_TIME) {
    gnss->time = (gnss_time_t){.hour = buffer[2], .min = buffer[1], .sec = buffer[0]};
    log_info("[GNSS time]: %02hu:%02hu:%02hu UTC", gnss->time.hour, gnss->time.min, gnss->time.sec);
  } else if (op_code == CMD_TEMP_INFO) {
    memcpy(&amplifier_temperature, buffer, 4);
  } else {
    log_error("Unknown Op Code");
  }

  return gnss_position_received;
}

void send_link_phrase(uint8_t* phrase, uint32_t length) {
  uint8_t out[11];  // 1 OP + 1 LEN + 8 DATA + 1 CRC
  out[0] = CMD_LINK_PHRASE;
  out[1] = (uint8_t)length;
  memcpy(&out[2], phrase, length);
  out[length + 2] = crc8(out, length + 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, length + 3, 2);
}

void send_setting(uint8_t command, uint8_t value) {
  uint8_t out[4];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = command;
  out[1] = 1;
  out[2] = value;
  out[3] = crc8(out, 3);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 4, 2);
}

void send_enable() {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = CMD_ENABLE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void send_disable() {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = CMD_DISBALE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void send_tx_payload(uint8_t* payload, uint32_t length) {
  uint8_t out[19];  // 1 OP + 1 LEN + 16 DATA + 1 CRC
  out[0] = CMD_TX;
  out[1] = (uint8_t)length;
  memcpy(&out[2], payload, length);
  out[length + 2] = crc8(out, length + 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, length + 3, 2);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &TELEMETRY_UART_HANDLE) {
    uint8_t tmp = uart_char;
    HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, &uart_char, 1);
    stream_write_byte(&uart_stream, tmp);
  }
}
