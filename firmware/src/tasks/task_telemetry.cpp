/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include "tasks/task_telemetry.hpp"
#include "comm/fifo.hpp"
#include "comm/stream.hpp"
#include "config/cats_config.hpp"
#include "config/globals.hpp"
#include "drivers/adc.hpp"
#include "tasks/task_peripherals.hpp"
#include "util/battery.hpp"
#include "util/crc.hpp"
#include "util/gnss.hpp"
#include "util/log.h"
#include "util/task_util.hpp"
#include "util/telemetry_reg.hpp"

namespace task {

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

void Telemetry::PackTxMessage(uint32_t ts, gnss_data_t* gnss, packed_tx_msg_t* tx_payload,
                              estimation_output_t estimation_data) const noexcept {
  static_assert(sizeof(packed_tx_msg_t) == 15);
  if (m_fsm_enum > INVALID) {
    tx_payload->state = m_fsm_enum;
  }

  /* If we are in testing mode, most of the tx_payload is not used.
   * Hence, use the state to inform the groundstation by setting the state
   * if the flight computer was armed or not through telemetry */
  if (m_testing_enabled) {
    tx_payload->state = static_cast<uint8_t>(m_testing_armed);
  }

  tx_payload->timestamp = ts / 100;

  if (get_error_by_tag(CATS_ERR_NON_USER_CFG)) {
    tx_payload->errors |= 0b00'00'01U;
  }

  if (get_error_by_tag(CATS_ERR_LOG_FULL)) {
    tx_payload->errors |= 0b00'00'10U;
  }

  if (get_error_by_tag(CATS_ERR_FILTER_ACC) || get_error_by_tag(CATS_ERR_FILTER_HEIGHT)) {
    tx_payload->errors |= 0b00'01'00U;
  }

  if (get_error_by_tag(CATS_ERR_TELEMETRY_HOT)) {
    tx_payload->errors |= 0b00'10'00U;
  }

  if (get_error_by_tag(CATS_ERR_NO_PYRO)) {
    tx_payload->errors |= 0b01'00'00U;
  }

  tx_payload->lat = static_cast<int32_t>(gnss->position.lat * 10000);
  tx_payload->lon = static_cast<int32_t>(gnss->position.lon * 10000);
  tx_payload->testing_on = m_testing_enabled;

  tx_payload->altitude = static_cast<int32_t>(estimation_data.height);
  tx_payload->velocity = static_cast<int16_t>(estimation_data.velocity);

  tx_payload->voltage = battery_voltage_byte();

  if (adc_get(ADC_PYRO1) > 500) {
    tx_payload->pyro_continuity |= 0b01U;
  }
  if (adc_get(ADC_PYRO2) > 500) {
    tx_payload->pyro_continuity |= 0b10U;
  }
}

void Telemetry::ParseRxMessage(packed_rx_msg_t* rx_payload) noexcept {
  /* Check if Correct Header */
  if (rx_payload->header != RX_PACKET_HEADER) {
    return;
  }

  /* Check if the linkphrase matches */
  if (rx_payload->passcode != m_test_phrase_crc) {
    return;
  }

  /* Packet Received, reset timeout */
  m_testing_timeout = osKernelGetTickCount();

  /* If the testing is armed, arm pyros */
  if (rx_payload->enable_testing_telemetry) {
    HAL_GPIO_WritePin(PYRO_EN_GPIO_Port, PYRO_EN_Pin, GPIO_PIN_SET);
    m_testing_armed = true;
  } else {
    HAL_GPIO_WritePin(PYRO_EN_GPIO_Port, PYRO_EN_Pin, GPIO_PIN_RESET);
    m_testing_armed = false;
  }

  /* Check if we received a zero; if we do, a new event can be triggered */
  if (rx_payload->event == 0) {
    m_event_reset = true;
  }

  /* Add event to eventqueue if its a valid event, if the flight computer is armed and if the last triggered event was
   * cleared by the groundstation */
  if ((rx_payload->event <= EV_CUSTOM_2) && (rx_payload->event > EV_CALIBRATE) && m_testing_armed && m_event_reset) {
    trigger_event(static_cast<cats_event_e>(rx_payload->event), false);
    /* Event thrown, set boolean to false and wait until groundstation clears the event again before triggering new
     * events */
    m_event_reset = false;
  }
}

[[noreturn]] void Telemetry::Run() noexcept {
  /* Give the telemetry hardware some time to initialize */
  osDelay(5000);

  /* Check if we are in testing mode */
  bool testing_enabled = false;
  /* Check if valid link parameters are set for testing mode to be enabled */
  /* If no test phrase is set, dont allow testing mode */
  if (global_cats_config.telemetry_settings.test_phrase[0] != 0) {
    testing_enabled = global_cats_config.enable_testing_mode;
  }

  /* Configure the telemetry MCU */
  SendSettings(CMD_DIRECTION, TX);
  osDelay(100);
  SendSettings(CMD_POWER_LEVEL, global_cats_config.telemetry_settings.power_level);
  osDelay(100);

  /* if we are in the testing mode, set the receiver to bidirectional mode */
  if (testing_enabled) {
    SendSettings(CMD_MODE, BIDIRECTIONAL);
    m_test_phrase_crc = crc32(global_cats_config.telemetry_settings.test_phrase, 8);
  } else {
    SendSettings(CMD_MODE, UNIDIRECTIONAL);
  }
  osDelay(100);

  /* Only start the telemetry when a link phrase is set */
  if (global_cats_config.telemetry_settings.link_phrase[0] != 0) {
    SendLinkPhrase(global_cats_config.telemetry_settings.link_phrase, 8);
    osDelay(100);
    SendEnable();
  }

  /* Start the interrupt request for the UART */
  HAL_UART_Receive_IT(&TELEMETRY_UART_HANDLE, (uint8_t*)&uart_char, 1);

  uint8_t uart_buffer[20];
  uint32_t uart_index = 0;
  state_e state = STATE_OP;
  bool valid_op = false;

  gnss_data_t gnss_data = {};
  bool gnss_position_received = false;

  uint32_t uart_timeout = osKernelGetTickCount();

  uint32_t tick_count = osKernelGetTickCount();
  constexpr uint32_t tick_update = sysGetTickFreq() / TELEMETRY_SAMPLING_FREQ;
  while (true) {
    /* Get new FSM enum */
    bool fsm_updated = GetNewFsmEnum();
    packed_tx_msg_t tx_payload = {};
    estimation_output_t estimation_output = {};
    if (m_task_state_estimation != nullptr) {
      estimation_output = m_task_state_estimation->GetEstimationOutput();
    }

    PackTxMessage(tick_count, &gnss_data, &tx_payload, estimation_output);

    SendTxPayload((uint8_t*)&tx_payload, sizeof(packed_tx_msg_t));

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
          valid_op = CheckValidOpCode(ch);
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
          uint8_t crc = crc8(uart_buffer, uart_index + 2);
          if (crc == ch) {
            gnss_position_received = Parse(uart_buffer[INDEX_OP], &uart_buffer[2], uart_buffer[INDEX_LEN], &gnss_data);
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

    /* Log GNSS time when changing to THRUSTING. */
    if (fsm_updated && (m_fsm_enum == THRUSTING)) {
      /* Time will be 0 if it was never received. */
      /* TODO: Keep track of the last timestamp when the GNSS time was received and add the difference between that and
       * current one to the GNSS time. This should be done when the date information is also sent via UART. */
      log_info("Logging GNSS Time: %02hu:%02hu:%02hu UTC", gnss_data.time.hour, gnss_data.time.min, gnss_data.time.sec);
      global_flight_stats.liftoff_time = gnss_data.time;
    }

    /* Go to high power mode if adaptive power is enabled */
    if (global_cats_config.telemetry_settings.adaptive_power == ON) {
      if (fsm_updated && (m_fsm_enum == THRUSTING)) {
        SendSettings(CMD_POWER_LEVEL, TELE_MAX_POWER);
      }
      if (fsm_updated && (m_fsm_enum == TOUCHDOWN)) {
        SendSettings(CMD_POWER_LEVEL, global_cats_config.telemetry_settings.power_level);
      }
    }

    RequestVersionNum();

    /* Disable the pyro channels if we are in testing mode and the timeout is achieved */
    if ((osKernelGetTickCount() - m_testing_timeout) > 2000 && testing_enabled) {
      HAL_GPIO_WritePin(PYRO_EN_GPIO_Port, PYRO_EN_Pin, GPIO_PIN_RESET);
      m_testing_armed = false;
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

bool Telemetry::CheckValidOpCode(uint8_t op_code) const noexcept {
  /* TODO loop over all opcodes and check if it exists */
  if (op_code == CMD_GNSS_INFO || op_code == CMD_GNSS_LOC || op_code == CMD_RX || op_code == CMD_INFO ||
      op_code == CMD_GNSS_TIME || op_code == CMD_TEMP_INFO || op_code == CMD_VERSION_INFO) {
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
bool Telemetry::Parse(uint8_t op_code, const uint8_t* buffer, uint32_t length, gnss_data_t* gnss) noexcept {
  if (length < 1) return false;

  bool gnss_position_received = false;

  if (op_code == CMD_RX) {
    packed_rx_msg_t rx_payload{};
    memcpy(&rx_payload, buffer, length);
    ParseRxMessage(&rx_payload);
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
    memcpy(const_cast<float32_t*>(&m_amplifier_temperature), buffer, 4);
    if (m_amplifier_temperature > k_amplifier_hot_limit) {
      add_error(CATS_ERR_TELEMETRY_HOT);
    }
  } else if (op_code == CMD_VERSION_INFO) {
    for (uint32_t i = 0; i < length; i++) {
      telemetry_code_version[i] = static_cast<char>(buffer[i]);
    }

  } else {
    log_error("Unknown Op Code");
  }

  return gnss_position_received;
}

void Telemetry::SendLinkPhrase(uint8_t* phrase, uint32_t length) noexcept {
  uint32_t uplink_phrase_crc = crc32(global_cats_config.telemetry_settings.link_phrase, length);
  uint8_t out[7];  // 1 OP + 1 LEN + 4 DATA + 1 CRC
  out[0] = CMD_LINK_PHRASE;
  out[1] = 4;
  memcpy(&out[2], &uplink_phrase_crc, 4);
  out[6] = crc8(out, 6);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 7, 2);
}

void Telemetry::SendSettings(uint8_t command, uint8_t value) noexcept {
  uint8_t out[4];  // 1 OP + 1 LEN + 1 DATA + 1 CRC
  out[0] = command;
  out[1] = 1;
  out[2] = value;
  out[3] = crc8(out, 3);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 4, 2);
}

void Telemetry::RequestVersionNum() noexcept {
  /* Return if the code version was fetched */
  if (strlen(telemetry_code_version) > 0) {
    return;
  }
  uint8_t out[3];  // 1 OP + 1 LEN + 1 CRC
  out[0] = CMD_VERSION_INFO;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void Telemetry::SendEnable() noexcept {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 CRC
  out[0] = CMD_ENABLE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void Telemetry::SendDisable() noexcept {
  uint8_t out[3];  // 1 OP + 1 LEN + 1 CRC
  out[0] = CMD_DISABLE;
  out[1] = 0;
  out[2] = crc8(out, 2);

  HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, out, 3, 2);
}

void Telemetry::SendTxPayload(uint8_t* payload, uint32_t length) noexcept {
  uint8_t out[18];  // 1 OP + 1 LEN + 15 DATA + 1 CRC
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

}  // namespace task
