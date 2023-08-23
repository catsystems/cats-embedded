/******************************************************************************
 * file    USBCDC.cpp
 *******************************************************************************
 * brief   High-Level Wrapper for USB CDC (virtual COM-Port) interface
 *******************************************************************************
 * author  Espressif Systems & Florian Baumgartner
 * version 1.0
 * date    2022-08-02
 *******************************************************************************
 * Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "USB.h"
#if CONFIG_TINYUSB_CDC_ENABLED

#include "USBCDC.h"
#include "esp32-hal-tinyusb.h"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
ESP_EVENT_DEFINE_BASE(ARDUINO_USB_CDC_EVENTS);
esp_err_t arduino_usb_event_post(esp_event_base_t event_base, int32_t event_id, void *event_data,
                                 size_t event_data_size, TickType_t ticks_to_wait);
esp_err_t arduino_usb_event_handler_register_with(esp_event_base_t event_base, int32_t event_id,
                                                  esp_event_handler_t event_handler, void *event_handler_arg);

constexpr uint8_t kMaxUsbCdcDevices = 2;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
USBCDC *devices[kMaxUsbCdcDevices] = {nullptr, nullptr};

static uint16_t load_cdc_descriptor(uint8_t *dst, uint8_t *itf) {
  const uint8_t str_index = tinyusb_add_string_descriptor("TinyUSB CDC");
  uint8_t descriptor[TUD_CDC_DESC_LEN] = {
      // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
      // NOLINTNEXTLINE(hicpp-signed-bitwise)
      TUD_CDC_DESCRIPTOR(*itf, str_index, 0x85, 64, 0x03, 0x84, 64)};
  *itf += 2;
  memcpy(dst, descriptor, TUD_CDC_DESC_LEN);
  return TUD_CDC_DESC_LEN;
}

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  if (itf < kMaxUsbCdcDevices && devices[itf] != nullptr) {
    devices[itf]->_onLineState(dtr, rts);
  }
}

// Invoked when line coding is change via SET_LINE_CODING
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding) {
  if (itf < kMaxUsbCdcDevices && devices[itf] != nullptr) {
    devices[itf]->_onLineCoding(p_line_coding->bit_rate, p_line_coding->stop_bits, p_line_coding->parity,
                                p_line_coding->data_bits);
  }
}

// Invoked when received new data
void tud_cdc_rx_cb(uint8_t itf) {
  if (itf < kMaxUsbCdcDevices && devices[itf] != nullptr) {
    devices[itf]->_onRX();
  }
}

// Invoked when received send break
void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms) {
  // log_v("itf: %u, duration_ms: %u", itf, duration_ms);
}

// Invoked when space becomes available in TX buffer
void tud_cdc_tx_complete_cb(uint8_t itf) {
  if (itf < kMaxUsbCdcDevices && devices[itf] != nullptr) {
    devices[itf]->_onTX();
  }
}

static void ARDUINO_ISR_ATTR cdc0_write_char(char c) {
  if (devices[0] != nullptr) {
    devices[0]->write(c);
  }
}

static void usb_unplugged_cb(void *arg, esp_event_base_t event_base [[maybe_unused]], int32_t event_id [[maybe_unused]],
                             void *event_data [[maybe_unused]]) {
  static_cast<USBCDC *>(arg)->_onUnplugged();
}

USBCDC::USBCDC(uint8_t itfn)
    : itf(itfn),
      bit_rate(0),
      stop_bits(0),
      parity(0),
      data_bits(0),
      dtr(false),
      rts(false),
      connected(false),
      reboot_enable(true),
      rx_queue(nullptr),
      tx_lock(nullptr),
      tx_timeout_ms(250) {
  tinyusb_enable_interface(USB_INTERFACE_CDC, TUD_CDC_DESC_LEN, load_cdc_descriptor);
  if (itf < kMaxUsbCdcDevices) {
    arduino_usb_event_handler_register_with(ARDUINO_USB_EVENTS, ARDUINO_USB_STOPPED_EVENT, usb_unplugged_cb, this);
  }
}

USBCDC::~USBCDC() { end(); }

void USBCDC::onEvent(esp_event_handler_t callback) { onEvent(ARDUINO_USB_CDC_ANY_EVENT, callback); }
void USBCDC::onEvent(arduino_usb_cdc_event_t event, esp_event_handler_t callback) {
  arduino_usb_event_handler_register_with(ARDUINO_USB_CDC_EVENTS, event, callback, this);
}

size_t USBCDC::setRxBufferSize(size_t size) {
  const size_t currentQueueSize =
      rx_queue != nullptr ? uxQueueSpacesAvailable(rx_queue) + uxQueueMessagesWaiting(rx_queue) : 0;

  if (size != currentQueueSize) {
    xQueueHandle new_rx_queue = nullptr;
    if (size > 0) {
      new_rx_queue = xQueueCreate(size, sizeof(uint8_t));
      if (new_rx_queue == nullptr) {
        log_e("CDC Queue creation failed.");
        return 0;
      }
      if (rx_queue != nullptr) {
        const size_t copySize = uxQueueMessagesWaiting(rx_queue);
        if (copySize > 0) {
          for (size_t i = 0; i < copySize; i++) {
            uint8_t ch = 0;
            xQueueReceive(rx_queue, &ch, 0);
            if (!xQueueSend(new_rx_queue, &ch, 0)) {
              arduino_usb_cdc_event_data_t p;
              p.rx_overflow.dropped_bytes = copySize - i;
              arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_RX_OVERFLOW_EVENT, &p,
                                     sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
              log_e("CDC RX Overflow.");
              break;
            }
          }
        }
        vQueueDelete(rx_queue);
      }
      rx_queue = new_rx_queue;
      return size;
      // NOLINTNEXTLINE(readability-else-after-return) more understandable like this
    } else {
      if (rx_queue != nullptr) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
      }
    }
  }
  return size;
}

void USBCDC::begin(uint32_t baud [[maybe_unused]]) {
  if (tx_lock == nullptr) {
    tx_lock = xSemaphoreCreateMutex();
  }
  // if rx_queue was set before begin(), keep it
  if (rx_queue == nullptr) {
    setRxBufferSize(256);  // default if not preset
  }
  devices[itf] = this;
}

void USBCDC::end() {
  connected = false;
  devices[itf] = nullptr;
  setRxBufferSize(0);
  if (tx_lock != nullptr) {
    vSemaphoreDelete(tx_lock);
    tx_lock = nullptr;
  }
}

void USBCDC::setTxTimeoutMs(uint32_t timeout) { tx_timeout_ms = timeout; }

void USBCDC::_onUnplugged() {
  if (connected) {
    connected = false;
    dtr = false;
    rts = false;
    arduino_usb_cdc_event_data_t p;
    arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_DISCONNECTED_EVENT, &p,
                           sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
  }
}

enum { CDC_LINE_IDLE, CDC_LINE_1, CDC_LINE_2, CDC_LINE_3 };
void USBCDC::_onLineState(bool _dtr, bool _rts) {
  static uint8_t lineState = CDC_LINE_IDLE;

  if (dtr == _dtr && rts == _rts) {
    return;  // Skip duplicate events
  }

  dtr = _dtr;
  rts = _rts;

  if (reboot_enable) {
    if (!dtr && rts) {
      if (lineState == CDC_LINE_IDLE) {
        lineState++;
        if (connected) {
          connected = false;
          arduino_usb_cdc_event_data_t p;
          arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_DISCONNECTED_EVENT, &p,
                                 sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
        }
      } else {
        lineState = CDC_LINE_IDLE;
      }
    } else if (dtr && rts) {
      if (lineState == CDC_LINE_1) {
        lineState++;
      } else {
        lineState = CDC_LINE_IDLE;
      }
    } else if (dtr && !rts) {
      if (lineState == CDC_LINE_2) {
        lineState++;
      } else {
        lineState = CDC_LINE_IDLE;
      }
    } else if (!dtr && !rts) {
      if (lineState == CDC_LINE_3) {
        usb_persist_restart(RESTART_BOOTLOADER);
      } else {
        lineState = CDC_LINE_IDLE;
      }
    }
  }

  // *********************************************************************************************************************************************
  // Florian Baumgartner - 02.08.2022
  // Removed line idle check since this causes connection state bug
  // *********************************************************************************************************************************************
  // if(lineState == CDC_LINE_IDLE){
  if (dtr && rts && !connected) {
    connected = true;
    arduino_usb_cdc_event_data_t p;
    arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_CONNECTED_EVENT, &p,
                           sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
  } else if (!dtr && connected) {
    connected = false;
    arduino_usb_cdc_event_data_t p;
    arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_DISCONNECTED_EVENT, &p,
                           sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
  }
  arduino_usb_cdc_event_data_t l;
  l.line_state.dtr = dtr;
  l.line_state.rts = rts;
  arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_LINE_STATE_EVENT, &l,
                         sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
  //}
}

void USBCDC::_onLineCoding(uint32_t _bit_rate, uint8_t _stop_bits, uint8_t _parity, uint8_t _data_bits) {
  if (bit_rate != _bit_rate || data_bits != _data_bits || stop_bits != _stop_bits || parity != _parity) {
    // ArduinoIDE sends LineCoding with 1200bps baud to reset the device
    if (reboot_enable && _bit_rate == 1200) {
      usb_persist_restart(RESTART_BOOTLOADER);
    } else {
      bit_rate = _bit_rate;
      data_bits = _data_bits;
      stop_bits = _stop_bits;
      parity = _parity;
      arduino_usb_cdc_event_data_t p;
      p.line_coding.bit_rate = bit_rate;
      p.line_coding.data_bits = data_bits;
      p.line_coding.stop_bits = stop_bits;
      p.line_coding.parity = parity;
      arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_LINE_CODING_EVENT, &p,
                             sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
    }
  }
}

void USBCDC::_onRX() {
  arduino_usb_cdc_event_data_t p;
  uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
  uint32_t count = tud_cdc_n_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE);
  for (uint32_t i = 0; i < count; i++) {
    if (rx_queue == nullptr || !xQueueSend(rx_queue, buf + i, 10)) {
      p.rx_overflow.dropped_bytes = count - i;
      arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_RX_OVERFLOW_EVENT, &p,
                             sizeof(arduino_usb_cdc_event_data_t), portMAX_DELAY);
      log_e("CDC RX Overflow.");
      count = i;
      break;
    }
  }
  if (count != 0) {
    p.rx.len = count;
    arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_RX_EVENT, &p, sizeof(arduino_usb_cdc_event_data_t),
                           portMAX_DELAY);
  }
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void USBCDC::_onTX() {
  arduino_usb_cdc_event_data_t p;
  arduino_usb_event_post(ARDUINO_USB_CDC_EVENTS, ARDUINO_USB_CDC_TX_EVENT, &p, sizeof(arduino_usb_cdc_event_data_t),
                         portMAX_DELAY);
}

void USBCDC::enableReboot(bool enable) { reboot_enable = enable; }

// NOLINTNEXTLINE(readability-make-member-function-const)
bool USBCDC::rebootEnabled() { return reboot_enable; }

int USBCDC::available() {
  if (itf >= kMaxUsbCdcDevices || rx_queue == nullptr) {
    return -1;
  }
  return static_cast<int>(uxQueueMessagesWaiting(rx_queue));
}

int USBCDC::peek() {
  if (itf >= kMaxUsbCdcDevices || rx_queue == nullptr) {
    return -1;
  }
  uint8_t c = 0;
  if (xQueuePeek(rx_queue, &c, 0) != 0) {
    return c;
  }
  return -1;
}

int USBCDC::read() {
  if (itf >= kMaxUsbCdcDevices || rx_queue == nullptr) {
    return -1;
  }
  uint8_t c = 0;
  if (xQueueReceive(rx_queue, &c, 0) != 0) {
    return c;
  }
  return -1;
}

size_t USBCDC::read(uint8_t *buffer, size_t size) {
  if (itf >= kMaxUsbCdcDevices || rx_queue == nullptr) {
    return -1;
  }
  uint8_t c = 0;
  size_t count = 0;
  while ((count < size) && (xQueueReceive(rx_queue, &c, 0) != 0)) {
    buffer[count++] = c;
  }
  return count;
}

void USBCDC::flush() {
  if (itf >= kMaxUsbCdcDevices || tx_lock == nullptr || !tud_cdc_n_connected(itf)) {
    return;
  }
  if (xSemaphoreTake(tx_lock, tx_timeout_ms / portTICK_PERIOD_MS) != pdPASS) {
    return;
  }
  tud_cdc_n_write_flush(itf);
  xSemaphoreGive(tx_lock);
}

int USBCDC::availableForWrite() {
  if (itf >= kMaxUsbCdcDevices || tx_lock == nullptr || !tud_cdc_n_connected(itf)) {
    return 0;
  }
  if (xSemaphoreTake(tx_lock, tx_timeout_ms / portTICK_PERIOD_MS) != pdPASS) {
    return 0;
  }
  const size_t a = tud_cdc_n_write_available(itf);
  xSemaphoreGive(tx_lock);
  return static_cast<int>(a);
}

size_t USBCDC::write(const uint8_t *buffer, size_t size) {
  if (itf >= kMaxUsbCdcDevices || tx_lock == nullptr || buffer == nullptr || size == 0 || !tud_cdc_n_connected(itf)) {
    return 0;
  }
  if (xPortInIsrContext() != 0) {
    BaseType_t taskWoken = 0;
    if (xSemaphoreTakeFromISR(tx_lock, &taskWoken) != pdPASS) {
      return 0;
    }
  } else if (xSemaphoreTake(tx_lock, tx_timeout_ms / portTICK_PERIOD_MS) != pdPASS) {
    return 0;
  }
  size_t to_send = size;
  size_t so_far = 0;
  while (to_send > 0) {
    if (!tud_cdc_n_connected(itf)) {
      size = so_far;
      break;
    }
    size_t space = tud_cdc_n_write_available(itf);
    if (space == 0) {
      tud_cdc_n_write_flush(itf);
      continue;
    }
    if (space > to_send) {
      space = to_send;
    }
    const size_t sent = tud_cdc_n_write(itf, buffer + so_far, space);
    if (sent > 0) {
      so_far += sent;
      to_send -= sent;
      tud_cdc_n_write_flush(itf);
    } else {
      size = so_far;
      break;
    }
  }
  if (xPortInIsrContext() != 0) {
    BaseType_t taskWoken = 0;
    xSemaphoreGiveFromISR(tx_lock, &taskWoken);
  } else {
    xSemaphoreGive(tx_lock);
  }
  return size;
}

size_t USBCDC::write(uint8_t c) { return write(&c, 1); }

// NOLINTNEXTLINE(readability-make-member-function-const)
uint32_t USBCDC::baudRate() { return bit_rate; }

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
void USBCDC::setDebugOutput(bool en) {
  if (en) {
    uartSetDebug(nullptr);
    // NOLINTNEXTLINE(google-readability-casting) no idea what this does
    ets_install_putc1((void (*)(char)) & cdc0_write_char);
  } else {
    ets_install_putc1(nullptr);
  }
}

USBCDC::operator bool() const {
  if (itf >= kMaxUsbCdcDevices) {
    return false;
  }
  return connected;
}

#if ARDUINO_USB_CDC_ON_BOOT && !ARDUINO_USB_MODE  // Serial used for USB CDC
USBCDC Serial(0);
#endif

#endif /* CONFIG_TINYUSB_CDC_ENABLED */
