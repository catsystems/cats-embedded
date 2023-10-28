/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

/// High-Level Wrapper for native USB support (device only)

#include "USB.h"

#if CONFIG_TINYUSB_ENABLED

#include "common/tusb_common.h"
#include "esp32-hal-tinyusb.h"
#include "esp32-hal.h"
#include "esp_private/system_internal.h"  // Needed for UF2-Bootloader hint: esp_reset_reason_set_hint()
#include "pins_arduino.h"

#ifndef USB_VID
#define USB_VID USB_ESPRESSIF_VID
#endif
#ifndef USB_PID
#define USB_PID 0x0002
#endif
#ifndef USB_MANUFACTURER
#define USB_MANUFACTURER "Espressif Systems"
#endif
#ifndef USB_PRODUCT
#define USB_PRODUCT ARDUINO_BOARD
#endif
#ifndef USB_SERIAL
#define USB_SERIAL "0"
#endif
#ifndef USB_WEBUSB_ENABLED
#define USB_WEBUSB_ENABLED false
#endif
#ifndef USB_WEBUSB_URL
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define USB_WEBUSB_URL "https://espressif.github.io/arduino-esp32/webusb.html"
#endif

#if CFG_TUD_DFU_RUNTIME
static uint16_t load_dfu_descriptor(uint8_t *dst, uint8_t *itf) {
#define DFU_ATTRS (DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_CAN_UPLOAD | DFU_ATTR_MANIFESTATION_TOLERANT)

  const uint8_t str_index = tinyusb_add_string_descriptor("TinyUSB DFU_RT");
  uint8_t descriptor[TUD_DFU_RT_DESC_LEN] = {
      // Interface number, string index, attributes, detach timeout, transfer size */
      // NOLINTNEXTLINE(hicpp-signed-bitwise)
      TUD_DFU_RT_DESCRIPTOR(*itf, str_index, DFU_ATTRS, 700U, 64U)};
  *itf += 1;
  memcpy(dst, descriptor, TUD_DFU_RT_DESC_LEN);
  return TUD_DFU_RT_DESC_LEN;
}
// Invoked on DFU_DETACH request to reboot to the bootloader
void tud_dfu_runtime_reboot_to_dfu_cb() {
  // MODIFICATION: If DFU Detach request gets received, UF2-Bootloader instead of ROM-Bootloader gets launched!

  // usb_persist_restart(RESTART_BOOTLOADER_DFU);
  const uint16_t APP_REQUEST_UF2_RESET_HINT = 0x11F2;
  esp_reset_reason();
  esp_reset_reason_set_hint(static_cast<esp_reset_reason_t>(APP_REQUEST_UF2_RESET_HINT));
  esp_restart();
}
#endif /* CFG_TUD_DFU_RUNTIME */

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
ESP_EVENT_DEFINE_BASE(ARDUINO_USB_EVENTS);
static esp_event_loop_handle_t arduino_usb_event_loop_handle = nullptr;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

esp_err_t arduino_usb_event_post(esp_event_base_t event_base, int32_t event_id, void *event_data,
                                 size_t event_data_size, TickType_t ticks_to_wait) {
  if (arduino_usb_event_loop_handle == nullptr) {
    return ESP_FAIL;
  }
  return esp_event_post_to(arduino_usb_event_loop_handle, event_base, event_id, event_data, event_data_size,
                           ticks_to_wait);
}
esp_err_t arduino_usb_event_handler_register_with(esp_event_base_t event_base, int32_t event_id,
                                                  esp_event_handler_t event_handler, void *event_handler_arg) {
  if (arduino_usb_event_loop_handle == nullptr) {
    return ESP_FAIL;
  }
  return esp_event_handler_register_with(arduino_usb_event_loop_handle, event_base, event_id, event_handler,
                                         event_handler_arg);
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static bool tinyusb_device_mounted = false;
static bool tinyusb_device_suspended = false;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// Invoked when device is mounted (configured)
void tud_mount_cb() {
  tinyusb_device_mounted = true;
  arduino_usb_event_data_t p;
  arduino_usb_event_post(ARDUINO_USB_EVENTS, ARDUINO_USB_STARTED_EVENT, &p, sizeof(arduino_usb_event_data_t),
                         portMAX_DELAY);
}

// Invoked when device is unmounted
void tud_umount_cb() {
  tinyusb_device_mounted = false;
  arduino_usb_event_data_t p;
  arduino_usb_event_post(ARDUINO_USB_EVENTS, ARDUINO_USB_STOPPED_EVENT, &p, sizeof(arduino_usb_event_data_t),
                         portMAX_DELAY);
}

// Invoked when usb bus is suspended
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  tinyusb_device_suspended = true;
  arduino_usb_event_data_t p;
  p.suspend.remote_wakeup_en = remote_wakeup_en;
  arduino_usb_event_post(ARDUINO_USB_EVENTS, ARDUINO_USB_SUSPEND_EVENT, &p, sizeof(arduino_usb_event_data_t),
                         portMAX_DELAY);
}

// Invoked when usb bus is resumed
void tud_resume_cb() {
  tinyusb_device_suspended = false;
  arduino_usb_event_data_t p;
  arduino_usb_event_post(ARDUINO_USB_EVENTS, ARDUINO_USB_RESUME_EVENT, &p, sizeof(arduino_usb_event_data_t),
                         portMAX_DELAY);
}

ESPUSB::ESPUSB(size_t task_stack_size, uint8_t event_task_priority)
    : vid(USB_VID),
      pid(USB_PID),
      product_name(USB_PRODUCT),
      manufacturer_name(USB_MANUFACTURER),
      serial_number(USB_SERIAL),
      fw_version(0x0100),
      usb_version(0x0200)  // at least 2.1 or 3.x for BOS & webUSB
      ,
      usb_class(TUSB_CLASS_MISC),
      usb_subclass(MISC_SUBCLASS_COMMON),
      usb_protocol(MISC_PROTOCOL_IAD),
      usb_attributes(TUSB_DESC_CONFIG_ATT_SELF_POWERED),
      usb_power_ma(500),
      webusb_enabled(USB_WEBUSB_ENABLED),
      webusb_url(USB_WEBUSB_URL),
      _started(false),
      _task_stack_size(task_stack_size),
      _event_task_priority(event_task_priority) {
  if (arduino_usb_event_loop_handle == nullptr) {
    const esp_event_loop_args_t event_task_args = {.queue_size = 5,
                                                   .task_name = "arduino_usb_events",
                                                   .task_priority = _event_task_priority,
                                                   .task_stack_size = _task_stack_size,
                                                   .task_core_id = tskNO_AFFINITY};
    if (esp_event_loop_create(&event_task_args, &arduino_usb_event_loop_handle) != ESP_OK) {
      log_e("esp_event_loop_create failed");
    }
  }
}

ESPUSB::~ESPUSB() {
  if (arduino_usb_event_loop_handle != nullptr) {
    esp_event_loop_delete(arduino_usb_event_loop_handle);
    arduino_usb_event_loop_handle = nullptr;
  }
}

bool ESPUSB::begin() {
  if (!_started) {
    tinyusb_device_config_t tinyusb_device_config = {.vid = vid,
                                                     .pid = pid,
                                                     .product_name = product_name.c_str(),
                                                     .manufacturer_name = manufacturer_name.c_str(),
                                                     .serial_number = serial_number.c_str(),
                                                     .fw_version = fw_version,
                                                     .usb_version = usb_version,
                                                     .usb_class = usb_class,
                                                     .usb_subclass = usb_subclass,
                                                     .usb_protocol = usb_protocol,
                                                     .usb_attributes = usb_attributes,
                                                     .usb_power_ma = usb_power_ma,
                                                     .webusb_enabled = webusb_enabled,
                                                     .webusb_url = webusb_url.c_str()};
    _started = tinyusb_init(&tinyusb_device_config) == ESP_OK;
  }
  return _started;
}

void ESPUSB::onEvent(esp_event_handler_t callback) { onEvent(ARDUINO_USB_ANY_EVENT, callback); }
void ESPUSB::onEvent(arduino_usb_event_t event, esp_event_handler_t callback) {
  arduino_usb_event_handler_register_with(ARDUINO_USB_EVENTS, event, callback, this);
}

ESPUSB::operator bool() const { return _started && tinyusb_device_mounted; }

// NOLINTBEGIN(readability-convert-member-functions-to-static,readability-make-member-function-const)
bool ESPUSB::enableDFU() {
#if CFG_TUD_DFU_RUNTIME
  return tinyusb_enable_interface(USB_INTERFACE_DFU, TUD_DFU_RT_DESC_LEN, load_dfu_descriptor) == ESP_OK;
#endif /* CFG_TUD_DFU_RUNTIME */
  return false;
}

bool ESPUSB::VID(uint16_t v) {
  if (!_started) {
    vid = v;
  }
  return !_started;
}
uint16_t ESPUSB::VID() { return vid; }

bool ESPUSB::PID(uint16_t p) {
  if (!_started) {
    pid = p;
  }
  return !_started;
}
uint16_t ESPUSB::PID() { return pid; }

bool ESPUSB::firmwareVersion(uint16_t version) {
  if (!_started) {
    fw_version = version;
  }
  return !_started;
}
uint16_t ESPUSB::firmwareVersion() { return fw_version; }

bool ESPUSB::usbVersion(uint16_t version) {
  if (!_started) {
    usb_version = version;
  }
  return !_started;
}
uint16_t ESPUSB::usbVersion() { return usb_version; }

bool ESPUSB::usbPower(uint16_t mA) {
  if (!_started) {
    usb_power_ma = mA;
  }
  return !_started;
}
uint16_t ESPUSB::usbPower() { return usb_power_ma; }

bool ESPUSB::usbClass(uint8_t _class) {
  if (!_started) {
    usb_class = _class;
  }
  return !_started;
}
uint8_t ESPUSB::usbClass() { return usb_class; }

bool ESPUSB::usbSubClass(uint8_t subClass) {
  if (!_started) {
    usb_subclass = subClass;
  }
  return !_started;
}
uint8_t ESPUSB::usbSubClass() { return usb_subclass; }

bool ESPUSB::usbProtocol(uint8_t protocol) {
  if (!_started) {
    usb_protocol = protocol;
  }
  return !_started;
}
uint8_t ESPUSB::usbProtocol() { return usb_protocol; }

bool ESPUSB::usbAttributes(uint8_t attr) {
  if (!_started) {
    usb_attributes = attr;
  }
  return !_started;
}
uint8_t ESPUSB::usbAttributes() { return usb_attributes; }

bool ESPUSB::webUSB(bool enabled) {
  if (!_started) {
    webusb_enabled = enabled;
    if (enabled && usb_version < 0x0210) {
      usb_version = 0x0210;
    }
  }
  return !_started;
}
bool ESPUSB::webUSB() { return webusb_enabled; }

bool ESPUSB::productName(const char *name) {
  if (!_started) {
    product_name = name;
  }
  return !_started;
}
const char *ESPUSB::productName() { return product_name.c_str(); }

bool ESPUSB::manufacturerName(const char *name) {
  if (!_started) {
    manufacturer_name = name;
  }
  return !_started;
}
const char *ESPUSB::manufacturerName() { return manufacturer_name.c_str(); }

bool ESPUSB::serialNumber(const char *name) {
  if (!_started) {
    serial_number = name;
  }
  return !_started;
}
const char *ESPUSB::serialNumber() { return serial_number.c_str(); }

bool ESPUSB::webUSBURL(const char *name) {
  if (!_started) {
    webusb_url = name;
  }
  return !_started;
}
const char *ESPUSB::webUSBURL() { return webusb_url.c_str(); }

// NOLINTEND(readability-convert-member-functions-to-static,readability-make-member-function-const)

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
ESPUSB USB;

#endif /* CONFIG_TINYUSB_ENABLED */
