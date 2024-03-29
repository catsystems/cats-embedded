/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Florian Baumgartner's ESP32 IoT Framework
/// (https://github.com/FlorianBaumgartner/ESP32_IoT_Framework), released under MIT License.

#ifndef CONSOLE_H
#define CONSOLE_H

#include <Arduino.h>
#include <USB.h>

// [hz]
inline constexpr uint8_t INTERFACE_UPDATE_RATE = 10;

// [#]  Buffer Size must be power of 2
inline constexpr uint32_t QUEUE_BUFFER_LENGTH = 1U << 13U;

// [ms] Data transmission hold-back delay after console object has been enabled
inline constexpr uint32_t CONSOLE_ACTIVE_DELAY = 3000;

// [ms] Data transmission hold-back delay after physical connection has been established (Terminal opened)
inline constexpr uint32_t INTERFACE_ACTIVE_DELAY = 1500;

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define CONSOLE_CLEAR "\033[2J\033[1;1H"

#define CONSOLE_COLOR_BLACK   "\033[0;30"
#define CONSOLE_COLOR_RED     "\033[0;31"
#define CONSOLE_COLOR_GREEN   "\033[0;32"
#define CONSOLE_COLOR_YELLOW  "\033[0;33"
#define CONSOLE_COLOR_BLUE    "\033[0;34"
#define CONSOLE_COLOR_MAGENTA "\033[0;35"
#define CONSOLE_COLOR_CYAN    "\033[0;36"
#define CONSOLE_COLOR_WHITE   "\033[0;37"
#define CONSOLE_COLOR_DEFAULT "\033[0;39"

#define CONSOLE_COLOR_BOLD_BLACK   "\033[1;30"
#define CONSOLE_COLOR_BOLD_RED     "\033[1;31"
#define CONSOLE_COLOR_BOLD_GREEN   "\033[1;32"
#define CONSOLE_COLOR_BOLD_YELLOW  "\033[1;33"
#define CONSOLE_COLOR_BOLD_BLUE    "\033[1;34"
#define CONSOLE_COLOR_BOLD_MAGENTA "\033[1;35"
#define CONSOLE_COLOR_BOLD_CYAN    "\033[1;36"
#define CONSOLE_COLOR_BOLD_WHITE   "\033[1;37"
#define CONSOLE_COLOR_BOLD_DEFAULT "\033[1;39"

#define CONSOLE_BACKGROUND_BLACK   ";40m"
#define CONSOLE_BACKGROUND_RED     ";41m"
#define CONSOLE_BACKGROUND_GREEN   ";42m"
#define CONSOLE_BACKGROUND_YELLOW  ";43m"
#define CONSOLE_BACKGROUND_BLUE    ";44m"
#define CONSOLE_BACKGROUND_MAGENTA ";45m"
#define CONSOLE_BACKGROUND_CYAN    ";46m"
#define CONSOLE_BACKGROUND_WHITE   ";47m"
#define CONSOLE_BACKGROUND_DEFAULT ";49m"
// NOLINTEND(cppcoreguidelines-macro-usage)

#define CONSOLE_OK      (CONSOLE_COLOR_GREEN CONSOLE_BACKGROUND_DEFAULT)
#define CONSOLE_LOG     (CONSOLE_COLOR_DEFAULT CONSOLE_BACKGROUND_DEFAULT)
#define CONSOLE_ERROR   (CONSOLE_COLOR_RED CONSOLE_BACKGROUND_DEFAULT)
#define CONSOLE_WARNING (CONSOLE_COLOR_YELLOW CONSOLE_BACKGROUND_DEFAULT)

#define DISABLE_MODULE_LEVEL dummy

enum ConsoleColor {
  COLOR_DEFAULT,
  COLOR_BLACK,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_YELLOW,
  COLOR_BLUE,
  COLOR_MAGENTA,
  COLOR_CYAN,
  COLOR_WHITE
};

class ConsoleStatus : public Stream {
 public:
  Stream* console = nullptr;
  enum ConsoleType { StatusOk_t, StatusLog_t, StatusWarning_t, StatusError_t, StatusCustom_t, StatusDummy_t };
  ConsoleType type;
  bool enabled = true;
  bool colorEnabled = true;
  const char* textColor = CONSOLE_COLOR_DEFAULT;
  const char* backgroundColor = CONSOLE_BACKGROUND_DEFAULT;

  explicit ConsoleStatus(ConsoleType t) : type(t) {}
  inline void ref(Stream* c) { console = c; }
  inline void enable(bool s) { enabled = s; }
  inline int available() override { return console->available(); }
  inline int read() override { return console->read(); }
  inline int peek() override { return console->peek(); }
  inline size_t write(uint8_t c) override { return write(static_cast<const uint8_t*>(&c), 1); }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  inline size_t write(const char* buffer, size_t size) { return write(reinterpret_cast<const uint8_t*>(buffer), size); }
  ConsoleStatus& operator[](ConsoleColor color) {
    switch (color) {
      case COLOR_DEFAULT:
        backgroundColor = CONSOLE_BACKGROUND_DEFAULT;
        break;
      case COLOR_BLACK:
        backgroundColor = CONSOLE_BACKGROUND_BLACK;
        break;
      case COLOR_RED:
        backgroundColor = CONSOLE_BACKGROUND_RED;
        break;
      case COLOR_GREEN:
        backgroundColor = CONSOLE_BACKGROUND_GREEN;
        break;
      case COLOR_YELLOW:
        backgroundColor = CONSOLE_BACKGROUND_YELLOW;
        break;
      case COLOR_BLUE:
        backgroundColor = CONSOLE_BACKGROUND_BLUE;
        break;
      case COLOR_MAGENTA:
        backgroundColor = CONSOLE_BACKGROUND_MAGENTA;
        break;
      case COLOR_CYAN:
        backgroundColor = CONSOLE_BACKGROUND_CYAN;
        break;
      case COLOR_WHITE:
        backgroundColor = CONSOLE_BACKGROUND_WHITE;
        break;
    }
    return *this;
  }
  size_t write(const uint8_t* buffer, size_t size) override {
    if (!enabled || type == StatusDummy_t) {
      return 0;
    }
    if (colorEnabled) {
      switch (type) {
        case StatusCustom_t:
          console->print(textColor);
          console->print(backgroundColor);
          break;
        case StatusOk_t:
          console->print(CONSOLE_OK);
          break;
        case StatusWarning_t:
          console->print(CONSOLE_WARNING);
          break;
        case StatusError_t:
          console->print(CONSOLE_ERROR);
          break;
        default:
          console->print(CONSOLE_LOG);
          break;
      }
    }
    size = console->write(buffer, size);
    if (colorEnabled) {
      console->print(CONSOLE_LOG);
    }
    backgroundColor = CONSOLE_BACKGROUND_DEFAULT;
    return size;
  }
};

class Console : public Stream {
 private:
  enum SerialType { Virtual_t, USBCDC_t, HardwareSerial_t };
  Stream& stream;
  SerialType type = Virtual_t;
  volatile bool initialized = false;
  volatile bool enabled = false;       // Indicates if the stream is enabled (e.g. is set after USB MSC setup is done)
  volatile bool streamActive = false;  // Indicates if the console is opened and data is tranmitted
  volatile char ringBuffer[QUEUE_BUFFER_LENGTH]{};
  volatile uint32_t writeIdx = 0, readIdx = 0;
  SemaphoreHandle_t bufferAccessSemaphore = nullptr;
  TaskHandle_t writeTaskHandle = nullptr;
  ConsoleStatus custom = ConsoleStatus(ConsoleStatus::StatusCustom_t);

  bool initialize();
  void printStartupMessage();
  static void writeTask(void* pvParameter);
  static void interfaceTask(void* pvParameter);
  static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
  bool getInterfaceState() {
    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast) dynamic cast is not allowed with -fno-rtti
    if (type == USBCDC_t) {
      return *static_cast<USBCDC*>(&stream);
    }
    if (type == HardwareSerial_t) {
      return *static_cast<HardwareSerial*>(&stream);
    }
    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast) dynamic cast is not allowed with -fno-rtti

    return true;
  }

 public:
  enum ConsoleLevel { LEVEL_LOG = 0, LEVEL_OK = 1, LEVEL_WARNING = 2, LEVEL_ERROR = 3, LEVEL_OFF = 4 };
  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
  ConsoleStatus ok = ConsoleStatus(ConsoleStatus::StatusOk_t);
  ConsoleStatus log = ConsoleStatus(ConsoleStatus::StatusLog_t);
  ConsoleStatus error = ConsoleStatus(ConsoleStatus::StatusError_t);
  ConsoleStatus warning = ConsoleStatus(ConsoleStatus::StatusWarning_t);
  ConsoleStatus dummy = ConsoleStatus(ConsoleStatus::StatusDummy_t);
  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

  explicit Console(USBCDC& stream) : stream(stream), type(USBCDC_t) {
    ok.ref(this);
    log.ref(this);
    error.ref(this);
    warning.ref(this);
    custom.ref(this);
    dummy.ref(this);
  }
  explicit Console(HardwareSerial& stream) : stream(stream), type(HardwareSerial_t) {
    ok.ref(this);
    log.ref(this);
    error.ref(this);
    warning.ref(this);
    custom.ref(this);
    dummy.ref(this);
  }
  bool begin();  // Used for USBSerial
  bool begin(uint32_t baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false,
             uint32_t timeout_ms = 20000UL,
             uint8_t rxfifo_full_thrhd = 112);  // Used for HardwareSerial
  void end();
  void enable(bool state) { enabled = state; }
  void flush() override { readIdx = writeIdx; }
  void printTimestamp();  // TODO: Add possibillity to add string as parameter
  void enableColors(bool state) {
    ok.colorEnabled = log.colorEnabled = error.colorEnabled = warning.colorEnabled = custom.colorEnabled =
        dummy.colorEnabled = state;
  }
  void setLevel(ConsoleLevel level) {
    log.enable(level <= LEVEL_LOG);
    ok.enable(level <= LEVEL_OK);
    warning.enable(level <= LEVEL_WARNING);
    error.enable(level <= LEVEL_ERROR);
  }
  ConsoleStatus& operator[](ConsoleColor color) {
    switch (color) {
      case COLOR_DEFAULT:
        custom.textColor = CONSOLE_COLOR_DEFAULT;
        break;
      case COLOR_BLACK:
        custom.textColor = CONSOLE_COLOR_BLACK;
        break;
      case COLOR_RED:
        custom.textColor = CONSOLE_COLOR_RED;
        break;
      case COLOR_GREEN:
        custom.textColor = CONSOLE_COLOR_GREEN;
        break;
      case COLOR_YELLOW:
        custom.textColor = CONSOLE_COLOR_YELLOW;
        break;
      case COLOR_BLUE:
        custom.textColor = CONSOLE_COLOR_BLUE;
        break;
      case COLOR_MAGENTA:
        custom.textColor = CONSOLE_COLOR_MAGENTA;
        break;
      case COLOR_CYAN:
        custom.textColor = CONSOLE_COLOR_CYAN;
        break;
      case COLOR_WHITE:
        custom.textColor = CONSOLE_COLOR_WHITE;
        break;
    }
    return custom;
  }

  explicit operator bool() const { return streamActive; }
  inline int available() override { return stream.available(); }
  inline int read() override { return stream.read(); }
  inline int peek() override { return stream.peek(); }
  inline size_t write(uint8_t c) override { return write(&c, 1); }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  inline size_t write(const char* buffer, size_t size) { return write(reinterpret_cast<const uint8_t*>(buffer), size); }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  inline size_t write(const char* s) { return write(reinterpret_cast<const uint8_t*>(s), strlen(s)); }
  inline size_t write(uint32_t n) { return write(static_cast<uint8_t>(n)); }
  inline size_t write(int32_t n) { return write(static_cast<uint8_t>(n)); }
  size_t write(const uint8_t* buffer, size_t size) override;
};

#ifndef USE_CUSTOM_CONSOLE
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern Console console;
#endif

#endif
