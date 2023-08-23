#pragma once

#include "telemetry/telemetryData.hpp"
#include "utils.hpp"

struct RecorderElement {
  packedRXMessage data;
  uint8_t source;  // Link 1 or Link 2
};

class Recorder {
 public:
  explicit Recorder(const char* directory) : directory(directory) {}
  bool begin();

  void enable() { enabled = true; }

  void disable() { enabled = false; }

  void record(packedRXMessage* data, uint8_t link_source) {
    if (enabled) {
      RecorderElement rec_elem = {*data, link_source};
      xQueueSend(queue, &rec_elem, 0);
    }
  }

 private:
  bool initialized = false;
  bool enabled = false;
  bool fileCreated = false;

  const char* directory;

  char fileName[30] = {};

  QueueHandle_t queue{nullptr};
  File file{};

  void createFile();

  static void recordTask(void* pvParameter);
};
