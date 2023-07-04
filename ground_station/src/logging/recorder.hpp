#pragma once

#include "telemetry/telemetryData.hpp"
#include "utils.hpp"

class Recorder {
 public:
  Recorder(const char* directory) : directory(directory) {}
  bool begin();

  void enable() { enabled = true; }

  void disable() { enabled = false; }

  void record(packedRXMessage* data) {
    if (enabled) {
      xQueueSend(queue, data, 0);
    }
  }

 private:
  bool initialized = false;
  bool enabled = false;
  bool fileCreated = false;

  const char* directory;

  char fileName[30] = {};

  QueueHandle_t queue;
  File file;

  void createFile();

  static void recordTask(void* pvParameter);
};
