/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <FatLib/FatFile.h>

#include "telemetry/telemetryData.hpp"
#include "utils.hpp"

struct RecorderElement {
  packedRXMessage data;
  uint8_t source;  // Link 1 or Link 2
};

class Recorder {
 public:
  explicit Recorder(const char* directory) : m_directory(directory) {}
  bool begin();

  void enable() { m_enabled = true; }

  void closeCurrentFile();

  void record(packedRXMessage* data, uint8_t link_source) {
    if (m_enabled) {
      RecorderElement rec_elem = {*data, link_source};
      xQueueSend(m_queue, &rec_elem, 0);
    }
  }

 private:
  bool m_initialized = false;
  bool m_enabled = false;
  bool m_fileCreated = false;

  const char* m_directory{nullptr};

  char m_fileName[60] = {};

  QueueHandle_t m_queue{nullptr};
  File32 m_file{};

  void createFile();
  void setNewFileName();

  static void recordTask(void* pvParameter);
};
