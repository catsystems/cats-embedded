/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "recorder.hpp"

#include "config.hpp"

bool Recorder::begin() {
  if (!fatfs.chdir(m_directory)) {
    console.error.print("[REC] Open directory failed");
    console.error.println(m_directory);
    fatfs.mkdir(&m_directory[1]);
    console.log.println("[REC] Crating directory");
    if (!fatfs.chdir(m_directory)) {
      console.error.println("[REC] Open directory failed");
      return false;
    }
  }

  setNewFileName();

  m_queue = xQueueCreate(64, sizeof(RecorderElement));
  xTaskCreate(recordTask, "task_recorder", 4096, this, 1, nullptr);
  m_initialized = true;
  return m_initialized;
}

void Recorder::createFile() {
  m_file = fatfs.open(m_fileName, FILE_WRITE);
  console.log.println(m_fileName);
  console.error.printf("[REC] File %s created\n", m_fileName);
  if (!m_file) {
    console.error.println("[REC] Open file failed");
    return;
  }
  m_fileCreated = true;
  m_file.println(
      "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/"
      "s],battery[decivolts],pyro1,pyro2");
}

void Recorder::setNewFileName() {
  int32_t number = 0;
  do {
    if (systemConfig.config.receiverMode == ReceiverTelemetryMode::kSingle) {
      snprintf(m_fileName, 60, "%s_%03ld.csv", systemConfig.config.linkPhrase1, number);
    } else {
      snprintf(m_fileName, 60, "%s_%s_%03ld.csv", systemConfig.config.linkPhrase1, systemConfig.config.linkPhrase2,
               number);
    }
    ++number;
  } while (fatfs.exists(m_fileName));

  console.log.printf("[REC] New file name set to %s\n", m_fileName);
}

void Recorder::closeCurrentFile() {
  // Close previously opened file
  console.log.printf("[REC] Closing %s\n", m_fileName);
  m_file.close();
  console.log.printf("[REC] %s closed\n", m_fileName);
  setNewFileName();
}

void Recorder::recordTask(void *pvParameter) {
  auto *rec = static_cast<Recorder *>(pvParameter);
  char line[128];
  uint32_t count = 0;
  RecorderElement element{};
  while (rec->m_initialized) {
    if (xQueueReceive(rec->m_queue, &element, portMAX_DELAY) == pdPASS) {
      if (!rec->m_fileCreated) {
        rec->createFile();
      }
      const auto &data = element.data;
      const auto pyro1_continuity = static_cast<bool>(data.pyro_continuity & 0x01U);
      const auto pyro2_continuity = static_cast<bool>(data.pyro_continuity & 0x02U);
      snprintf(line, 128, "%hu,%d,%d,%d,%d,%d,%d,%d,%d,%hu,%hu", element.source, data.timestamp, data.state,
               data.errors, data.lat, data.lon, data.altitude, data.velocity, data.voltage,
               static_cast<uint8_t>(pyro1_continuity), static_cast<uint8_t>(pyro2_continuity));
      rec->m_file.println(line);
      count++;

      if (count == 10) {
        count = 0;
        rec->m_file.sync();
      }
    }
  }
  vTaskDelete(nullptr);
}
