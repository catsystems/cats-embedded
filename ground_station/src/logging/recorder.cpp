/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "recorder.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

#include "console.hpp"

namespace {
constexpr uint8_t kReadyState = 2;
constexpr uint8_t kTouchdownState = 7;
constexpr size_t kQueueDepth = 128;
constexpr char kHeader[] =
    "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/"
    "s],battery[decivolts],pyro1,pyro2";
}  // namespace

bool Recorder::begin() {
  if (!fatfs.chdir(directory)) {
    fatfs.mkdir(&directory[1]);
    if (!fatfs.chdir(directory)) {
      setFault("Log directory unavailable");
      return false;
    }
  }

  queue = xQueueCreate(kQueueDepth, sizeof(Command));
  responseQueue = xQueueCreate(4, sizeof(Response));
  fsMutex = xSemaphoreCreateMutex();
  catalogMutex = xSemaphoreCreateMutex();
  if (queue == nullptr || responseQueue == nullptr || fsMutex == nullptr || catalogMutex == nullptr) {
    setFault("Recorder queue unavailable");
    return false;
  }
  initialized = true;
  xTaskCreate(recordTask, "task_recorder", 6144, this, 1, nullptr);
  return true;
}

void Recorder::onTelemetryPacket(const packedRXMessage& packet, uint8_t source) {
  if (!initialized || !enabled || source < 1 || source > 2) {
    return;
  }
  Command command{};
  command.type = CommandType::Sample;
  command.data = packet;
  command.source = source;
  if (xQueueSend(queue, &command, 0) != pdPASS) {
    portENTER_CRITICAL(&statusMux);
    status.droppedSamples++;
    strncpy(status.lastError, "Recorder queue overflow", sizeof(status.lastError) - 1U);
    status.state = RecorderState::Fault;
    portEXIT_CRITICAL(&statusMux);
  }
}

RecorderStatus Recorder::getStatus() const {
  portENTER_CRITICAL(&statusMux);
  const RecorderStatus copy = status;
  portEXIT_CRITICAL(&statusMux);
  return copy;
}

void Recorder::setFault(const char* message) {
  portENTER_CRITICAL(&statusMux);
  status.state = RecorderState::Fault;
  strncpy(status.lastError, message, sizeof(status.lastError) - 1U);
  status.lastError[sizeof(status.lastError) - 1U] = '\0';
  portEXIT_CRITICAL(&statusMux);
  console.warning.print("[REC] ");
  console.warning.println(message);
}

bool Recorder::chooseNextFileName() {
  int32_t highest = -1;
  auto directoryFile = fatfs.open(directory);
  if (!directoryFile) {
    return false;
  }
  auto entry = directoryFile.openNextFile();
  while (entry) {
    char name[kLogFilenameSize]{};
    entry.getName(name, sizeof(name));
    highest = std::max(highest, logNumber(name));
    entry.close();
    entry = directoryFile.openNextFile();
  }
  directoryFile.close();
  snprintf(fileName, sizeof(fileName), "log_%03ld.csv", static_cast<long>(highest + 1));
  return true;
}

bool Recorder::createFile() {
  if (xSemaphoreTake(fsMutex, portMAX_DELAY) != pdTRUE) {
    setFault("Storage lock failed");
    return false;
  }
  if (!chooseNextFileName()) {
    xSemaphoreGive(fsMutex);
    setFault("Could not scan log directory");
    return false;
  }
  char path[kLogFilenameSize + 16U]{};
  snprintf(path, sizeof(path), "%s/%s", directory, fileName);
  file = fatfs.open(path, FILE_WRITE);
  if (!file) {
    xSemaphoreGive(fsMutex);
    setFault("Could not open log file");
    return false;
  }
  if (file.println(kHeader) == 0) {
    file.close();
    xSemaphoreGive(fsMutex);
    setFault("Could not write log header");
    return false;
  }
  xSemaphoreGive(fsMutex);
  fileCreated = true;
  unsyncedRows = 0;
  portENTER_CRITICAL(&statusMux);
  status = RecorderStatus{};
  status.state = RecorderState::Recording;
  strncpy(status.activeFilename, fileName, sizeof(status.activeFilename) - 1U);
  portEXIT_CRITICAL(&statusMux);
  return true;
}

bool Recorder::writeSample(const Command& command) {
  if (!fileCreated && !createFile()) {
    return false;
  }
  const auto& data = command.data;
  const auto sourceBit = static_cast<uint8_t>(1U << (command.source - 1U));
  portENTER_CRITICAL(&statusMux);
  status.participantMask |= sourceBit;
  portEXIT_CRITICAL(&statusMux);
  char line[128]{};
  const auto pyro1 = static_cast<uint8_t>((data.pyro_continuity & 0x01U) != 0U);
  const auto pyro2 = static_cast<uint8_t>((data.pyro_continuity & 0x02U) != 0U);
  snprintf(line, sizeof(line), "%hu,%d,%d,%d,%d,%d,%d,%d,%d,%hu,%hu", command.source, data.timestamp, data.state,
           data.errors, data.lat, data.lon, data.altitude, data.velocity, data.voltage, pyro1, pyro2);
  if (xSemaphoreTake(fsMutex, portMAX_DELAY) != pdTRUE) {
    setFault("Storage lock failed");
    return false;
  }
  const auto written = file.println(line) > 0;
  xSemaphoreGive(fsMutex);
  if (!written) {
    setFault("Could not write log sample");
    return false;
  }
  portENTER_CRITICAL(&statusMux);
  status.writtenSamples++;
  portEXIT_CRITICAL(&statusMux);
  if (++unsyncedRows >= 10U) {
    unsyncedRows = 0;
    if (xSemaphoreTake(fsMutex, portMAX_DELAY) == pdTRUE) {
      const auto synced = file.sync();
      xSemaphoreGive(fsMutex);
      if (!synced) {
        setFault("Could not sync log file");
        return false;
      }
    }
  }
  return true;
}

bool Recorder::finalizeFile(FinalizeReason reason [[maybe_unused]]) {
  if (!fileCreated) {
    return false;
  }
  portENTER_CRITICAL(&statusMux);
  status.state = RecorderState::Finalizing;
  portEXIT_CRITICAL(&statusMux);
  if (xSemaphoreTake(fsMutex, portMAX_DELAY) != pdTRUE) {
    setFault("Storage lock failed");
    return false;
  }
  const auto synced = file.sync();
  file.close();
  xSemaphoreGive(fsMutex);
  if (!synced) {
    setFault("Could not finalize log");
    return false;
  }
  completedParticipantMask = status.participantMask;
  fileCreated = false;
  armed = false;
  rearmMask = 0;
  touchdownMask = 0;
  portENTER_CRITICAL(&statusMux);
  status.state = RecorderState::Idle;
  status.activeFilename[0] = '\0';
  portEXIT_CRITICAL(&statusMux);
  return true;
}

bool Recorder::deleteFile(const char* name) {
  if (!Utils::isFilesystemAvailable() || name == nullptr || name[0] == '\0' ||
      (fileCreated && strcmp(name, fileName) == 0)) {
    return false;
  }
  char path[kLogFilenameSize + 16U]{};
  snprintf(path, sizeof(path), "%s/%s", directory, name);
  if (xSemaphoreTake(fsMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  const bool result = fatfs.remove(path);
  xSemaphoreGive(fsMutex);
  return result;
}

bool Recorder::submitAndWait(Command& command) {
  if (!initialized) {
    return false;
  }
  command.requestId = ++nextRequestId;
  if (xQueueSend(queue, &command, pdMS_TO_TICKS(250)) != pdPASS) {
    return false;
  }
  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
  Response response{};
  while (xTaskGetTickCount() < deadline) {
    const TickType_t remaining = deadline - xTaskGetTickCount();
    if (xQueueReceive(responseQueue, &response, remaining) != pdPASS) {
      return false;
    }
    if (response.requestId == command.requestId) {
      return response.result;
    }
  }
  return false;
}

bool Recorder::finalize(FinalizeReason reason) {
  Command command{};
  command.type = CommandType::Finalize;
  command.reason = reason;
  return submitAndWait(command);
}

bool Recorder::shareWithMassStorage() {
  Command command{};
  command.type = CommandType::ShareMassStorage;
  return submitAndWait(command);
}

bool Recorder::sync() {
  Command command{};
  command.type = CommandType::Sync;
  return submitAndWait(command);
}

bool Recorder::deleteLog(const char* name) {
  Command command{};
  command.type = CommandType::Delete;
  strncpy(command.name, name, sizeof(command.name) - 1U);
  return submitAndWait(command);
}

// Keep command handling together so storage ownership transitions remain explicit.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Recorder::recordTask(void* pvParameter) {
  auto* ref = static_cast<Recorder*>(pvParameter);
  Command command{};
  while (ref->initialized) {
    if (xQueueReceive(ref->queue, &command, portMAX_DELAY) != pdPASS) {
      continue;
    }
    bool result = true;
    if (command.type == CommandType::Sample) {
      if (!ref->enabled) {
        continue;
      }
      const auto bit = static_cast<uint8_t>(1U << (command.source - 1U));
      if (!ref->armed) {
        if (command.data.state <= kReadyState) {
          ref->rearmMask |= bit;
          const bool canRearm = ref->receiverMode == SINGLE ||
                                (ref->completedParticipantMask != 0U &&
                                 (ref->rearmMask & ref->completedParticipantMask) == ref->completedParticipantMask);
          if (canRearm) {
            ref->armed = true;
            ref->completedParticipantMask = 0;
            portENTER_CRITICAL(&ref->statusMux);
            ref->status.participantMask = 0;
            ref->status.writtenSamples = 0;
            portEXIT_CRITICAL(&ref->statusMux);
          }
        }
      } else if (command.data.state > kReadyState) {
        if (!Utils::isFilesystemAvailable() && !Utils::claimFirmwareStorage()) {
          ref->setFault("Storage unavailable for logging");
          result = false;
        } else {
          result = ref->writeSample(command);
        }
        if (result && command.data.state == kTouchdownState) {
          ref->touchdownMask |= bit;
          const bool complete = ref->receiverMode == SINGLE ||
                                (ref->status.participantMask != 0U &&
                                 (ref->touchdownMask & ref->status.participantMask) == ref->status.participantMask);
          if (complete && !ref->neverStopLogging) {
            result = ref->finalizeFile(FinalizeReason::MissionComplete);
          }
        }
      }
    } else if (command.type == CommandType::Finalize) {
      result = ref->finalizeFile(command.reason);
    } else if (command.type == CommandType::Sync) {
      if (!ref->fileCreated) {
        result = true;
      } else if (xSemaphoreTake(ref->fsMutex, portMAX_DELAY) == pdTRUE) {
        result = ref->file.sync();
        xSemaphoreGive(ref->fsMutex);
        if (!result) {
          ref->setFault("Could not sync log file");
        }
      } else {
        result = false;
      }
    } else if (command.type == CommandType::CatalogRefresh) {
      auto refreshed = std::vector<LogEntry>{};
      result = ref->scanCatalog(refreshed);
      if (result && xSemaphoreTake(ref->catalogMutex, portMAX_DELAY) == pdTRUE) {
        ref->catalogCache = std::move(refreshed);
        xSemaphoreGive(ref->catalogMutex);
      }
    } else if (command.type == CommandType::Delete) {
      result = ref->deleteFile(command.name);
    } else if (command.type == CommandType::ShareMassStorage) {
      result = !ref->fileCreated && ref->getStatus().state == RecorderState::Idle;
      if (result) {
        result = Utils::requestMassStorage();
      }
    }
    if (command.requestId != 0U) {
      const Response response{command.requestId, result};
      (void)xQueueSend(ref->responseQueue, &response, 0);
    }
  }
  vTaskDelete(nullptr);
}

int32_t Recorder::logNumber(const char* name) {
  // sscanf's %ld conversion requires the C library's long type.
  // NOLINTNEXTLINE(google-runtime-int)
  long number = -1;
  char tail = '\0';
  return sscanf(name, "log_%ld.csv%c", &number, &tail) == 1 ? static_cast<int32_t>(number) : -1;
}

bool Recorder::isCsvLog(const char* name) {
  if (name == nullptr) {
    return false;
  }
  const size_t length = strlen(name);
  if (length <= 4U || name[length - 4U] != '.') {
    return false;
  }
  const auto lower = [](char value) {
    return static_cast<char>(value >= 'A' && value <= 'Z' ? value + ('a' - 'A') : value);
  };
  return lower(name[length - 3U]) == 'c' && lower(name[length - 2U]) == 's' && lower(name[length - 1U]) == 'v';
}

bool Recorder::scanCatalog(std::vector<LogEntry>& entries) {
  entries.clear();
  if (xSemaphoreTake(fsMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  auto directoryFile = fatfs.open(directory);
  if (!directoryFile) {
    xSemaphoreGive(fsMutex);
    return false;
  }
  auto entry = directoryFile.openNextFile();
  while (entry) {
    LogEntry log{};
    entry.getName(log.name, sizeof(log.name));
    if (!entry.isDirectory() && isCsvLog(log.name)) {
      log.sizeBytes = entry.size();
      log.active = fileCreated && strcmp(log.name, fileName) == 0;
      entries.push_back(log);
    }
    entry.close();
    entry = directoryFile.openNextFile();
  }
  directoryFile.close();
  xSemaphoreGive(fsMutex);
  std::stable_sort(entries.begin(), entries.end(), [](const LogEntry& lhs, const LogEntry& rhs) {
    const int32_t lhsNumber = logNumber(lhs.name);
    const int32_t rhsNumber = logNumber(rhs.name);
    if ((lhsNumber >= 0) != (rhsNumber >= 0)) {
      return lhsNumber >= 0;
    }
    if (lhsNumber >= 0 && lhsNumber != rhsNumber) {
      return lhsNumber > rhsNumber;
    }
    return false;
  });
  return true;
}

bool Recorder::refreshCatalog(std::vector<LogEntry>& entries) {
  Command command{};
  command.type = CommandType::CatalogRefresh;
  if (!submitAndWait(command) || xSemaphoreTake(catalogMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  entries = catalogCache;
  xSemaphoreGive(catalogMutex);
  return true;
}
