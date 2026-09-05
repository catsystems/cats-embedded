/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "radio_firmware_updater.hpp"

#include <algorithm>
#include <cstring>

#include "console.hpp"

namespace RadioUpdate {
namespace {
constexpr const char* kDirectory = "/telemetry_firmware";

class FileSource final : public Source {
 public:
  explicit FileSource(File& file) : file(file) {}
  uint32_t size() override { return file.size(); }
  bool read(uint32_t offset, uint8_t* data, size_t length) override {
    return file.seek(offset) && file.read(data, length) == static_cast<int>(length);
  }

 private:
  File& file;
};

class UartPort final : public Port {
 public:
  UartPort(Telemetry& link, RadioFirmwareUpdater& updater) : link(link), updater(updater) {}
  bool write(const uint8_t* data, size_t size) override {
    // ESP-IDF bounds the physical TX drain as well as the RX waits below.
    return link.updateSerial().write(data, size) == size;
  }
  bool read(uint8_t* data, size_t size, uint32_t timeoutMs) override {
    const uint32_t started = millis();
    size_t received = 0;
    while (received < size && millis() - started < timeoutMs) {
      const int byte = link.updateSerial().read();
      if (byte >= 0) {
        data[received++] = static_cast<uint8_t>(byte);
      } else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
    return received == size;
  }
  void configure(bool rom) override { link.configureUpdateUart(rom); }
  void drain() override {
    for (size_t count = 0; count < 1024 && link.updateSerial().available(); ++count) {
      (void)link.updateSerial().read();
    }
  }
  void wait(uint32_t ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }
  uint32_t now() override { return millis(); }
  void progress(Phase phase, uint8_t percent) override { updater.progress(phase, percent); }

 private:
  Telemetry& link;
  RadioFirmwareUpdater& updater;
};
}  // namespace

bool RadioFirmwareUpdater::begin() {
  queue = xQueueCreate(1, sizeof(Request));
  if (queue == nullptr) {
    return false;
  }
  if (xTaskCreate(worker, "radio_update", 6144, this, 1, &task) != pdPASS) {
    vQueueDelete(queue);
    queue = nullptr;
    return false;
  }
  return true;
}

Snapshot RadioFirmwareUpdater::snapshot() const {
  portENTER_CRITICAL(&stateMux);
  const Snapshot result = state;
  portEXIT_CRITICAL(&stateMux);
  return result;
}

bool RadioFirmwareUpdater::filename(size_t index, char* destination, size_t capacity) const {
  portENTER_CRITICAL(&stateMux);
  const bool valid = !state.busy && state.phase == Phase::Browsing && index < catalog.size() && capacity > 0;
  if (valid) {
    strncpy(destination, catalog[index].value, capacity - 1);
    destination[capacity - 1] = '\0';
  }
  portEXIT_CRITICAL(&stateMux);
  return valid;
}

bool RadioFirmwareUpdater::submit(const Request& request, Phase phase) {
  portENTER_CRITICAL(&stateMux);
  const bool allowed = queue != nullptr && !state.busy &&
                       (request.action != Action::Program || state.phase == Phase::Confirm) &&
                       (request.action != Action::Select || state.phase == Phase::Browsing);
  if (allowed) {
    if (request.action == Action::Browse) {
      const uint32_t revision = state.revision;
      state = Snapshot{};
      state.revision = revision;
    }
    state.busy = true;
    state.phase = phase;
    state.error[0] = '\0';
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
  if (!allowed) {
    return false;
  }
  // State changes BEFORE enqueueing; the UI cannot mistake the previous result
  // for this request's completion. One outstanding request owns all resources.
  if (xQueueSend(queue, &request, 0) != pdPASS) {
    finish(Phase::Failed, "Update queue unavailable");
    return false;
  }
  return true;
}

bool RadioFirmwareUpdater::browse() { return submit(Request{}, Phase::Preparing); }
bool RadioFirmwareUpdater::program() { return submit(Request{Action::Program, {}}, Phase::Preparing); }
bool RadioFirmwareUpdater::close() { return submit(Request{Action::Close, {}}, Phase::Closing); }

bool RadioFirmwareUpdater::select(size_t index) {
  Request request{Action::Select, {}};
  return filename(index, request.filename, sizeof(request.filename)) && submit(request, Phase::Validating);
}

void RadioFirmwareUpdater::progress(Phase phase, uint8_t percent) {
  portENTER_CRITICAL(&stateMux);
  if (state.phase != phase || state.percent != percent) {
    state.phase = phase;
    state.percent = percent;
    ++state.revision;
  }
  portEXIT_CRITICAL(&stateMux);
}

void RadioFirmwareUpdater::finish(Phase phase, const char* error) {
  portENTER_CRITICAL(&stateMux);
  state.phase = phase;
  state.busy = false;
  strncpy(state.error, error, sizeof(state.error) - 1);
  state.stackFreeBytes = uxTaskGetStackHighWaterMark(nullptr);
  ++state.revision;
  portEXIT_CRITICAL(&stateMux);
}

void RadioFirmwareUpdater::release() {
  file.close();
  if (storageOwned) {
    Utils::endRadioUpdateStorage();
    storageOwned = false;
  }
  if (recorderPaused) {
    recorder.resumeAfterRadioUpdate();
    recorderPaused = false;
  }
}

void RadioFirmwareUpdater::scan() {
  release();
  catalog.clear();
  if (!Utils::isFilesystemAvailable()) {
    finish(Phase::Failed, "Safely eject USB drive, then retry");
    return;
  }
  if (!links[0]->safeForUpdate() || !links[1]->safeForUpdate() || !recorder.pauseForRadioUpdate()) {
    finish(Phase::Failed, "Need idle recorder and safe, non-testing links");
    return;
  }
  recorderPaused = true;
  storageOwned = Utils::beginRadioUpdateStorage();
  if (!storageOwned) {
    release();
    finish(Phase::Failed, "Safely eject USB drive, then retry");
    return;
  }
  auto directory = fatfs.open(kDirectory);
  const char* error = "";
  if (!directory || !directory.isDirectory()) {
    error = "Copy .bin files into /telemetry_firmware";
  } else {
    auto entry = directory.openNextFile();
    while (entry) {
      // FatFs long names can exceed our bound. Never silently select a truncated name.
      char name[256]{};
      const size_t length = entry.getName(name, sizeof(name));
      if (!entry.isDirectory() && length > 4) {
        const char* suffix = name + length - 4;
        const bool binSuffix = suffix[0] == '.' && (suffix[1] == 'b' || suffix[1] == 'B') &&
                               (suffix[2] == 'i' || suffix[2] == 'I') && (suffix[3] == 'n' || suffix[3] == 'N');
        if (binSuffix && length >= kNameSize) {
          error = "Firmware filename too long (max 127)";
        } else if (isBinName(name)) {
          if (catalog.size() == kMaxFiles) {
            error = "More than 64 .bin files; remove extras";
          } else {
            Filename item{};
            memcpy(item.value, name, length + 1);
            catalog.push_back(item);
          }
        }
      }
      entry.close();
      if (error[0] != '\0') {
        break;
      }
      entry = directory.openNextFile();
    }
  }
  directory.close();
  if (error[0] != '\0' || catalog.empty()) {
    release();
    finish(Phase::Failed, error[0] != '\0' ? error : "No .bin files in /telemetry_firmware");
    return;
  }
  std::sort(catalog.begin(), catalog.end(),
            [](const Filename& a, const Filename& b) { return strcmp(a.value, b.value) < 0; });
  portENTER_CRITICAL(&stateMux);
  state.fileCount = catalog.size();
  portEXIT_CRITICAL(&stateMux);
  finish(Phase::Browsing);
}

void RadioFirmwareUpdater::open(const char* name) {
  file.close();
  char path[kNameSize + 24]{};
  snprintf(path, sizeof(path), "%s/%s", kDirectory, name);
  file = fatfs.open(path, FILE_READ);
  FileSource source(file);
  ImageInfo info{};
  if (!storageOwned || !file || file.isDirectory() || !isBinName(name) || !inspect(source, info)) {
    release();
    finish(Phase::Failed, "Invalid .bin size, vectors, or file read");
    return;
  }
  portENTER_CRITICAL(&stateMux);
  strncpy(state.filename, name, sizeof(state.filename) - 1);
  state.image = info;
  portEXIT_CRITICAL(&stateMux);
  finish(Phase::Confirm);
}

void RadioFirmwareUpdater::updateBoth() {
  const ImageInfo expected = snapshot().image;
  bool held[2]{};
  LinkResult results[2]{};
  const char* error = "";
  if (!storageOwned || !recorderPaused || !file || !links[0]->safeForUpdate() || !links[1]->safeForUpdate()) {
    error = "Safety state changed; update not started";
  } else {
    held[0] = links[0]->beginUpdate();
    held[1] = held[0] && links[1]->beginUpdate();
    if (!held[1] || !links[0]->safeForUpdate() || !links[1]->safeForUpdate()) {
      error = "UART handoff/safety check failed";
    } else {
      FileSource source(file);
      for (size_t index = 0; index < 2; ++index) {
        portENTER_CRITICAL(&stateMux);
        state.link = static_cast<uint8_t>(index + 1);
        state.percent = 0;
        ++state.revision;
        portEXIT_CRITICAL(&stateMux);
        UartPort port(*links[index], *this);
        RomBootloader bootloader(port);
        const bool success = bootloader.run(source, expected, results[index]);
        portENTER_CRITICAL(&stateMux);
        state.results[index] = results[index];
        portEXIT_CRITICAL(&stateMux);
        if (!success) {
          error = bootloader.error();
          break;
        }
      }
    }
  }
  for (size_t index = 0; index < 2; ++index) {
    if (held[index]) {
      // A missing entry ACK is ambiguous: that application may already be in
      // ROM. Quarantine it, while restoring untouched and verified links.
      links[index]->finishUpdate(!results[index].entryRequested || results[index].success,
                                 results[index].success ? results[index].version : nullptr);
    }
  }
  release();
  finish(error[0] == '\0' ? Phase::Complete : Phase::Failed, error);
  const auto status = snapshot();
  console.log.printf("[RADIO UPDATE] %s; link1=%s link2=%s; ROM=%02x/%02x; stack free=%lu bytes\n",
                     phaseName(status.phase), results[0].version, results[1].version, results[0].romRevision,
                     results[1].romRevision, static_cast<unsigned long>(status.stackFreeBytes));
}

void RadioFirmwareUpdater::worker(void* parameter) {
  auto* self = static_cast<RadioFirmwareUpdater*>(parameter);
  Request request{};
  for (;;) {
    if (xQueueReceive(self->queue, &request, portMAX_DELAY) != pdPASS) {
      continue;
    }
    switch (request.action) {
      case Action::Browse:
        self->scan();
        break;
      case Action::Select:
        self->open(request.filename);
        break;
      case Action::Program:
        self->updateBoth();
        break;
      case Action::Close:
        self->release();
        self->finish(Phase::Idle);
        break;
    }
  }
}

}  // namespace RadioUpdate
