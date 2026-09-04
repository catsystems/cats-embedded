/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <vector>

#include "logging/recorder.hpp"
#include "rom_bootloader.hpp"
#include "telemetry/telemetry.hpp"

namespace RadioUpdate {

struct Snapshot {
  Phase phase{Phase::Idle};
  bool busy{false};
  uint32_t revision{0};
  char filename[kNameSize]{};
  ImageInfo image{};
  size_t fileCount{0};
  uint8_t link{0};
  uint8_t percent{0};
  LinkResult results[2]{};
  char error[96]{};
  uint32_t stackFreeBytes{0};
};

class RadioFirmwareUpdater {
 public:
  RadioFirmwareUpdater(Recorder& recorder, Telemetry& first, Telemetry& second)
      : recorder(recorder), links{&first, &second} {}
  bool begin();
  bool browse();
  bool select(size_t index);
  bool program();
  bool close();
  Snapshot snapshot() const;
  bool filename(size_t index, char* destination, size_t capacity) const;
  void progress(Phase phase, uint8_t percent);

 private:
  enum class Action : uint8_t { Browse, Select, Program, Close };
  struct Request {
    Action action{Action::Browse};
    char filename[kNameSize]{};
  };
  struct Filename {
    char value[kNameSize]{};
  };
  Recorder& recorder;
  Telemetry* links[2];
  QueueHandle_t queue{nullptr};
  TaskHandle_t task{nullptr};
  mutable portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
  Snapshot state{};
  std::vector<Filename> catalog{};
  File file{};
  bool recorderPaused{false};
  bool storageOwned{false};
  bool submit(const Request& request, Phase phase);
  void scan();
  void open(const char* name);
  void updateBoth();
  void release();
  void finish(Phase phase, const char* error = "");
  static void worker(void* parameter);
};

}  // namespace RadioUpdate
