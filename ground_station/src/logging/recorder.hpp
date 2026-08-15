/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "config.hpp"
#include "telemetry/packetSink.hpp"
#include "utils.hpp"

enum class RecorderState : uint8_t { Idle, Recording, Finalizing, Fault };
enum class FinalizeReason : uint8_t { MissionComplete, UserRequested, Shutdown };
constexpr size_t kLogFilenameSize = 64;

struct RecorderStatus {
  RecorderState state{RecorderState::Idle};
  char activeFilename[kLogFilenameSize]{};
  uint8_t participantMask{0};
  uint32_t writtenSamples{0};
  uint32_t droppedSamples{0};
  char lastError[64]{};
};

struct LogEntry {
  char name[kLogFilenameSize]{};
  size_t sizeBytes{0};
  bool active{false};
};

class Recorder : public ITelemetryPacketSink {
 public:
  explicit Recorder(const char* directory) : directory(directory) {}
  bool begin();

  void enable() { enabled = true; }
  void disable() { enabled = false; }
  void configure(ReceiverTelemetryMode_e mode, bool neverStop) {
    receiverMode = mode;
    neverStopLogging = neverStop;
  }

  void onTelemetryPacket(const packedRXMessage& packet, uint8_t source) override;
  RecorderStatus getStatus() const;
  bool shareWithMassStorage();
  bool sync();
  bool finalize(FinalizeReason reason = FinalizeReason::UserRequested);
  bool deleteLog(const char* name);
  bool refreshCatalog(std::vector<LogEntry>& entries);

  const char* getDirectory() const { return directory; }

 private:
  enum class CommandType : uint8_t { Sample, Finalize, Sync, CatalogRefresh, Delete, ShareMassStorage };
  struct Command {
    CommandType type{CommandType::Sample};
    packedRXMessage data{};
    uint8_t source{0};
    FinalizeReason reason{FinalizeReason::MissionComplete};
    char name[kLogFilenameSize]{};
    uint32_t requestId{0};
  };
  struct Response {
    uint32_t requestId{0};
    bool result{false};
  };

  bool initialized{false};
  std::atomic<bool> enabled{false};
  bool fileCreated{false};
  bool armed{true};
  bool neverStopLogging{false};
  ReceiverTelemetryMode_e receiverMode{SINGLE};
  uint8_t touchdownMask{0};
  uint8_t rearmMask{0};
  uint8_t completedParticipantMask{0};
  uint32_t unsyncedRows{0};

  const char* directory;
  char fileName[kLogFilenameSize]{};
  QueueHandle_t queue{nullptr};
  QueueHandle_t responseQueue{nullptr};
  SemaphoreHandle_t fsMutex{nullptr};
  SemaphoreHandle_t catalogMutex{nullptr};
  mutable portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
  File file{};
  RecorderStatus status{};
  std::vector<LogEntry> catalogCache{};
  uint32_t nextRequestId{0};

  bool chooseNextFileName();
  bool createFile();
  bool writeSample(const Command& command);
  bool finalizeFile(FinalizeReason reason);
  bool deleteFile(const char* name);
  bool scanCatalog(std::vector<LogEntry>& entries);
  void setFault(const char* message);
  bool submitAndWait(Command& command);
  static bool isCsvLog(const char* name);
  static int32_t logNumber(const char* name);
  static void recordTask(void* pvParameter);
};
