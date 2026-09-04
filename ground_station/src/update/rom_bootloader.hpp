/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>

namespace RadioUpdate {

constexpr uint32_t kFlashBase = 0x08000000U;
constexpr uint32_t kFlashSize = 128U * 1024U;
constexpr uint32_t kPageSize = 2048U;
constexpr size_t kBlockSize = 256U;
constexpr size_t kNameSize = 128U;
constexpr size_t kMaxFiles = 64U;

enum class Phase : uint8_t {
  Idle,
  Preparing,
  Browsing,
  Validating,
  Confirm,
  Entering,
  Identifying,
  Erasing,
  Writing,
  Verifying,
  Activating,
  Starting,
  Complete,
  Failed,
  Closing
};

struct ImageInfo {
  uint32_t size{0};
  uint32_t crc{0};
  uint8_t vectors[8]{};
};

struct LinkResult {
  bool attempted{false};
  bool success{false};
  bool entryRequested{false};
  bool destructive{false};
  uint8_t romRevision{0};
  char version[32]{};
};

// Only the byte transport and file reads are substituted in native tests. The
// production protocol, validation, erase/write ordering and verification run unchanged.
class Port {
 public:
  virtual ~Port() = default;
  virtual bool write(const uint8_t* data, size_t size) = 0;
  virtual bool read(uint8_t* data, size_t size, uint32_t timeoutMs) = 0;
  virtual void configure(bool rom) = 0;
  virtual void drain() = 0;
  virtual void wait(uint32_t ms) = 0;
  virtual uint32_t now() = 0;
  virtual void progress(Phase phase, uint8_t percent) = 0;
};

class Source {
 public:
  virtual ~Source() = default;
  virtual uint32_t size() = 0;
  virtual bool read(uint32_t offset, uint8_t* data, size_t length) = 0;
};

uint32_t crcUpdate(uint32_t state, const uint8_t* data, size_t size);
bool validVectors(const uint8_t* vectors, uint32_t size);
bool isBinName(const char* name);
bool inspect(Source& source, ImageInfo& info);
const char* phaseName(Phase phase);

class RomBootloader {
 public:
  explicit RomBootloader(Port& port) : port(port) {}
  // No retries of an entire update and no automatic reset/recovery after failure.
  bool run(Source& source, const ImageInfo& expected, LinkResult& result);
  const char* error() const { return failure; }

 private:
  Port& port;
  bool extendedErase{false};
  const char* failure{""};
  bool fail(const char* message);
  bool ack(uint32_t timeoutMs = 1000);
  bool command(uint8_t value);
  bool address(uint32_t value);
  bool enter(LinkResult& result);
  bool identify(LinkResult& result);
  bool readMemory(uint32_t address, uint8_t* data, size_t size);
  bool writeMemory(uint32_t address, const uint8_t* data, size_t size);
  bool erasePage(uint16_t page);
  bool writeRange(Source& source, uint32_t start, uint32_t end, uint8_t low, uint8_t high);
  bool verify(Source& source, const ImageInfo& info, bool activated);
  bool version(char* output, size_t capacity);
  bool frame(uint8_t command, uint8_t* payload, size_t capacity, size_t& size, uint32_t timeoutMs);
};

}  // namespace RadioUpdate
