/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "rom_bootloader.hpp"

#include <algorithm>
#include <cstring>

#include "telemetry/crc.hpp"

namespace RadioUpdate {
namespace {
constexpr uint8_t kAck = 0x79;
constexpr uint8_t kVersionCommand = 0x60;
constexpr uint8_t kBootCommand = 0x80;

uint32_t littleWord(const uint8_t* bytes) {
  return uint32_t{bytes[0]} | (uint32_t{bytes[1]} << 8U) | (uint32_t{bytes[2]} << 16U) | (uint32_t{bytes[3]} << 24U);
}

bool sameImage(const ImageInfo& lhs, const ImageInfo& rhs) {
  return lhs.size == rhs.size && lhs.crc == rhs.crc && memcmp(lhs.vectors, rhs.vectors, 8) == 0;
}
}  // namespace

uint32_t crcUpdate(uint32_t state, const uint8_t* data, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    state ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      state = (state >> 1U) ^ ((state & 1U) != 0U ? 0xEDB88320U : 0U);
    }
  }
  return state;
}

bool validVectors(const uint8_t* vectors, uint32_t size) {
  const uint32_t sp = littleWord(vectors);
  const uint32_t reset = littleWord(vectors + 4);
  return size >= kBlockSize && size <= kFlashSize && (sp & 7U) == 0U && sp >= 0x20000000U && sp <= 0x20009000U &&
         (reset & 1U) != 0U && (reset & ~1U) >= kFlashBase && (reset & ~1U) < kFlashBase + size;
}

bool isBinName(const char* name) {
  if (name == nullptr) {
    return false;
  }
  const size_t length = strlen(name);
  if (length <= 4 || length >= kNameSize || strchr(name, '/') != nullptr || strchr(name, '\\') != nullptr) {
    return false;
  }
  const char* suffix = name + length - 4;
  return suffix[0] == '.' && (suffix[1] == 'b' || suffix[1] == 'B') && (suffix[2] == 'i' || suffix[2] == 'I') &&
         (suffix[3] == 'n' || suffix[3] == 'N');
}

bool inspect(Source& source, ImageInfo& info) {
  info = ImageInfo{};
  info.size = source.size();
  if (info.size < kBlockSize || info.size > kFlashSize || !source.read(0, info.vectors, 8) ||
      !validVectors(info.vectors, info.size)) {
    return false;
  }
  uint8_t block[kBlockSize]{};
  uint32_t crc = ~0U;
  for (uint32_t offset = 0; offset < info.size; offset += sizeof(block)) {
    const size_t count = std::min<uint32_t>(sizeof(block), info.size - offset);
    if (!source.read(offset, block, count)) {
      return false;
    }
    crc = crcUpdate(crc, block, count);
  }
  info.crc = crc ^ ~0U;
  return source.size() == info.size;
}

const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::Idle:
      return "Idle";
    case Phase::Preparing:
      return "Preparing";
    case Phase::Browsing:
      return "Choose firmware";
    case Phase::Validating:
      return "Checking file";
    case Phase::Confirm:
      return "Confirm update";
    case Phase::Entering:
      return "Entering bootloader";
    case Phase::Identifying:
      return "Checking device";
    case Phase::Erasing:
      return "Erasing";
    case Phase::Writing:
      return "Writing";
    case Phase::Verifying:
      return "Verifying";
    case Phase::Activating:
      return "Activating";
    case Phase::Starting:
      return "Starting radio";
    case Phase::Complete:
      return "Both radios updated";
    case Phase::Failed:
      return "Update stopped";
    case Phase::Closing:
      return "Restoring operation";
  }
  return "Unknown";
}

bool RomBootloader::fail(const char* message) {
  failure = message;
  return false;
}

bool RomBootloader::ack(uint32_t timeoutMs) {
  uint8_t byte = 0;
  return port.read(&byte, 1, timeoutMs) && byte == kAck;
}

bool RomBootloader::command(uint8_t value) {
  const uint8_t bytes[] = {value, static_cast<uint8_t>(value ^ 0xFFU)};
  return port.write(bytes, sizeof(bytes)) && ack();
}

bool RomBootloader::address(uint32_t value) {
  uint8_t bytes[] = {static_cast<uint8_t>(value >> 24U), static_cast<uint8_t>(value >> 16U),
                     static_cast<uint8_t>(value >> 8U), static_cast<uint8_t>(value), 0};
  bytes[4] = bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3];
  return port.write(bytes, sizeof(bytes)) && ack();
}

bool RomBootloader::frame(uint8_t expectedCommand, uint8_t* payload, size_t capacity, size_t& size,
                          uint32_t timeoutMs) {
  uint8_t buffer[67]{};
  size_t used = 0;
  const uint32_t started = port.now();
  while (port.now() - started < timeoutMs) {
    uint8_t byte = 0;
    if (!port.read(&byte, 1, 20)) {
      used = 0;
      continue;
    }
    if (used == 0 && byte != expectedCommand) {
      continue;
    }
    buffer[used++] = byte;
    if (used == 2 && (byte == 0 || byte > capacity || byte > sizeof(buffer) - 3)) {
      used = 0;
    } else if (used >= 3 && used == static_cast<size_t>(buffer[1]) + 3) {
      if (crc8(buffer, used - 1) == byte) {
        size = buffer[1];
        memcpy(payload, buffer + 2, size);
        return true;
      }
      used = 0;
    }
  }
  return false;
}

bool RomBootloader::enter(LinkResult& result) {
  port.progress(Phase::Entering, 0);
  port.configure(false);
  port.drain();
  uint8_t disable[] = {0x21, 0, 0};
  disable[2] = crc8(disable, 2);
  if (!port.write(disable, sizeof(disable))) {
    return fail("UART write failed");
  }
  port.wait(100);
  port.drain();
  uint8_t request[] = {kBootCommand, 8, 'C', 'A', 'T', 'S', 'B', 'L', 1, 0xA5, 0};
  request[10] = crc8(request, 10);
  result.entryRequested = true;  // An ACK can be lost after the application has jumped.
  uint8_t response[2]{};
  size_t length = 0;
  if (!port.write(request, sizeof(request)) || !frame(kBootCommand, response, 2, length, 1000) || length != 2 ||
      response[0] != 1 || response[1] != kAck) {
    return fail("No entry ACK; ST-Link bootstrap/recovery");
  }
  port.wait(50);
  port.configure(true);
  port.drain();
  constexpr uint8_t sync = 0x7F;
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (port.write(&sync, 1) && ack(500)) {
      return true;
    }
    port.wait(50);
    port.drain();
  }
  return fail("ROM sync failed; ST-Link recovery");
}

bool RomBootloader::readMemory(uint32_t location, uint8_t* data, size_t size) {
  if (size == 0 || size > kBlockSize || !command(0x11) || !address(location)) {
    return false;
  }
  const auto n = static_cast<uint8_t>(size - 1);
  const uint8_t bytes[] = {n, static_cast<uint8_t>(n ^ 0xFFU)};
  return port.write(bytes, sizeof(bytes)) && ack() && port.read(data, size, 1000);
}

bool RomBootloader::identify(LinkResult& result) {
  port.progress(Phase::Identifying, 2);
  uint8_t count = 0;
  uint8_t capabilities[32]{};
  if (!command(0x00) || !port.read(&count, 1, 1000) || count >= sizeof(capabilities) ||
      !port.read(capabilities, static_cast<size_t>(count) + 1, 1000) || !ack() || capabilities[0] != 0x31) {
    return fail("Unsupported ROM capabilities");
  }
  const auto has = [&](uint8_t value) {
    return std::find(capabilities + 1, capabilities + count + 1, value) != capabilities + count + 1;
  };
  extendedErase = has(0x44);
  if (!has(0x01) || !has(0x02) || !has(0x11) || !has(0x21) || !has(0x31) || (!extendedErase && !has(0x43))) {
    return fail("ROM commands unavailable/protected");
  }
  uint8_t versionInfo[3]{};
  if (!command(0x01) || !port.read(versionInfo, 3, 1000) || !ack() || versionInfo[0] != 0x31) {
    return fail("Unsupported ROM protocol version");
  }
  uint8_t id[3]{};
  if (!command(0x02) || !port.read(id, 3, 1000) || !ack() || id[0] != 1 || id[1] != 4 || id[2] != 0x60) {
    return fail("Wrong MCU; expected STM32G071/081");
  }
  uint8_t geometry[2]{};
  if (!readMemory(0x1FFF75E0U, geometry, 2) || geometry[0] != 128 || geometry[1] != 0) {
    return fail("Wrong or unreadable flash size");
  }
  if (!readMemory(0x1FFF6FFEU, &result.romRevision, 1) || result.romRevision < 0xB1 || result.romRevision > 0xB4) {
    return fail("Unsupported ROM revision (need B1-B4)");
  }
  // RM0444: option words are spaced eight bytes apart (word + complement).
  // Read only: never issue write/read unprotect or change option bytes.
  uint8_t options[56]{};
  uint8_t security[4]{};
  if (!readMemory(0x1FFF7800U, options, sizeof(options)) || !readMemory(0x1FFF7870U, security, 4)) {
    return fail("Cannot read protection settings");
  }
  const auto wrpActive = [&](size_t offset) { return (options[offset] & 0x3FU) <= (options[offset + 2] & 0x3FU); };
  if (options[0] != 0xAA || wrpActive(24) || wrpActive(32) || options[8] <= options[16] || options[40] <= options[48] ||
      (security[0] & 0x7FU) != 0) {
    return fail("Protected flash; separate provisioning needed");
  }
  return true;
}

bool RomBootloader::erasePage(uint16_t page) {
  if (!command(extendedErase ? 0x44 : 0x43)) {
    return false;
  }
  uint8_t extended[] = {0, 0, static_cast<uint8_t>(page >> 8U), static_cast<uint8_t>(page), 0};
  extended[4] = extended[2] ^ extended[3];
  const uint8_t basic[] = {0, static_cast<uint8_t>(page), static_cast<uint8_t>(page)};
  const bool sent = extendedErase ? port.write(extended, sizeof(extended)) : port.write(basic, sizeof(basic));
  if (!sent || !ack(5000)) {
    return false;
  }
  // AN2606: B3 does not wait for erase BUSY. Never batch pages; wait >=40ms.
  port.wait(40);
  uint8_t block[kBlockSize]{};
  for (uint32_t offset = 0; offset < kPageSize; offset += sizeof(block)) {
    if (!readMemory(kFlashBase + uint32_t{page} * kPageSize + offset, block, sizeof(block)) ||
        !std::all_of(block, block + sizeof(block), [](uint8_t value) { return value == 0xFF; })) {
      return false;
    }
  }
  return true;
}

bool RomBootloader::writeMemory(uint32_t location, const uint8_t* data, size_t size) {
  if (size == 0 || size > kBlockSize || (size & 7U) != 0 || (location & 7U) != 0 || !command(0x31) ||
      !address(location)) {
    return false;
  }
  uint8_t buffer[kBlockSize + 2]{};
  buffer[0] = static_cast<uint8_t>(size - 1);
  memcpy(buffer + 1, data, size);
  uint8_t checksum = buffer[0];
  for (size_t index = 0; index < size; ++index) {
    checksum ^= data[index];
  }
  buffer[size + 1] = checksum;
  if (!port.write(buffer, size + 2)) {
    return false;
  }
  if (!ack()) {
    // A lost final ACK may still have programmed the double words. Never write
    // them again: G0 ECC programming is not safely repeatable without erase.
    port.wait(40);
    port.drain();
  }
  uint8_t actual[kBlockSize]{};
  return readMemory(location, actual, size) && memcmp(actual, data, size) == 0;
}

bool RomBootloader::writeRange(Source& source, uint32_t start, uint32_t end, uint8_t low, uint8_t high) {
  uint8_t bytes[kBlockSize]{};
  for (uint32_t offset = start; offset < end; offset += sizeof(bytes)) {
    const size_t length = std::min<uint32_t>(sizeof(bytes), end - offset);
    memset(bytes, 0xFF, sizeof(bytes));
    if (!source.read(offset, bytes, length) || !writeMemory(kFlashBase + offset, bytes, (length + 7U) & ~7U)) {
      return false;
    }
    port.progress(Phase::Writing, static_cast<uint8_t>(low + (high - low) * (offset + length - start) / (end - start)));
  }
  return true;
}

bool RomBootloader::verify(Source& source, const ImageInfo& info, bool activated) {
  uint8_t expected[kBlockSize]{};
  uint8_t actual[kBlockSize]{};
  uint32_t crc = ~0U;
  const uint32_t start = activated ? 0 : 8;
  if (!activated) {
    crc = crcUpdate(crc, info.vectors, 8);
    if (!readMemory(kFlashBase, actual, 8) ||
        !std::all_of(actual, actual + 8, [](uint8_t byte) { return byte == 0xFF; })) {
      return false;
    }
  }
  for (uint32_t offset = start; offset < info.size; offset += sizeof(actual)) {
    const size_t count = std::min<uint32_t>(sizeof(actual), info.size - offset);
    if (!source.read(offset, expected, count) || !readMemory(kFlashBase + offset, actual, count) ||
        memcmp(expected, actual, count) != 0) {
      return false;
    }
    crc = crcUpdate(crc, actual, count);
    const uint8_t low = activated ? 90 : 72;
    port.progress(Phase::Verifying, static_cast<uint8_t>(low + 8U * (offset + count) / info.size));
  }
  return source.size() == info.size && (crc ^ ~0U) == info.crc;
}

bool RomBootloader::version(char* output, size_t capacity) {
  const uint32_t started = port.now();
  uint8_t request[] = {kVersionCommand, 0, 0};
  request[2] = crc8(request, 2);
  uint8_t payload[31]{};
  while (port.now() - started < 8000) {
    size_t size = 0;
    if (!port.write(request, sizeof(request))) {
      return false;
    }
    if (frame(kVersionCommand, payload, sizeof(payload), size, 200)) {
      if (size > 0 && payload[size - 1] == 0) {
        --size;
      }
      if (size != 0 && size < capacity &&
          std::all_of(payload, payload + size, [](uint8_t byte) { return byte >= 0x20 && byte <= 0x7E; })) {
        memcpy(output, payload, size);
        output[size] = '\0';
        return true;
      }
    }
  }
  return false;
}

bool RomBootloader::run(Source& source, const ImageInfo& expected, LinkResult& result) {
  result = LinkResult{};
  result.attempted = true;
  failure = "";
  ImageInfo current{};
  if (!inspect(source, current) || !sameImage(expected, current)) {
    return fail("Source file changed or invalid");
  }
  if (!enter(result) || !identify(result)) {
    return false;
  }
  const uint32_t pages = (expected.size + kPageSize - 1) / kPageSize;
  for (uint32_t page = 0; page < pages; ++page) {
    port.progress(Phase::Erasing, static_cast<uint8_t>(5 + 15U * page / pages));
    result.destructive = true;  // Set BEFORE sending the first erase command.
    if (!erasePage(static_cast<uint16_t>(page))) {
      return fail("Erase/blank check failed; ST-Link recovery");
    }
  }
  if (!writeRange(source, 256, expected.size, 20, 68) || !writeRange(source, 8, 256, 68, 72)) {
    return fail("Write/readback failed; ST-Link recovery");
  }
  if (!verify(source, expected, false) || !inspect(source, current) || !sameImage(expected, current)) {
    return fail("Verification/source changed; ST-Link recovery");
  }
  port.progress(Phase::Activating, 85);
  if (!writeMemory(kFlashBase, expected.vectors, 8) || !verify(source, expected, true)) {
    return fail("Activation/verification failed; ST-Link recovery");
  }
  port.progress(Phase::Starting, 99);
  if (!command(0x21) || !address(kFlashBase)) {
    return fail("Application start failed; ST-Link recovery");
  }
  port.configure(false);
  port.drain();
  if (!version(result.version, sizeof(result.version))) {
    return fail("No application version; ST-Link recovery");
  }
  result.success = true;
  return true;
}

}  // namespace RadioUpdate
