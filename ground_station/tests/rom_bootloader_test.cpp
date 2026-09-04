#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include "telemetry/crc.hpp"
#include "update/rom_bootloader.hpp"

using namespace RadioUpdate;

class MemorySource final : public Source {
 public:
  explicit MemorySource(size_t size = 3000) : bytes(size, 0xA5) {
    const uint32_t sp = 0x20009000;
    const uint32_t reset = kFlashBase + 0x101;
    memcpy(bytes.data(), &sp, 4);
    memcpy(bytes.data() + 4, &reset, 4);
  }
  uint32_t size() override { return bytes.size(); }
  bool read(uint32_t offset, uint8_t* destination, size_t length) override {
    if (offset + length > bytes.size()) return false;
    memcpy(destination, bytes.data() + offset, length);
    return true;
  }
  std::vector<uint8_t> bytes;
};

class FakePort final : public Port {
 public:
  bool write(const uint8_t* data, size_t size) override {
    if (!rom) {
      if (size == 11 && data[0] == 0x80) {
        const uint8_t frame[] = {0x80, 2, 1, 0x79, 0x42};
        queue(frame, sizeof(frame) - 1);
        const uint8_t content[] = {0x80, 2, 1, 0x79};
        rx.push_back(crc8(content, sizeof(content)));
      } else if (size == 3 && data[0] == 0x60) {
        std::vector<uint8_t> response{0x60, 5, '1', '.', '2', '.', '0'};
        response.push_back(crc8(response.data(), response.size()));
        queue(response.data(), response.size());
      }
      return true;
    }
    if (size == 1 && data[0] == 0x7F) {
      rx.push_back(0x79);
      return true;
    }
    if (stage == 0 && size == 2 && data[1] == static_cast<uint8_t>(data[0] ^ 0xFFU)) {
      command = data[0];
      stage = 1;
      rx.push_back(0x79);
      if (command == 0x00) {
        const uint8_t response[] = {7, 0x31, 0x00, 0x01, 0x02, 0x11, 0x21, 0x31, 0x43, 0x79};
        queue(response, sizeof(response));
        stage = 0;
      } else if (command == 0x01) {
        const uint8_t response[] = {0x31, 0, 0, 0x79};
        queue(response, sizeof(response));
        stage = 0;
      } else if (command == 0x02) {
        const uint8_t response[] = {1, 4, 0x60, 0x79};
        queue(response, sizeof(response));
        stage = 0;
      }
      return true;
    }
    if ((command == 0x11 || command == 0x31 || command == 0x21) && stage == 1 && size == 5) {
      address = (uint32_t{data[0]} << 24U) | (uint32_t{data[1]} << 16U) | (uint32_t{data[2]} << 8U) | data[3];
      rx.push_back(0x79);
      stage = 2;
      return true;
    }
    if (command == 0x11 && stage == 2 && size == 2) {
      const size_t count = size_t{data[0]} + 1;
      rx.push_back(0x79);
      std::vector<uint8_t> result(count, 0);
      if (address >= kFlashBase && address + count <= kFlashBase + flash.size()) {
        memcpy(result.data(), flash.data() + address - kFlashBase, count);
      } else if (address == 0x1FFF75E0U) {
        result[0] = 128;
      } else if (address == 0x1FFF6FFEU) {
        result[0] = revision;
      } else if (address == 0x1FFF7870U) {
        result[0] = protectedFlash ? 1 : 0;
      } else if (address >= 0x1FFF7800U && address + count <= 0x1FFF7800U + options.size()) {
        memcpy(result.data(), options.data() + address - 0x1FFF7800U, count);
      } else
        return false;
      queue(result.data(), result.size());
      stage = 0;
      return true;
    }
    if (command == 0x31 && stage == 2 && size >= 10) {
      const size_t count = size_t{data[0]} + 1;
      assert(count % 8 == 0 && count <= kBlockSize);
      memcpy(flash.data() + address - kFlashBase, data + 1, count);
      writes.push_back(address);
      if (!dropWriteAck) rx.push_back(0x79);
      dropWriteAck = false;
      stage = 0;
      return true;
    }
    if (command == 0x43 && stage == 1 && size == 3) {
      const size_t page = data[1];
      std::fill(flash.begin() + page * kPageSize, flash.begin() + (page + 1) * kPageSize, 0xFF);
      rx.push_back(0x79);
      stage = 0;
      return true;
    }
    if (command == 0x21 && stage == 2) {
      stage = 0;
      return true;
    }
    return false;
  }
  bool read(uint8_t* data, size_t size, uint32_t) override {
    if (rx.size() < size) {
      time += 25;
      return false;
    }
    for (size_t index = 0; index < size; ++index) {
      data[index] = rx.front();
      rx.pop_front();
    }
    return true;
  }
  void configure(bool inRom) override { rom = inRom; }
  void drain() override { rx.clear(); }
  void wait(uint32_t ms) override { time += ms; }
  uint32_t now() override { return time++; }
  void progress(Phase, uint8_t) override {}
  void queue(const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; ++i) rx.push_back(data[i]);
  }

  FakePort() : flash(kFlashSize, 0), options(0x74, 0) {
    options[0] = 0xAA;
    options[8] = 0xFF;
    options[16] = 0;
    options[24] = 0x3F;
    options[26] = 0;
    options[32] = 0x3F;
    options[34] = 0;
    options[40] = 0xFF;
    options[48] = 0;
  }
  std::vector<uint8_t> flash;
  std::vector<uint8_t> options;
  std::deque<uint8_t> rx;
  std::vector<uint32_t> writes;
  uint32_t time{0};
  uint32_t address{0};
  uint8_t command{0};
  uint8_t stage{0};
  uint8_t revision{0xB2};
  bool rom{false};
  bool dropWriteAck{true};
  bool protectedFlash{false};
};

int main() {
  MemorySource image;
  ImageInfo info{};
  assert(inspect(image, info));
  assert(info.size == image.bytes.size());
  assert(validVectors(info.vectors, info.size));
  assert(isBinName("telemetry.bin") && isBinName("A.BIN"));
  assert(!isBinName("telemetry.bin.old") && !isBinName("folder/telemetry.bin"));

  FakePort port;
  LinkResult result{};
  RomBootloader updater(port);
  if (!updater.run(image, info, result)) {
    std::cerr << updater.error() << '\n';
    return 1;
  }
  assert(result.success && result.destructive && result.romRevision == 0xB2);
  assert(std::string(result.version) == "1.2.0");
  assert(port.writes.back() == kFlashBase);  // Activation vectors are last.
  assert(std::equal(image.bytes.begin(), image.bytes.end(), port.flash.begin()));

  FakePort oldRom;
  oldRom.revision = 0xB0;
  RomBootloader rejected(oldRom);
  assert(!rejected.run(image, info, result));
  assert(!result.destructive && oldRom.writes.empty());

  FakePort protectedRom;
  protectedRom.protectedFlash = true;
  RomBootloader protectedUpdater(protectedRom);
  assert(!protectedUpdater.run(image, info, result));
  assert(!result.destructive && protectedRom.writes.empty());

  image.bytes[0] = 1;
  assert(!inspect(image, info));
  return 0;
}
