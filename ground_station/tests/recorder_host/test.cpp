// Copyright (C) 2026 Control and Telemetry Systems GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "hmi_host.hpp"
#include "Adafruit_FlashCache.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

static void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
namespace host {
std::function<void()> beforeReplyDelivery;
std::atomic<uint32_t> clockOffset{0};
unsigned scans = 0;
bool failSend = false;
std::vector<HostQueue*> queues;
std::vector<SemaphoreHandle_t> mutexes;
std::thread worker;
struct Stop {};
void shutdown() {
  for (auto* queue : queues) {
    std::lock_guard<std::mutex> lock(queue->mutex);
    queue->stopped = true;
    queue->changed.notify_all();
  }
  if (worker.joinable()) worker.join();
  for (auto* queue : queues) delete queue;
  for (auto* mutex : mutexes) delete mutex;
}
}
SemaphoreHandle_t storageAccessMutex = nullptr;
QueueHandle_t xQueueCreate(size_t capacity, size_t itemSize) {
  auto* queue = new HostQueue{capacity, itemSize};
  host::queues.push_back(queue);
  return queue;
}
int xQueueSend(QueueHandle_t queue, const void* item, TickType_t timeout) {
  if (timeout && host::failSend) return 0;
  std::unique_lock<std::mutex> lock(queue->mutex);
  if (!queue->changed.wait_for(lock, std::chrono::milliseconds(timeout),
                               [&] { return queue->items.size() < queue->capacity; })) return 0;
  const auto* bytes = static_cast<const uint8_t*>(item);
  queue->items.emplace_back(bytes, bytes + queue->itemSize);
  queue->changed.notify_all();
  return pdPASS;
}
int xQueueReceive(QueueHandle_t queue, void* item, TickType_t timeout) {
  std::unique_lock<std::mutex> lock(queue->mutex);
  const auto ready = [&] { return queue->stopped || !queue->items.empty(); };
  if (timeout == portMAX_DELAY) queue->changed.wait(lock, ready);
  else if (!queue->changed.wait_for(lock, std::chrono::milliseconds(timeout), ready)) return 0;
  if (queue->stopped) throw host::Stop{};
  std::memcpy(item, queue->items.front().data(), queue->itemSize);
  queue->items.pop_front();
  queue->changed.notify_all();
  lock.unlock();
  // Delay delivery, not worker processing: the recorder can close the log before
  // the UI receives the earlier reply. This forces the identified race window.
  if (queue == host::queues.at(1) && host::beforeReplyDelivery) {
    auto hook = std::move(host::beforeReplyDelivery);
    host::beforeReplyDelivery = nullptr;
    hook();
  }
  return pdPASS;
}
SemaphoreHandle_t xSemaphoreCreateMutex() {
  auto* mutex = new std::timed_mutex;
  host::mutexes.push_back(mutex);
  return mutex;
}
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout) {
  if (mutex == storageAccessMutex) ++host::scans;
  if (timeout == portMAX_DELAY) { mutex->lock(); return pdTRUE; }
  return mutex->try_lock_for(std::chrono::milliseconds(timeout)) ? pdTRUE : 0;
}
void xSemaphoreGive(SemaphoreHandle_t mutex) { mutex->unlock(); }
void xTaskCreate(void (*task)(void*), const char*, size_t, void* argument, int, void*) {
  host::worker = std::thread([=] { try { task(argument); } catch (host::Stop&) {} });
}
TickType_t xTaskGetTickCount() {
  static const auto start = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
}
uint32_t millis() { return xTaskGetTickCount() + host::clockOffset.load(); }

class Flash : public FsBlockDeviceInterface {
 public:
  Adafruit_SPIFlashBase raw;
  Adafruit_FlashCache cache;
  bool failSync = false;
  bool isBusy() override { return false; }
  Sector_t sectorCount() override { return raw.bytes.size() / 512; }
  bool readSector(Sector_t sector, uint8_t* dst) override { return readSectors(sector, dst, 1); }
  bool readSectors(Sector_t sector, uint8_t* dst, size_t count) override {
    check((sector + count) * 512 <= raw.bytes.size(), "read beyond flash");
    return cache.read(&raw, sector * 512, dst, count * 512);
  }
  bool writeSector(Sector_t sector, const uint8_t* src) override { return writeSectors(sector, src, 1); }
  bool writeSectors(Sector_t sector, const uint8_t* src, size_t count) override {
    check((sector + count) * 512 <= raw.bytes.size(), "write beyond flash");
    return cache.write(&raw, sector * 512, src, count * 512);
  }
  bool syncDevice() override { return !failSync && cache.sync(&raw); }
} flash;
FatFileSystem fatfs;
Config systemConfig;
static bool hostOwned = false;
bool Utils::isConnected() { return true; }
bool Utils::isFilesystemAvailable() { return !hostOwned; }
bool Utils::requestMassStorage() {
  check(fatfs.end() != nullptr && flash.syncDevice(), "USB handoff flush");
  hostOwned = true;
  return true;
}
bool Utils::claimFirmwareStorage(uint32_t) {
  if (hostOwned) check(fatfs.begin(&flash), "USB reclaim remount");
  hostOwned = false;
  return true;
}

static std::map<std::string, std::string> expected;
static const std::string header = "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/s],battery[decivolts],pyro1,pyro2\r\n";
static packedRXMessage packet(uint8_t state, unsigned timestamp) {
  packedRXMessage data{};
  data.state = state; data.timestamp = timestamp;
  data.lat = 470000; data.lon = 80000; data.altitude = 123; data.velocity = 42; data.voltage = 80;
  return data;
}
static std::string row(uint8_t state, unsigned timestamp, uint8_t source) {
  return std::to_string(source) + ',' + std::to_string(timestamp) + ',' + std::to_string(state) +
         ",0,470000,80000,123,42,80,0,0\r\n";
}
static void send(Recorder& recorder, uint8_t state, unsigned timestamp, uint8_t source = 1) {
  recorder.onTelemetryPacket(packet(state, timestamp), source);
  check(recorder.sync(), "drain queued telemetry");
}
static std::string beginSegment(Recorder& recorder, uint8_t state = 3, unsigned timestamp = 1) {
  check(recorder.startLiveRecording(), "enter Live");
  send(recorder, state, timestamp);
  const auto status = recorder.getStatus();
  check(status.state == RecorderState::Recording, "mid-flight entry starts recording");
  std::string name = status.activeFilename;
  check(!expected.count(name), "new segment must not reuse an old filename");
  expected[name] = header + row(state, timestamp, 1);
  return name;
}
static void verifyDisk() {
  // Fresh filesystem AND flash cache: only persisted bytes count.
  Flash fresh;
  fresh.raw.bytes = flash.raw.bytes;
  FatVolume mounted;
  check(mounted.begin(&fresh), "persisted filesystem mounts");
  check(mounted.fatType() == 12, "test uses GS FAT12 geometry");
  auto directory = mounted.open("/logs");
  check(bool(directory), "persisted /logs exists");
  std::set<std::string> names;
  for (auto file = directory.openNextFile(); file; file = directory.openNextFile()) {
    char name[64]{};
    file.getName(name, sizeof(name));
    check(expected.count(name) != 0, "unexpected/corrupt directory entry");
    std::string bytes(file.size(), '\0');
    check(file.read(bytes.data(), bytes.size()) == static_cast<int>(bytes.size()), "read full persisted log");
    check(bytes == expected.at(name), "persisted log contents changed or samples lost");
    names.insert(name);
    file.close();
  }
  check(names.size() == expected.size(), "one or more logs disappeared");
  directory.close();
  mounted.end();
}
static void testDown(Recorder& recorder) {
  for (bool dual : {false, true}) for (bool never : {false, true}) {
    recorder.configure(dual ? DUAL : SINGLE, never);
    check(recorder.shareWithMassStorage(), "idle USB sharing");
    check(recorder.startLiveRecording(), "re-enter Live at DOWN");
    for (unsigned i = 0; i < 100; ++i) send(recorder, 7, i, i % 2 + 1);
    check(recorder.getStatus().state == RecorderState::Idle, "DOWN must not create a new log");
    check(hostOwned, "DOWN alone must not reclaim USB for recording");
    Hmi hmi(recorder); hmi.backButton.pressed = true; hmi.live();
    check(hmi.state == Hmi::MENU && hmi.liveExitState == Hmi::LiveExitState::None,
          "B must leave DOWN immediately without confirmation");
    verifyDisk();
    Utils::claimFirmwareStorage();
  }
  std::cout << "PASS: DOWN re-entry, both receiver modes and stop settings, no new files\n";
}
static void testExit(Recorder& recorder) {
  recorder.configure(SINGLE, false);
  const auto name = beginSegment(recorder);
  Hmi hmi(recorder);
  host::beforeReplyDelivery = [&] {
    send(recorder, 7, 2);
    expected[name] += row(7, 2, 1);
    check(recorder.getStatus().activeFilename[0] == '\0', "touchdown happens before reply delivery");
  };
  hmi.backButton.pressed = true; hmi.live();
  check(hmi.liveExitState == Hmi::LiveExitState::Confirm, "exit must use its reply, never show false Stop Failed");
  hmi.okButton.pressed = true; hmi.live();
  check(hmi.state == Hmi::MENU, "confirm after touchdown leaves Live");
  verifyDisk();

  check(recorder.startLiveRecording(), "enter before queued liftoff");
  recorder.onTelemetryPacket(packet(3, 3), 1);
  bool prompt = false;
  check(!recorder.stopLiveRecording(true, &prompt) && prompt, "queued liftoff must require confirmation");
  const std::string queuedName = recorder.getStatus().activeFilename;
  expected[queuedName] = header + row(3, 3, 1);
  check(recorder.stopLiveRecording(), "confirmed stop drains preceding samples");
  recorder.onTelemetryPacket(packet(4, 4), 1);
  check(recorder.sync(), "drain after leaving");
  prompt = true; host::failSend = true;
  check(!recorder.stopLiveRecording(true, &prompt) && !prompt, "queue failure is not a confirmation request");
  host::failSend = false;
  verifyDisk();
  std::cout << "PASS: real queued exit/landing interleaving, pending liftoff, disabled samples and failure reply\n";
}
static void testScan(Recorder& recorder) {
  recorder.configure(SINGLE, true);
  beginSegment(recorder);
  for (int phase = 0; phase < 3; ++phase) {
    if (phase == 1) {
      flash.failSync = true;
      check(!recorder.sync(), "injected sync failure");
      flash.failSync = false;
      check(recorder.getStatus().state == RecorderState::Fault, "real recorder fault state");
    }
    if (phase == 2) check(recorder.stopLiveRecording(), "close after fault");
    Hmi hmi(recorder);
    hmi.isLogging = false; // Stale Idle snapshot, Fault, then genuinely Idle.
    const auto scans = host::scans;
    Hmi::update(&hmi);
    check(host::scans == scans, "Live must never run the free-space filesystem scan");
  }
  Hmi menu(recorder); menu.state = Hmi::MENU;
  const auto scans = host::scans;
  Hmi::update(&menu);
  check(host::scans == scans + 1, "Menu still updates free space using actual SdFat scan");
  verifyDisk();
  std::cout << "PASS: production HMI scan guard for stale Idle, Fault and Idle; Menu scan still works\n";
}
static void testSegments(Recorder& recorder) {
  for (bool dual : {false, true}) for (bool never : {false, true}) {
    recorder.configure(dual ? DUAL : SINGLE, never);
    for (unsigned segment = 0; segment < 16; ++segment) {
      check(recorder.shareWithMassStorage(), "share before mid-flight entry");
      const auto name = beginSegment(recorder, 3 + segment % 4);
      check(!hostOwned, "first flight sample reclaims storage");
      for (unsigned i = 2; i <= 30; ++i) {
        const uint8_t source = dual ? i % 2 + 1 : 1;
        send(recorder, 4, i, source);
        expected[name] += row(4, i, source);
      }
      send(recorder, 7, 31, 1); expected[name] += row(7, 31, 1);
      if (dual) {
        check(recorder.getStatus().state == RecorderState::Recording, "other participant still flying");
        send(recorder, 7, 32, 2); expected[name] += row(7, 32, 2);
      }
      check(recorder.getStatus().state == (never ? RecorderState::Recording : RecorderState::Idle),
            "touchdown honors stop setting and retains final sample");
      if (never && segment == 0) {
        Hmi hmi(recorder); hmi.backButton.pressed = true; hmi.live();
        check(hmi.liveExitState == Hmi::LiveExitState::Confirm, "active log asks confirmation");
        hmi.backButton.pressed = true; hmi.live();
        check(hmi.state == Hmi::LIVE && recorder.getStatus().state == RecorderState::Recording,
              "B cancels without closing log");
        hmi.backButton.pressed = true; hmi.live();
        flash.failSync = true;
        hmi.okButton.pressed = true; hmi.live();
        check(hmi.state == Hmi::LIVE && hmi.liveExitState == Hmi::LiveExitState::StopFailed,
              "failed sync must retain Live and the open log");
        flash.failSync = false;
        hmi.backButton.pressed = true; hmi.live();
        hmi.backButton.pressed = true; hmi.live();
        hmi.okButton.pressed = true; hmi.live();
        check(hmi.state == Hmi::MENU, "retry closes and leaves Live");
      } else check(recorder.stopLiveRecording(), "stop segment");
      verifyDisk();
    }
  }
  std::cout << "PASS: 64 flight segments, dual/single + Never/landing, USB remounts, cancellation/failure/retry\n";
}
int main(int argc, char** argv) {
  int result = 0;
  try {
    check(argc == 3, "usage: test image all|down|exit|scan");
    std::ifstream input(argv[1], std::ios::binary);
    flash.raw.bytes.assign(std::istreambuf_iterator<char>(input), {});
    check(flash.raw.bytes.size() == 960 * 1024, "GS volume size");
    check(fatfs.begin(&flash), "initial mount");
    check(fatfs.mkdir("/logs"), "seed log directory");
    for (const char* name : {"log_000.csv", "log_001.csv", "log_002.csv", "cats1_legacy.csv"}) {
      expected[name] = std::string("Existing flight ") + name + '\n' + std::string(1800, 'x');
      auto file = fatfs.open((std::string("/logs/") + name).c_str(), FILE_WRITE);
      check(file.write(expected[name].data(), expected[name].size()) == expected[name].size() && file.close(), "seed old log");
    }
    storageAccessMutex = xSemaphoreCreateMutex();
    Recorder recorder("/logs");
    check(recorder.begin(), "start production recorder task");
    const std::string mode = argv[2];
    if (mode == "all" || mode == "down") testDown(recorder);
    if (mode == "all" || mode == "exit") testExit(recorder);
    if (mode == "all" || mode == "scan") testScan(recorder);
    if (mode == "all") testSegments(recorder);
    check(recorder.stopLiveRecording(), "final stop");
    verifyDisk();
    check(fatfs.end() != nullptr, "final unmount");
    host::shutdown(); // Join before recorder/status mutex lifetime ends.
    std::cout << "PASS: all " << expected.size() << " log names and complete contents survived fresh mounts\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    host::shutdown();
    result = 1;
  }
  return result;
}
