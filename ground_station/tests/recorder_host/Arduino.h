// Copyright (C) 2026 Control and Telemetry Systems GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

// Host adapters only: the recorder and SdFat sources are compiled unchanged.
#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

constexpr double PI = 3.141592653589793;
constexpr int LOW = 0;
constexpr int SS = 10;
using TickType_t = uint32_t;
constexpr int pdPASS = 1, pdTRUE = 1;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
inline TickType_t pdMS_TO_TICKS(uint32_t value) { return value; }
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
inline void portENTER_CRITICAL(std::mutex* mutex) { mutex->lock(); }
inline void portEXIT_CRITICAL(std::mutex* mutex) { mutex->unlock(); }
struct HostQueue {
  size_t capacity, itemSize;
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<std::vector<uint8_t>> items;
  bool stopped = false;
};
using QueueHandle_t = HostQueue*;
using SemaphoreHandle_t = std::timed_mutex*;
QueueHandle_t xQueueCreate(size_t capacity, size_t itemSize);
int xQueueSend(QueueHandle_t queue, const void* item, TickType_t timeout);
int xQueueReceive(QueueHandle_t queue, void* item, TickType_t timeout);
SemaphoreHandle_t xSemaphoreCreateMutex();
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout);
void xSemaphoreGive(SemaphoreHandle_t mutex);
void xTaskCreate(void (*task)(void*), const char*, size_t, void* argument, int, void*);
inline void vTaskDelete(void*) {}
TickType_t xTaskGetTickCount();
uint32_t millis();
inline void vTaskDelay(TickType_t) {}
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline int digitalRead(int) { return 1; }
inline int analogRead(int) { return 4000; }
inline void setTime(int, int, int, int, int, int) {}
inline void adjustTime(long) {}

namespace host {
extern std::function<void()> beforeReplyDelivery;
extern std::atomic<uint32_t> clockOffset;
extern unsigned scans;
extern bool failSend;
void shutdown();
}

// Replace only the physical flash transport under the pinned FlashCache.cpp.
// Its 4 KiB erase/rewrite/cache logic is the actual library implementation.
#define ADAFRUIT_SPIFLASHBASE_H_
#define SFLASH_SECTOR_SIZE 4096
using std::min;
class Adafruit_SPIFlashBase {
 public:
  std::vector<uint8_t> bytes;
  uint32_t readBuffer(uint32_t address, uint8_t* buffer, uint32_t size) {
    std::memcpy(buffer, bytes.data() + address, size);
    return size;
  }
  uint32_t writeBuffer(uint32_t address, const uint8_t* buffer, uint32_t size) {
    for (uint32_t i = 0; i < size; ++i) bytes.at(address + i) &= buffer[i];
    return size;
  }
  bool eraseSector(uint32_t sector) {
    std::fill_n(bytes.begin() + sector * 4096, 4096, 0xff);
    return true;
  }
};
