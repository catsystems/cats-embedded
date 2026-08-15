/// Copyright (C) 2024, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "console.hpp"
#include "utils.hpp"

template <typename T>
struct StatValue {
  T value{};
  bool valid{false};
};

class FlightStatistics {
 public:
  int32_t getMaxAltitude() const { return maxAltitude.value; }
  float getTimeToApogee() const { return timeToApogee.value; }
  int32_t getMaxVelocity() const { return maxVelocity.value; }
  float getDrogueDescentRate() const { return drogueRate.value; }
  float getMainDescentRate() const { return mainRate.value; }
  float getLastLatitude() const { return lastLatitude.value; }
  float getLastLongitude() const { return lastLongitude.value; }
  float getFlightTime() const { return flightTime.value; }

  bool hasMaxAltitude() const { return maxAltitude.valid; }
  bool hasTimeToApogee() const { return timeToApogee.valid; }
  bool hasMaxVelocity() const { return maxVelocity.valid; }
  bool hasDrogueDescentRate() const { return drogueRate.valid; }
  bool hasMainDescentRate() const { return mainRate.valid; }
  bool hasLastLocation() const { return lastLatitude.valid && lastLongitude.valid; }
  bool hasFlightTime() const { return flightTime.valid; }
  bool isParticipant() const { return participant; }

 private:
  friend class FlightLogAnalysis;
  StatValue<int32_t> maxAltitude{};
  StatValue<float> timeToApogee{};
  StatValue<int32_t> maxVelocity{};
  StatValue<float> drogueRate{};
  StatValue<float> mainRate{};
  StatValue<float> lastLatitude{};
  StatValue<float> lastLongitude{};
  StatValue<float> flightTime{};
  bool participant{false};
};

class FlightLogAnalysis {
 public:
  enum Phase : uint8_t { Liftoff = 1U, Apogee = 2U, Main = 4U, Touchdown = 8U };

  bool parse(const char* directory, const char* name) {
    *this = FlightLogAnalysis{};
    char path[96]{};
    snprintf(path, sizeof(path), "%s/%s", directory, name);
    auto file = fatfs.open(path, FILE_READ);
    if (!file) {
      return false;
    }
    char line[160]{};
    bool header = true;
    while (file.available()) {
      const size_t length = file.readBytesUntil('\n', line, sizeof(line) - 1U);
      line[length] = '\0';
      if (header) {
        header = false;
        continue;
      }
      if (length == 0U || length == sizeof(line) - 1U) {
        malformedRows++;
        while (file.available() && file.read() != '\n') {
        }
        continue;
      }
      int32_t fields[11]{};
      if (!parseFields(line, fields)) {
        malformedRows++;
        continue;
      }
      const int32_t link = fields[0];
      const int32_t timestamp = fields[1];
      const int32_t state = fields[2];
      const int32_t latitude = fields[4];
      const int32_t longitude = fields[5];
      if (link < 1 || link > 2 || timestamp < 0 || timestamp > 65535 || state < 0 || state > 7 || latitude < -900000 ||
          latitude > 900000 || longitude < -1800000 || longitude > 1800000 ||
          fields[7] < std::numeric_limits<int16_t>::min() || fields[7] > std::numeric_limits<int16_t>::max() ||
          fields[9] < 0 || fields[9] > 1 || fields[10] < 0 || fields[10] > 1) {
        malformedRows++;
        continue;
      }
      Working& work = working[link - 1];
      if (work.hasTimestamp && timestamp < work.lastTimestamp) {
        malformedRows++;
        continue;
      }
      work.hasTimestamp = true;
      work.lastTimestamp = timestamp;
      processRow(static_cast<uint8_t>(link - 1), timestamp, state, latitude, longitude, fields[6], fields[7]);
      validRows++;
    }
    file.close();
    for (uint8_t index = 0; index < 2; ++index) {
      finish(index);
    }
    complete = malformedRows == 0U;
    bool anyParticipant = false;
    // Both summaries and their corresponding phase masks are updated by index.
    // NOLINTNEXTLINE(modernize-loop-convert)
    for (uint8_t index = 0; index < 2; ++index) {
      if (summaries[index].participant) {
        anyParticipant = true;
        constexpr auto kAllPhases =
            static_cast<uint8_t>(static_cast<unsigned>(Liftoff) | static_cast<unsigned>(Apogee) |
                                 static_cast<unsigned>(Main) | static_cast<unsigned>(Touchdown));
        complete = complete && phasePresence[index] == kAllPhases;
      }
    }
    complete = complete && anyParticipant;
    return true;
  }

  // This is the compact result object populated by parse().
  // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
  FlightStatistics summaries[2]{};
  size_t validRows{0};
  size_t malformedRows{0};
  uint8_t phasePresence[2]{};
  bool complete{false};
  // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

 private:
  struct Working {
    bool hasTimestamp{false};
    int32_t lastTimestamp{0};
    bool hasLiftoff{false};
    bool hasApogee{false};
    bool hasMain{false};
    bool hasTouchdown{false};
    int32_t liftoffTimestamp{0};
    int32_t apogeeTimestamp{0};
    int32_t mainTimestamp{0};
    int32_t touchdownTimestamp{0};
    int32_t apogeeAltitude{0};
    int32_t mainAltitude{0};
    int32_t lastAltitude{0};
  };
  Working working[2]{};

  static bool parseFields(const char* line, int32_t fields[11]) {
    const char* cursor = line;
    for (uint8_t index = 0; index < 11; ++index) {
      errno = 0;
      // strtol writes a mutable pointer even though the parsed input is not modified.
      char* parsedEnd = nullptr;  // NOLINT(misc-const-correctness)
      // strtol's result type is fixed by the C library API.
      // NOLINTNEXTLINE(google-runtime-int)
      const long value = strtol(cursor, &parsedEnd, 10);
      const char* end = parsedEnd;
      if (errno == ERANGE || end == cursor || value < std::numeric_limits<int32_t>::min() ||
          value > std::numeric_limits<int32_t>::max()) {
        return false;
      }
      fields[index] = static_cast<int32_t>(value);
      if (index < 10) {
        if (*end != ',') {
          return false;
        }
        cursor = end + 1;
      } else {
        while (*end == '\r' || *end == ' ') {
          ++end;
        }
        if (*end != '\0') {
          return false;
        }
      }
    }
    return true;
  }

  void processRow(uint8_t index, int32_t timestamp, int32_t state, int32_t latitude, int32_t longitude,
                  int32_t altitude, int32_t velocity) {
    Working& work = working[index];
    FlightStatistics& stats = summaries[index];
    if (state > 2) {
      stats.participant = true;
    }
    if (!stats.maxAltitude.valid || altitude > stats.maxAltitude.value) {
      stats.maxAltitude = {altitude, true};
    }
    if (!stats.maxVelocity.valid || velocity > stats.maxVelocity.value) {
      stats.maxVelocity = {velocity, true};
    }
    if ((latitude != 0 || longitude != 0) && latitude >= -900000 && latitude <= 900000 && longitude >= -1800000 &&
        longitude <= 1800000) {
      stats.lastLatitude = {static_cast<float>(latitude) / 10000.0F, true};
      stats.lastLongitude = {static_cast<float>(longitude) / 10000.0F, true};
    }
    work.lastAltitude = altitude;
    if (state == 3 && !work.hasLiftoff) {
      work.hasLiftoff = true;
      work.liftoffTimestamp = timestamp;
      phasePresence[index] |= Liftoff;
    } else if (state == 5 && !work.hasApogee) {
      work.hasApogee = true;
      work.apogeeTimestamp = timestamp;
      work.apogeeAltitude = altitude;
      phasePresence[index] |= Apogee;
    } else if (state == 6 && !work.hasMain) {
      work.hasMain = true;
      work.mainTimestamp = timestamp;
      work.mainAltitude = altitude;
      phasePresence[index] |= Main;
    } else if (state == 7 && !work.hasTouchdown) {
      work.hasTouchdown = true;
      work.touchdownTimestamp = timestamp;
      phasePresence[index] |= Touchdown;
    }
  }

  void finish(uint8_t index) {
    const Working& work = working[index];
    FlightStatistics& stats = summaries[index];
    if (work.hasLiftoff && work.hasApogee && work.apogeeTimestamp >= work.liftoffTimestamp) {
      stats.timeToApogee = {static_cast<float>(work.apogeeTimestamp - work.liftoffTimestamp) / 10.0F, true};
    }
    if (work.hasLiftoff && work.hasTimestamp && work.lastTimestamp >= work.liftoffTimestamp) {
      stats.flightTime = {static_cast<float>(work.lastTimestamp - work.liftoffTimestamp) / 10.0F, true};
    }
    if (work.hasApogee && work.hasMain && work.mainTimestamp > work.apogeeTimestamp) {
      stats.drogueRate = {static_cast<float>(work.apogeeAltitude - work.mainAltitude) /
                              (static_cast<float>(work.mainTimestamp - work.apogeeTimestamp) / 10.0F),
                          true};
    }
    if (work.hasMain && work.hasTouchdown && work.lastTimestamp > work.mainTimestamp) {
      stats.mainRate = {static_cast<float>(work.mainAltitude - work.lastAltitude) /
                            (static_cast<float>(work.lastTimestamp - work.mainTimestamp) / 10.0F),
                        true};
    }
  }
};
