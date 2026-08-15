// WebAssembly host ABI backed by the simulator controller and the production
// Window renderer. JavaScript only forwards input, virtual time, and pixels.
#include "hmi_controller.hpp"
#include "window_hmi_renderer.hpp"
#include "hmi/location_qr.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <sstream>
#include <string>

namespace {
class Clock final : public IClock {
 public:
  uint64_t nowMs() const override { return now; }
  uint8_t hour() const override { return hourValue; }
  uint8_t minute() const override { return minuteValue; }
  uint64_t now = 0;
  uint8_t hourValue = 0;
  uint8_t minuteValue = 0;
};

class Link final : public ITelemetryLink {
 public:
  LinkSnapshot snapshot() const override { return value; }
  void clearUpdates() override { value.telemetry.updated = false; value.info.updated = false; }
  void enterTesting() override { value.telemetry.testingMode = true; value.telemetry.state = 1; }
  void exitTesting() override { value.telemetry.testingMode = false; }
  void triggerEvent(uint8_t event) override { lastEvent = event; }
  void enable() override { value.enabled = true; }
  void disable() override { value.enabled = false; }
  void setLinkPhrase(const std::string& phrase) override { linkPhrase = phrase; }
  void setTestingPhrase(const std::string& phrase) override { testingPhrase = phrase; }

  LinkSnapshot value{};
  uint8_t lastEvent = 0;
  std::string linkPhrase;
  std::string testingPhrase;
};

class Navigation final : public INavigation {
 public:
  NavigationSnapshot snapshot() const override { return value; }
  void startCalibration() override { value.calibrationState = 1; }
  void cancelCalibration() override { value.calibrationState = 2; }
  NavigationSnapshot value{};
};

class Logs final : public ILogStore {
 public:
  std::vector<FlightLogSnapshot> listLogs() const override { return value; }
  FlightStatisticsSnapshot statistics(const FlightLogSnapshot& log, uint8_t link) const override {
    if (cachedName != log.name || cachedCsv != log.csv) {
      cachedName = log.name;
      cachedCsv = log.csv;
      cachedStatistics = analyze(log);
    }
    return link >= 1 && link <= 2 ? cachedStatistics[link - 1U] : FlightStatisticsSnapshot{};
  }
  RecorderSnapshot recorderStatus() const override { return status; }
  void configure(bool dualMode, bool neverStop) override { dual = dualMode; never = neverStop; }

  void record(const TelemetrySample& sample, uint8_t link) override {
    if (link < 1 || link > 2) return;
    const uint8_t bit = static_cast<uint8_t>(1U << (link - 1U));
    if (!armed) {
      if (sample.state <= 2) {
        rearmMask |= bit;
        if (!dual || (completedParticipants != 0U && (rearmMask & completedParticipants) == completedParticipants)) {
          armed = true;
          completedParticipants = 0;
          status.participantMask = 0;
          status.writtenRows = 0;
        }
      }
      return;
    }
    if (sample.state <= 2) return;
    if (failWrite) {
      status.state = "fault";
      status.lastError = "Injected write failure";
      return;
    }
    if (activeIndex < 0) createLog();
    char row[160]{};
    std::snprintf(row, sizeof(row), "%u,%u,%u,%u,%ld,%ld,%ld,%d,%u,%u,%u\n", link, sample.timestampDs,
                  sample.state, sample.errors, static_cast<long>(std::lround(sample.latitude * 10000.0F)),
                  static_cast<long>(std::lround(sample.longitude * 10000.0F)), static_cast<long>(sample.altitudeM),
                  sample.velocityMps, static_cast<unsigned>(std::lround(sample.voltage * 10.0F)),
                  static_cast<unsigned>((sample.pyroContinuity & 1U) != 0U),
                  static_cast<unsigned>((sample.pyroContinuity & 2U) != 0U));
    value[static_cast<size_t>(activeIndex)].csv += row;
    value[static_cast<size_t>(activeIndex)].sizeBytes = value[static_cast<size_t>(activeIndex)].csv.size();
    status.participantMask |= bit;
    status.writtenRows++;
    cachedName.clear();
    if (sample.state == 7) {
      touchdownMask |= bit;
      const bool completeMission = !dual ||
          (status.participantMask != 0U && (touchdownMask & status.participantMask) == status.participantMask);
      if (completeMission && !never) (void)finalize();
    }
  }

  bool finalize() override {
    if (activeIndex < 0) return false;
    if (failFinalize) {
      status.state = "fault";
      status.lastError = "Injected finalize failure";
      return false;
    }
    value[static_cast<size_t>(activeIndex)].active = false;
    completedParticipants = status.participantMask;
    activeIndex = -1;
    armed = false;
    rearmMask = 0;
    touchdownMask = 0;
    status.state = "idle";
    status.activeFilename.clear();
    return true;
  }

  bool remove(const std::string& name) override {
    if (failDelete) return false;
    const auto found = std::find_if(value.begin(), value.end(), [&](const FlightLogSnapshot& log) {
      return log.name == name;
    });
    if (found == value.end() || found->active) return false;
    const auto erasedIndex = static_cast<int32_t>(std::distance(value.begin(), found));
    value.erase(found);
    if (activeIndex > erasedIndex) --activeIndex;
    cachedName.clear();
    return true;
  }

  void load(std::vector<FlightLogSnapshot> logs) {
    value = std::move(logs);
    value.erase(std::remove_if(value.begin(), value.end(), [](const FlightLogSnapshot& log) {
                  return !isCsv(log.name);
                }),
                value.end());
    std::stable_sort(value.begin(), value.end(), [](const FlightLogSnapshot& lhs, const FlightLogSnapshot& rhs) {
      const int32_t lhsNumber = number(lhs.name);
      const int32_t rhsNumber = number(rhs.name);
      if ((lhsNumber >= 0) != (rhsNumber >= 0)) return lhsNumber >= 0;
      if (lhsNumber >= 0 && lhsNumber != rhsNumber) return lhsNumber > rhsNumber;
      return false;
    });
    activeIndex = -1;
    for (size_t index = 0; index < value.size(); ++index) {
      value[index].sizeBytes = value[index].csv.size();
      if (value[index].active && activeIndex < 0) activeIndex = static_cast<int32_t>(index);
    }
    status = {};
    if (activeIndex >= 0) {
      status.state = "recording";
      status.activeFilename = value[static_cast<size_t>(activeIndex)].name;
    }
    cachedName.clear();
  }

  bool failDelete = false;
  bool failFinalize = false;
  bool failWrite = false;
  std::vector<FlightLogSnapshot> value;

 private:
  static constexpr const char* header =
      "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/s],battery[decivolts],pyro1,pyro2\n";
  bool dual = false;
  bool never = false;
  bool armed = true;
  int32_t activeIndex = -1;
  uint8_t touchdownMask = 0;
  uint8_t rearmMask = 0;
  uint8_t completedParticipants = 0;
  RecorderSnapshot status{};
  mutable std::string cachedName;
  mutable std::string cachedCsv;
  mutable std::array<FlightStatisticsSnapshot, 2> cachedStatistics{};

  static int32_t number(const std::string& name) {
    long result = -1;
    char tail = '\0';
    return std::sscanf(name.c_str(), "log_%ld.csv%c", &result, &tail) == 1 ? static_cast<int32_t>(result) : -1;
  }

  static bool isCsv(const std::string& name) {
    if (name.size() <= 4U || name[name.size() - 4U] != '.') return false;
    const auto lower = [](char value) {
      return static_cast<char>(value >= 'A' && value <= 'Z' ? value + ('a' - 'A') : value);
    };
    return lower(name[name.size() - 3U]) == 'c' && lower(name[name.size() - 2U]) == 's' &&
           lower(name[name.size() - 1U]) == 'v';
  }

  void createLog() {
    int32_t highest = -1;
    for (const auto& log : value) highest = std::max(highest, number(log.name));
    char name[30]{};
    std::snprintf(name, sizeof(name), "log_%03ld.csv", static_cast<long>(highest + 1));
    value.insert(value.begin(), FlightLogSnapshot{name, header, std::strlen(header), true});
    activeIndex = 0;
    status = {};
    status.state = "recording";
    status.activeFilename = name;
  }

  static bool parseRow(const std::string& row, std::array<int32_t, 11>& fields) {
    const char* cursor = row.c_str();
    for (size_t index = 0; index < fields.size(); ++index) {
      char* end = nullptr;
      const long value = std::strtol(cursor, &end, 10);
      if (end == cursor || value < INT32_MIN || value > INT32_MAX) return false;
      fields[index] = static_cast<int32_t>(value);
      if (index + 1U < fields.size()) {
        if (*end != ',') return false;
        cursor = end + 1;
      } else if (*end != '\0' && *end != '\r') {
        return false;
      }
    }
    return true;
  }

  static std::array<FlightStatisticsSnapshot, 2> analyze(const FlightLogSnapshot& log) {
    struct Work {
      bool timestamp = false;
      int32_t lastTs = 0;
      bool liftoff = false;
      bool apogee = false;
      bool main = false;
      bool touchdown = false;
      int32_t liftoffTs = 0;
      int32_t apogeeTs = 0;
      int32_t mainTs = 0;
      int32_t touchdownTs = 0;
      int32_t apogeeAlt = 0;
      int32_t mainAlt = 0;
      int32_t lastAlt = 0;
    };
    std::array<FlightStatisticsSnapshot, 2> output{};
    std::array<Work, 2> work{};
    std::istringstream rows(log.csv);
    std::string row;
    bool first = true;
    while (std::getline(rows, row)) {
      if (first) { first = false; continue; }
      std::array<int32_t, 11> fields{};
      if (!parseRow(row, fields) || fields[0] < 1 || fields[0] > 2 || fields[1] < 0 || fields[1] > 65535 ||
          fields[2] < 0 || fields[2] > 7 || fields[4] < -900000 || fields[4] > 900000 ||
          fields[5] < -1800000 || fields[5] > 1800000 || fields[9] < 0 || fields[9] > 1 || fields[10] < 0 || fields[10] > 1) {
        for (auto& stats : output) stats.malformedRows++;
        continue;
      }
      const size_t index = static_cast<size_t>(fields[0] - 1);
      if (work[index].timestamp && fields[1] < work[index].lastTs) {
        for (auto& stats : output) stats.malformedRows++;
        continue;
      }
      for (auto& stats : output) stats.validRows++;
      Work& current = work[index];
      auto& stats = output[index];
      current.timestamp = true;
      current.lastTs = fields[1];
      current.lastAlt = fields[6];
      if (fields[2] > 2) stats.participant = true;
      if (!stats.maxAltitudeValid || fields[6] > stats.maxAltitudeM) {
        stats.maxAltitudeM = fields[6]; stats.maxAltitudeValid = true;
      }
      if (!stats.maxVelocityValid || fields[7] > stats.maxVelocityMps) {
        stats.maxVelocityMps = fields[7]; stats.maxVelocityValid = true;
      }
      if (fields[4] != 0 || fields[5] != 0) {
        stats.lastLatitude = static_cast<float>(fields[4]) / 10000.0F;
        stats.lastLongitude = static_cast<float>(fields[5]) / 10000.0F;
        stats.lastLocationValid = LocationQr::IsValid(stats.lastLatitude, stats.lastLongitude);
      }
      if (fields[2] == 3 && !current.liftoff) {
        current.liftoff = true; current.liftoffTs = fields[1];
      } else if (fields[2] == 5 && !current.apogee) {
        current.apogee = true; current.apogeeTs = fields[1]; current.apogeeAlt = fields[6];
      } else if (fields[2] == 6 && !current.main) {
        current.main = true; current.mainTs = fields[1]; current.mainAlt = fields[6];
      } else if (fields[2] == 7 && !current.touchdown) {
        current.touchdown = true; current.touchdownTs = fields[1];
      }
    }
    for (size_t index = 0; index < output.size(); ++index) {
      const Work& current = work[index];
      auto& stats = output[index];
      if (current.liftoff && current.apogee && current.apogeeTs >= current.liftoffTs) {
        stats.timeToApogeeS = static_cast<float>(current.apogeeTs - current.liftoffTs) / 10.0F;
        stats.timeToApogeeValid = true;
      }
      if (current.liftoff && current.timestamp && current.lastTs >= current.liftoffTs) {
        stats.flightTimeS = static_cast<float>(current.lastTs - current.liftoffTs) / 10.0F;
        stats.flightTimeValid = true;
      }
      if (current.apogee && current.main && current.mainTs > current.apogeeTs) {
        stats.drogueDescentRateMps = static_cast<float>(current.apogeeAlt - current.mainAlt) * 10.0F /
                                     static_cast<float>(current.mainTs - current.apogeeTs);
        stats.drogueRateValid = true;
      }
      if (current.main && current.touchdown && current.lastTs > current.mainTs) {
        stats.mainDescentRateMps = static_cast<float>(current.mainAlt - current.lastAlt) * 10.0F /
                                   static_cast<float>(current.lastTs - current.mainTs);
        stats.mainRateValid = true;
      }
      stats.complete = stats.participant && current.liftoff && current.apogee && current.main && current.touchdown &&
                       stats.malformedRows == 0U;
    }
    return output;
  }
};

class ConfigStore final : public IConfigStore {
 public:
  GsConfigSnapshot& config() override { return value; }
  const GsConfigSnapshot& config() const override { return value; }
  void save() override { ++saveCount; }
  GsConfigSnapshot value;
  uint32_t saveCount = 0;
};

class Device final : public IDeviceStatus {
 public:
  DeviceStatusSnapshot snapshot() const override { return value; }
  bool requestMassStorage() override {
    if (!value.usb || value.usbStorageState != "firmware") return false;
    value.usbStorageState = "host";
    return true;
  }
  void requestFirmwareStorage() override { value.usbStorageState = "firmware"; }
  DeviceStatusSnapshot value;
};

SimulatorDisplay display;
Clock virtualClock;
WindowHmiRenderer renderer(display, virtualClock);
Link link1;
Link link2;
Navigation navigation;
Logs logs;
ConfigStore configStore;
Device device;
HmiController controller(renderer, link1, link2, navigation, logs, configStore, device, virtualClock);
HmiInput input;
std::string snapshot;

bool numberField(const char* json, const char* key, int32_t& output) {
  if (json == nullptr || key == nullptr) return false;
  const std::string needle = std::string("\"") + key + "\"";
  const char* found = std::strstr(json, needle.c_str());
  if (found == nullptr) return false;
  found = std::strchr(found, ':');
  if (found == nullptr) return false;
  char* end = nullptr;
  const long parsed = std::strtol(found + 1, &end, 10);
  if (end == found + 1) return false;
  output = static_cast<int32_t>(parsed);
  return true;
}

bool decimalField(const char* json, const char* key, float& output) {
  if (json == nullptr || key == nullptr) return false;
  const std::string needle = std::string("\"") + key + "\"";
  const char* found = std::strstr(json, needle.c_str());
  if (found == nullptr) return false;
  found = std::strchr(found, ':');
  if (found == nullptr) return false;
  char* end = nullptr;
  output = std::strtof(found + 1, &end);
  return end != found + 1;
}

bool booleanField(const char* json, const char* key, bool& output) {
  if (json == nullptr || key == nullptr) return false;
  const std::string needle = std::string("\"") + key + "\"";
  const char* found = std::strstr(json, needle.c_str());
  if (found == nullptr) return false;
  found = std::strchr(found, ':');
  if (found == nullptr) return false;
  ++found;
  while (*found == ' ' || *found == '\t' || *found == '\r' || *found == '\n') ++found;
  if (std::strncmp(found, "true", 4) == 0) {
    output = true;
    return true;
  }
  if (std::strncmp(found, "false", 5) == 0) {
    output = false;
    return true;
  }
  return false;
}

bool stringField(const char* json, const char* key, std::string& output) {
  if (json == nullptr || key == nullptr) return false;
  const std::string needle = std::string("\"") + key + "\"";
  const char* found = std::strstr(json, needle.c_str());
  if (found == nullptr) return false;
  found = std::strchr(found + needle.size(), ':');
  if (found == nullptr) return false;
  ++found;
  while (*found == ' ' || *found == '\t' || *found == '\r' || *found == '\n') ++found;
  if (*found != '"') return false;
  ++found;

  output.clear();
  while (*found != '\0' && *found != '"') {
    if (*found != '\\') {
      output += *found++;
      continue;
    }
    ++found;
    if (*found == '\0') return false;
    switch (*found) {
      case 'n': output += '\n'; break;
      case 'r': output += '\r'; break;
      case 't': output += '\t'; break;
      case '"': output += '"'; break;
      case '\\': output += '\\'; break;
      default: output += *found; break;
    }
    ++found;
  }
  return *found == '"';
}

void updateLink(Link& link, const char* json) {
  int32_t integer = 0;
  bool boolean = false;
  if (numberField(json, "state", integer)) link.value.telemetry.state = static_cast<uint8_t>(integer);
  if (numberField(json, "errors", integer)) link.value.telemetry.errors = static_cast<uint8_t>(integer);
  if (numberField(json, "altitudeM", integer)) link.value.telemetry.altitudeM = integer;
  if (numberField(json, "velocityMps", integer)) link.value.telemetry.velocityMps = static_cast<int16_t>(integer);
  if (numberField(json, "timestampDs", integer)) link.value.telemetry.timestampDs = static_cast<uint16_t>(integer);
  if (numberField(json, "pyroContinuity", integer)) link.value.telemetry.pyroContinuity = static_cast<uint8_t>(integer);
  if (numberField(json, "linkQuality", integer)) link.value.info.linkQuality = static_cast<uint8_t>(integer);
  if (numberField(json, "rssi", integer)) link.value.info.rssi = static_cast<int8_t>(integer);
  if (numberField(json, "snr", integer)) link.value.info.snr = static_cast<int8_t>(integer);
  if (decimalField(json, "latitude", link.value.telemetry.latitude)) {}
  if (decimalField(json, "longitude", link.value.telemetry.longitude)) {}
  if (decimalField(json, "voltage", link.value.telemetry.voltage)) {}
  if (booleanField(json, "testingMode", boolean)) link.value.telemetry.testingMode = boolean;
  if (booleanField(json, "connected", boolean)) link.value.connected = boolean;
  if (booleanField(json, "updated", boolean)) {
    link.value.telemetry.updated = boolean;
    link.value.info.updated = boolean;
  } else {
    link.value.telemetry.updated = true;
    link.value.info.updated = true;
  }
}

std::string quote(const std::string& value) {
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '\\' || character == '"') escaped += '\\';
    escaped += character;
  }
  escaped += '"';
  return escaped;
}

void rebuildSnapshot() {
  const HmiSnapshot state = controller.snapshot();
  snapshot = "{\"activeScreen\":" + quote(state.screen) +
             ",\"liveView\":" + quote(state.liveView) +
             ",\"testingState\":" + quote(state.testingState) +
             ",\"testingSelection\":" + std::to_string(state.testingSelection) +
             ",\"keyboardSelection\":" + std::to_string(state.keyboardSelection) +
             ",\"keyboardUppercase\":" + std::string(state.keyboardUppercase ? "true" : "false") +
             ",\"calibrationState\":" + quote(state.calibrationState) +
             ",\"settingsState\":" + quote(state.settingsState) +
             ",\"dataStatistics\":" + std::string(state.dataStatistics ? "true" : "false") +
             ",\"currentDataSubview\":" + quote(state.currentDataSubview) +
             ",\"dataSelection\":" + std::to_string(state.dataSelection) +
             ",\"logCount\":" + std::to_string(state.logs.size()) +
             ",\"logScrollOffset\":" + std::to_string(state.logScrollOffset) +
             ",\"selectedLogHealth\":" + quote(state.selectedLogHealth) +
             ",\"qrView\":" + quote(state.qrView) +
             ",\"qrUrl\":" + quote(state.qrUrl) +
             ",\"inputState\":" + quote(state.inputState) +
             ",\"menuSelection\":" + std::to_string(state.menuSelection) +
             ",\"settingsPage\":" + std::to_string(state.settingsPage) +
             ",\"settingsSelection\":" + std::to_string(state.settingsSelection) +
             ",\"recorderState\":" + quote(state.recorder.state) +
             ",\"activeFilename\":" + quote(state.recorder.activeFilename) +
             ",\"recordedRowCount\":" + std::to_string(state.recorder.writtenRows) +
             ",\"droppedRowCount\":" + std::to_string(state.recorder.droppedRows) +
             ",\"usbStorageState\":" + quote(state.device.usbStorageState) +
             ",\"usbStorageMessage\":" + quote(state.usbStorageMessage) +
             ",\"selectedRecoveryLink\":" + std::to_string(state.selectedRecoveryLink) +
             ",\"recoverySolution\":{\"valid\":" + std::string(state.recoverySolution.valid ? "true" : "false") +
             ",\"latitude\":" + std::to_string(state.recoverySolution.latitude) +
             ",\"longitude\":" + std::to_string(state.recoverySolution.longitude) +
             ",\"distanceM\":" + std::to_string(state.recoverySolution.distanceM) +
             ",\"azimuthRad\":" + std::to_string(state.recoverySolution.azimuthRad) + "}" +
             ",\"virtualTimeMs\":" + std::to_string(state.virtualTimeMs) +
             ",\"startupElapsedMs\":" + std::to_string(state.startupElapsedMs) +
             ",\"startupPhase\":" + quote(state.startupPhase) +
             ",\"configuration\":{\"dualReceiver\":" + std::string(state.configuration.dualReceiver ? "true" : "false") +
             ",\"imperialUnits\":" + std::string(state.configuration.imperialUnits ? "true" : "false") +
             ",\"neverStopLogging\":" + std::string(state.configuration.neverStopLogging ? "true" : "false") +
             ",\"startupAnimation\":" + std::string(state.configuration.startupAnimation ? "true" : "false") +
             ",\"timeZoneOffset\":" + std::to_string(state.configuration.timeZoneOffset) +
             ",\"linkPhrase1\":" + quote(state.configuration.linkPhrase1) +
             ",\"linkPhrase2\":" + quote(state.configuration.linkPhrase2) +
             ",\"testingPhrase\":" + quote(state.configuration.testingPhrase) + "}" +
             ",\"framebufferRevision\":" + std::to_string(state.framebufferRevision) +
             ",\"actions\":[";
  for (size_t index = 0; index < state.actions.size(); ++index) {
    if (index != 0) snapshot += ',';
    snapshot += "{\"type\":" + quote(state.actions[index].type) +
                ",\"link\":" + std::to_string(state.actions[index].link) +
                ",\"value\":" + std::to_string(state.actions[index].value) + "}";
  }
  snapshot += "]}";
}

void step() { controller.step(input, virtualClock.now); }

void resetState(bool skipStartup, bool preserveConfiguration = false) {
  const GsConfigSnapshot savedConfiguration = configStore.value;
  virtualClock.now = 0;
  virtualClock.hourValue = 0;
  virtualClock.minuteValue = 0;
  input = {};
  link1 = Link{};
  link2 = Link{};
  navigation = Navigation{};
  logs = Logs{};
  configStore = ConfigStore{};
  if (preserveConfiguration) configStore.value = savedConfiguration;
  device = Device{};
  controller.start();
  if (skipStartup) {
    virtualClock.now = StartupIntro::kDurationMs;
    step();
  }
  rebuildSnapshot();
}
}  // namespace

extern "C" {
void gs_reset() { resetState(true); }
void gs_restart() { resetState(false, true); }
void gs_advance(uint32_t milliseconds);
void gs_press(const char* button) {
  if (button == nullptr) return;
  const std::string name(button);
  const std::array<std::string, 7> names = {"up", "down", "left", "right", "center", "ok", "back"};
  for (size_t index = 0; index < names.size(); ++index) {
    if (name == names[index] || (name == "a" && index == 5) || (name == "b" && index == 6)) {
      input.held[index] = true;
      input.pressed[index] = true;
    }
  }
  step();
  input.pressed = {};
  rebuildSnapshot();
}
void gs_release(const char* button) {
  if (button == nullptr) return;
  const std::string name(button);
  const std::array<std::string, 7> names = {"up", "down", "left", "right", "center", "ok", "back"};
  for (size_t index = 0; index < names.size(); ++index) if (name == names[index]) input.held[index] = false;
  step();
  rebuildSnapshot();
}
void gs_hold(const char* button, uint32_t milliseconds) {
  gs_press(button);
  gs_advance(milliseconds);
  gs_release(button);
}
void gs_advance(uint32_t milliseconds) {
  const uint64_t target = virtualClock.now + milliseconds;
  while (virtualClock.now < target) {
    virtualClock.now = std::min<uint64_t>(target, virtualClock.now + 20);
    step();
    input.pressed = {};
  }
  rebuildSnapshot();
}
void gs_set_link_json(const char* link, const char* json) {
  if (link != nullptr && (std::strcmp(link, "2") == 0 || std::strcmp(link, "link2") == 0)) updateLink(link2, json);
  else updateLink(link1, json);
  step();
  rebuildSnapshot();
}
void gs_set_navigation_json(const char* json) {
  int32_t integer = 0;
  if (numberField(json, "calibrationState", integer)) navigation.value.calibrationState = static_cast<uint8_t>(integer);
  if (numberField(json, "calibrationPercentage", integer)) navigation.value.calibrationPercentage = static_cast<float>(integer);
  (void)decimalField(json, "distanceM", navigation.value.distanceM);
  (void)decimalField(json, "northRad", navigation.value.northRad);
  (void)decimalField(json, "homeLatitude", navigation.value.homeLatitude);
  (void)decimalField(json, "homeLongitude", navigation.value.homeLongitude);
  (void)decimalField(json, "rocketLatitude", navigation.value.rocketLatitude);
  (void)decimalField(json, "rocketLongitude", navigation.value.rocketLongitude);
  (void)decimalField(json, "azimuthRad", navigation.value.azimuthRad);
  step();
  rebuildSnapshot();
}
void gs_set_sensor_json(const char* json) { gs_set_navigation_json(json); }
void gs_set_device_status_json(const char* json) {
  int32_t integer = 0;
  bool boolean = false;
  if (numberField(json, "hour", integer)) device.value.hour = static_cast<uint8_t>(integer);
  virtualClock.hourValue = device.value.hour;
  if (numberField(json, "minute", integer)) device.value.minute = static_cast<uint8_t>(integer);
  virtualClock.minuteValue = device.value.minute;
  if (numberField(json, "freeStoragePercent", integer)) device.value.freeStoragePercent = static_cast<uint32_t>(integer);
  if (booleanField(json, "usb", boolean)) device.value.usb = boolean;
  if (!device.value.usb && device.value.usbStorageState == "host") device.value.usbStorageState = "firmware";
  (void)stringField(json, "usbStorageState", device.value.usbStorageState);
  if (booleanField(json, "gnss", boolean)) device.value.gnss = boolean;
  if (booleanField(json, "recorderWriteFailure", boolean)) logs.failWrite = boolean;
  if (booleanField(json, "deleteFailure", boolean)) logs.failDelete = boolean;
  if (booleanField(json, "finalizeFailure", boolean)) logs.failFinalize = boolean;
  (void)decimalField(json, "batteryVoltage", device.value.batteryVoltage);
  rebuildSnapshot();
}
void gs_set_configuration_json(const char* json) {
  int32_t integer = 0;
  bool boolean = false;
  if (numberField(json, "timeZoneOffset", integer)) configStore.value.timeZoneOffset = static_cast<int16_t>(integer);
  if (booleanField(json, "neverStopLogging", boolean)) configStore.value.neverStopLogging = boolean;
  if (booleanField(json, "startupAnimation", boolean)) configStore.value.startupAnimation = boolean;
  if (booleanField(json, "dualReceiver", boolean)) configStore.value.dualReceiver = boolean;
  if (booleanField(json, "imperialUnits", boolean)) configStore.value.imperialUnits = boolean;
  rebuildSnapshot();
}
void gs_set_logs_json(const char* json) {
  std::vector<FlightLogSnapshot> fixtures;
  const char* cursor = json;
  while (cursor != nullptr) {
    cursor = std::strstr(cursor, "\"name\"");
    if (cursor == nullptr) break;
    std::string name;
    std::string csv;
    if (stringField(cursor, "name", name) && stringField(cursor, "csv", csv)) {
      bool active = false;
      (void)booleanField(cursor, "active", active);
      fixtures.push_back({name, csv, csv.size(), active});
    }
    cursor += 6;
  }
  logs.load(std::move(fixtures));
  rebuildSnapshot();
}
void gs_load_replay_json(const char*) { rebuildSnapshot(); }
const char* gs_snapshot_json() { rebuildSnapshot(); return snapshot.c_str(); }
const uint8_t* gs_framebuffer() { return display.framebuffer(); }
uint32_t gs_framebuffer_size() { return static_cast<uint32_t>(display.framebufferSize()); }
uint32_t gs_framebuffer_revision() { return display.revision(); }
}
