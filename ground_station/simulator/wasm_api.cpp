// WebAssembly host ABI backed by the simulator controller and the production
// Window renderer. JavaScript only forwards input, virtual time, and pixels.
#include "hmi_controller.hpp"
#include "window_hmi_renderer.hpp"

#include <algorithm>
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
    FlightStatisticsSnapshot result;
    std::istringstream rows(log.csv);
    std::string row;
    while (std::getline(rows, row)) {
      std::istringstream fields(row);
      std::string field;
      std::array<std::string, 6> values{};
      size_t count = 0;
      while (count < values.size() && std::getline(fields, field, ',')) values[count++] = field;
      if (count < values.size() || values[0] == "link" || values[0].empty()) continue;
      try {
        if (std::stoi(values[0]) != link) continue;
        const float latitude = static_cast<float>(std::stoi(values[4])) / 10000.0F;
        const float longitude = static_cast<float>(std::stoi(values[5])) / 10000.0F;
        if (latitude != 0.0F && longitude != 0.0F && latitude >= -90.0F && latitude <= 90.0F &&
            longitude >= -180.0F && longitude <= 180.0F) {
          result.lastLatitude = latitude;
          result.lastLongitude = longitude;
        }
      } catch (const std::exception&) {
        continue;
      }
    }
    return result;
  }
  void record(const TelemetrySample&, uint8_t) override {}
  std::vector<FlightLogSnapshot> value;
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
  if (numberField(json, "linkQuality", integer)) link.value.info.linkQuality = static_cast<uint8_t>(integer);
  if (numberField(json, "rssi", integer)) link.value.info.rssi = static_cast<int8_t>(integer);
  if (numberField(json, "snr", integer)) link.value.info.snr = static_cast<int8_t>(integer);
  if (decimalField(json, "latitude", link.value.telemetry.latitude)) {}
  if (decimalField(json, "longitude", link.value.telemetry.longitude)) {}
  if (decimalField(json, "voltage", link.value.telemetry.voltage)) {}
  if (booleanField(json, "testingMode", boolean)) link.value.telemetry.testingMode = boolean;
  if (booleanField(json, "connected", boolean)) link.value.connected = boolean;
  link.value.telemetry.updated = true;
  link.value.info.updated = true;
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
             ",\"qrView\":" + quote(state.qrView) +
             ",\"qrUrl\":" + quote(state.qrUrl) +
             ",\"inputState\":" + quote(state.inputState) +
             ",\"menuSelection\":" + std::to_string(state.menuSelection) +
             ",\"settingsPage\":" + std::to_string(state.settingsPage) +
             ",\"settingsSelection\":" + std::to_string(state.settingsSelection) +
             ",\"virtualTimeMs\":" + std::to_string(state.virtualTimeMs) +
             ",\"configuration\":{\"dualReceiver\":" + std::string(state.configuration.dualReceiver ? "true" : "false") +
             ",\"imperialUnits\":" + std::string(state.configuration.imperialUnits ? "true" : "false") +
             ",\"neverStopLogging\":" + std::string(state.configuration.neverStopLogging ? "true" : "false") +
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

void resetState() {
  virtualClock.now = 0;
  virtualClock.hourValue = 0;
  virtualClock.minuteValue = 0;
  input = {};
  link1 = Link{};
  link2 = Link{};
  navigation = Navigation{};
  logs = Logs{};
  configStore = ConfigStore{};
  device = Device{};
  controller.start();
  virtualClock.now = 2000;
  step();
  rebuildSnapshot();
}
}  // namespace

extern "C" {
void gs_reset() { resetState(); }
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
  if (booleanField(json, "gnss", boolean)) device.value.gnss = boolean;
  (void)decimalField(json, "batteryVoltage", device.value.batteryVoltage);
  rebuildSnapshot();
}
void gs_set_configuration_json(const char* json) {
  int32_t integer = 0;
  bool boolean = false;
  if (numberField(json, "timeZoneOffset", integer)) configStore.value.timeZoneOffset = static_cast<int16_t>(integer);
  if (booleanField(json, "neverStopLogging", boolean)) configStore.value.neverStopLogging = boolean;
  if (booleanField(json, "dualReceiver", boolean)) configStore.value.dualReceiver = boolean;
  if (booleanField(json, "imperialUnits", boolean)) configStore.value.imperialUnits = boolean;
  rebuildSnapshot();
}
void gs_set_logs_json(const char* json) {
  logs.value.clear();
  const char* cursor = json;
  while (cursor != nullptr) {
    cursor = std::strstr(cursor, "\"name\"");
    if (cursor == nullptr) break;
    std::string name;
    std::string csv;
    if (stringField(cursor, "name", name) && stringField(cursor, "csv", csv)) {
      logs.value.push_back({name, csv});
    }
    cursor += 6;
  }
  rebuildSnapshot();
}
void gs_load_replay_json(const char*) { rebuildSnapshot(); }
const char* gs_snapshot_json() { rebuildSnapshot(); return snapshot.c_str(); }
const uint8_t* gs_framebuffer() { return display.framebuffer(); }
uint32_t gs_framebuffer_size() { return static_cast<uint32_t>(display.framebufferSize()); }
uint32_t gs_framebuffer_revision() { return display.revision(); }
}
