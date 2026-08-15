/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hmi/startup_intro.hpp"

#include "clock.hpp"

// Simulator state uses portable values rather than packed radio or Arduino
// types. The WebAssembly adapters inject them directly.
struct TelemetrySample {
  uint8_t state = 2;
  uint8_t errors = 0;
  int32_t altitudeM = 0;
  int16_t velocityMps = 0;
  float latitude = 0.0F;
  float longitude = 0.0F;
  float voltage = 0.0F;
  uint8_t pyroContinuity = 0;
  bool testingMode = false;
  uint16_t timestampDs = 0;
  uint64_t lastUpdateMs = 0;
  bool updated = false;
};

struct LinkInfoSnapshot {
  uint8_t linkQuality = 0;
  int8_t rssi = 0;
  int8_t snr = 0;
  uint64_t lastUpdateMs = 0;
  bool updated = false;
};

struct LinkSnapshot {
  TelemetrySample telemetry;
  LinkInfoSnapshot info;
  bool enabled = true;
  bool connected = false;
};

struct NavigationSnapshot {
  float homeLatitude = 0.0F;
  float homeLongitude = 0.0F;
  float rocketLatitude = 0.0F;
  float rocketLongitude = 0.0F;
  float northRad = 0.0F;
  float azimuthRad = 0.0F;
  float distanceM = 0.0F;
  float elevationRad = 0.0F;
  float ax = 0.0F;
  float ay = 0.0F;
  float az = 1.0F;
  float gx = 0.0F;
  float gy = 0.0F;
  float gz = 0.0F;
  float mx = 0.0F;
  float my = 0.0F;
  float mz = 0.0F;
  float calibrationPercentage = 0.0F;
  uint8_t calibrationState = 0;
  bool updated = false;
};

struct DeviceStatusSnapshot {
  float batteryVoltage = 0.0F;
  bool usb = false;
  uint32_t freeStoragePercent = 100;
  bool gnss = false;
  bool clockValid = false;
  uint8_t hour = 0;
  uint8_t minute = 0;
  bool logging = false;
  bool recorderFault = false;
  std::string usbStorageState = "firmware";
};

struct GsConfigSnapshot {
  int16_t timeZoneOffset = 0;
  bool neverStopLogging = false;
  bool startupAnimation = true;
  bool dualReceiver = false;
  std::string linkPhrase1;
  std::string linkPhrase2;
  std::string testingPhrase;
  bool imperialUnits = false;
};

struct FlightLogSnapshot {
  std::string name;
  std::string csv;
  size_t sizeBytes = 0;
  bool active = false;
};

struct FlightStatisticsSnapshot {
  int32_t maxAltitudeM = 0;
  float timeToApogeeS = 0.0F;
  int32_t maxVelocityMps = 0;
  float drogueDescentRateMps = 0.0F;
  float mainDescentRateMps = 0.0F;
  float lastLatitude = 0.0F;
  float lastLongitude = 0.0F;
  float flightTimeS = 0.0F;
  bool maxAltitudeValid = false;
  bool timeToApogeeValid = false;
  bool maxVelocityValid = false;
  bool drogueRateValid = false;
  bool mainRateValid = false;
  bool lastLocationValid = false;
  bool flightTimeValid = false;
  bool participant = false;
  bool complete = false;
  size_t validRows = 0;
  size_t malformedRows = 0;
};

struct RecorderSnapshot {
  std::string state = "idle";
  std::string activeFilename;
  uint8_t participantMask = 0;
  uint32_t writtenRows = 0;
  uint32_t droppedRows = 0;
  std::string lastError;
};

struct RecoverySolutionSnapshot {
  bool valid = false;
  float latitude = 0.0F;
  float longitude = 0.0F;
  float distanceM = 0.0F;
  float azimuthRad = 0.0F;
};

enum class HmiButton : uint8_t { Up, Down, Left, Right, Center, Ok, Back };

struct HmiInput {
  std::array<bool, 7> held{};
  std::array<bool, 7> pressed{};
};

struct PlatformAction {
  std::string type;
  uint8_t link = 0;
  int32_t value = 0;
  std::string text;
};

class ITelemetryLink {
 public:
  virtual ~ITelemetryLink() = default;
  [[nodiscard]] virtual LinkSnapshot snapshot() const = 0;
  virtual void clearUpdates() = 0;
  virtual void enterTesting() = 0;
  virtual void exitTesting() = 0;
  virtual void triggerEvent(uint8_t event) = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual void setLinkPhrase(const std::string& phrase) = 0;
  virtual void setTestingPhrase(const std::string& phrase) = 0;
};

class INavigation {
 public:
  virtual ~INavigation() = default;
  [[nodiscard]] virtual NavigationSnapshot snapshot() const = 0;
  virtual void startCalibration() = 0;
  virtual void cancelCalibration() = 0;
};

class ILogStore {
 public:
  virtual ~ILogStore() = default;
  [[nodiscard]] virtual std::vector<FlightLogSnapshot> listLogs() const = 0;
  [[nodiscard]] virtual FlightStatisticsSnapshot statistics(const FlightLogSnapshot& log, uint8_t link) const = 0;
  [[nodiscard]] virtual RecorderSnapshot recorderStatus() const = 0;
  virtual void configure(bool dual, bool neverStop) = 0;
  virtual void record(const TelemetrySample& sample, uint8_t link) = 0;
  virtual bool finalize() = 0;
  virtual bool remove(const std::string& name) = 0;
};

class IConfigStore {
 public:
  virtual ~IConfigStore() = default;
  [[nodiscard]] virtual GsConfigSnapshot& config() = 0;
  [[nodiscard]] virtual const GsConfigSnapshot& config() const = 0;
  virtual void save() = 0;
};

class IDeviceStatus {
 public:
  virtual ~IDeviceStatus() = default;
  [[nodiscard]] virtual DeviceStatusSnapshot snapshot() const = 0;
  virtual bool requestMassStorage() = 0;
  virtual void requestFirmwareStorage() = 0;
};

struct HmiSnapshot {
  std::string screen = "logo";
  std::string testingState = "disclaimer";
  std::string calibrationState = "idle";
  std::string settingsState = "list";
  std::string inputState = "idle";
  std::string liveView = "gnss";
  int16_t menuSelection = 0;
  int16_t testingSelection = 0;
  int16_t keyboardSelection = 0;
  bool keyboardUppercase = false;
  int16_t settingsPage = 0;
  int16_t settingsSelection = -1;
  int16_t dataSelection = 0;
  bool dataStatistics = false;
  std::string currentDataSubview = "list";
  size_t logScrollOffset = 0;
  std::string selectedLogHealth = "none";
  std::string dataMessageTitle;
  std::string dataMessageText;
  std::string usbStorageMessage;
  std::string qrView = "none";
  std::string qrUrl;
  uint64_t virtualTimeMs = 0;
  uint32_t startupElapsedMs = 0;
  std::string startupPhase = "rocket_flight";
  GsConfigSnapshot configuration;
  std::array<LinkSnapshot, 2> links{};
  NavigationSnapshot navigation;
  DeviceStatusSnapshot device;
  std::vector<FlightLogSnapshot> logs;
  std::array<FlightStatisticsSnapshot, 2> flightStatistics{};
  std::array<FlightStatisticsSnapshot, 2> recoveryLocations{};
  RecorderSnapshot recorder;
  int8_t selectedRecoveryLink = -1;
  RecoverySolutionSnapshot recoverySolution;
  std::vector<PlatformAction> actions;
  uint32_t framebufferRevision = 0;
};

class IHmiRenderer {
 public:
  virtual ~IHmiRenderer() = default;
  virtual void begin() = 0;
  virtual void render(const HmiSnapshot& state) = 0;
  [[nodiscard]] virtual uint32_t revision() const = 0;
};

class HmiController {
 public:
  HmiController(IHmiRenderer& renderer, ITelemetryLink& link1, ITelemetryLink& link2, INavigation& navigation,
                ILogStore& logs, IConfigStore& config, IDeviceStatus& device, IClock& clock);

  void start();
  void step(const HmiInput& input, uint64_t nowMs);
  [[nodiscard]] HmiSnapshot snapshot() const;
  [[nodiscard]] const std::vector<PlatformAction>& actions() const { return actions_; }
  void clearActions() { actions_.clear(); }

 private:
  enum class Screen : uint8_t { Logo, Menu, Live, Recovery, Testing, Data, Sensors, Settings, Bootloader, UsbStorage };
  enum class TestingState : uint8_t { Disclaimer, CanStart, CannotStart, Waiting, Failed, Started, ConfirmEvent };
  enum class CalibrationState : uint8_t { Idle, Prepare, Calibrating, Concluded };
  enum class DataSubview : uint8_t { List, Details, Options, ConfirmFinalize, ConfirmDelete, Message };

  static constexpr uint64_t kTestingTimeoutMs = 10000;
  static constexpr uint64_t kLongPressMs = 500;
  static constexpr size_t kKeyboardMaxLength = 16;

  void render();
  void enter(Screen screen);
  bool pressed(HmiButton button) const;
  bool repeated(HmiButton button, uint64_t nowMs);
  void menuStep(uint64_t nowMs);
  void liveStep(uint64_t nowMs);
  void ingestTelemetry();
  void updateAutomaticUsbStorage();
  void testingStep(uint64_t nowMs);
  void dataStep(uint64_t nowMs);
  void sensorsStep(uint64_t nowMs);
  void settingsStep(uint64_t nowMs);
  void updateRecoveryLocations();
  bool showRecoveryLocation(size_t linkIndex);
  void emit(const char* type, uint8_t link = 0, int32_t value = 0, const std::string& text = {});
  bool showQr(const char* view, float latitude, float longitude);
  void clearQr();
  [[nodiscard]] std::string screenName() const;
  [[nodiscard]] std::string testingName() const;
  [[nodiscard]] std::string calibrationName() const;
  [[nodiscard]] std::string dataSubviewName() const;

  IHmiRenderer& renderer_;
  ITelemetryLink& link1_;
  ITelemetryLink& link2_;
  INavigation& navigation_;
  ILogStore& logs_;
  IConfigStore& config_;
  IDeviceStatus& device_;
  IClock& clock_;

  Screen screen_ = Screen::Logo;
  TestingState testingState_ = TestingState::Disclaimer;
  CalibrationState calibrationState_ = CalibrationState::Idle;
  HmiInput input_{};
  HmiInput previousInput_{};
  std::array<uint64_t, 7> heldSince_{};
  std::array<uint64_t, 7> lastRepeat_{};
  std::vector<PlatformAction> actions_;
  uint64_t nowMs_ = 0;
  uint64_t startupStartedMs_ = 0;
  uint32_t lastStartupFrame_ = UINT32_MAX;
  uint64_t testingStartedMs_ = 0;
  int16_t menuSelection_ = 0;
  int16_t settingsPage_ = 0;
  int16_t settingsSelection_ = -1;
  int16_t dataSelection_ = 0;
  int16_t testingSelection_ = 0;
  size_t logScrollOffset_ = 0;
  DataSubview dataSubview_ = DataSubview::List;
  std::string dataMessageTitle_;
  std::string dataMessageText_;
  std::string usbStorageMessage_;
  std::string qrView_ = "none";
  std::string qrUrl_;
  std::array<FlightStatisticsSnapshot, 2> recoveryLocations_{};
  int8_t selectedRecoveryLink_ = 0;
  bool liveDownrange_ = false;
  bool keyboardActive_ = false;
  int16_t keyboardSelection_ = 0;
  bool keyboardUppercase_ = false;
  bool usbStorageSession_ = false;
  bool usbPreviouslyConnected_ = false;
  bool automaticUsbSharePending_ = false;
  std::string previousRecorderState_ = "idle";
};
