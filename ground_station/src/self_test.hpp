/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Shared by the firmware and simulator. Hardware adapters provide observations;
// this state machine owns timing, acceptance and the order of operator prompts.
struct SelfTestRadioStats {
  uint32_t packets{0};
  uint32_t infoSamples{0};
  uint32_t lqSum{0};
  int8_t minimumSnr{127};
  uint32_t lastPacketMs{0};
  uint32_t maxGapMs{0};
};

struct SelfTestLinkObservation {
  SelfTestRadioStats radio{};
  char version[17]{};
  uint32_t versionReplies{0};
  uint32_t fixCount{0};
  uint32_t lastFixMs{0};
  uint32_t timeUpdates{0};
  uint32_t lastTimeMs{0};
  uint32_t utcSeconds{0};
  float latitude{0};
  float longitude{0};
  int16_t satellites{-1};
};

struct SelfTestSensorObservation {
  uint32_t sequence{0};
  uint32_t sampledAtMs{0};
  bool imuDetected{false};
  uint8_t magnetometer{0};  // I2C address: 0x0d (L), 0x2c (P), or zero.
  bool accelerationFresh{false};
  bool gyroFresh{false};
  bool magnetometerFresh{false};
  bool readError{false};
  std::array<float, 3> acceleration{};
  std::array<float, 3> gyro{};
  std::array<float, 3> rawGyro{};   // Degrees/s before the saved zero-rate offsets.
  std::array<float, 3> magnetic{};  // Uncalibrated ADC counts, for both variants.
};

struct SelfTestInput {
  std::array<SelfTestLinkObservation, 2> links{};
  SelfTestSensorObservation sensors{};
  bool linksReady{false};
  bool usbConnected{false};
  bool usbStorageReady{false};
  bool usbStorageFault{false};
  uint32_t usbReadCount{0};
  float batteryVoltage{0};
  uint32_t combinedPackets{0};
  uint8_t buttonsHeld{0};  // up, down, left, right, center, A, B
  uint8_t buttonsPressed{0};
};

struct SelfTestProfile {
  // Initial bench limits: qualify on the actual fixture before accepting a batch.
  static constexpr const char* kPhrase1 = "cats_test_1";
  static constexpr const char* kPhrase2 = "cats_test_2";
  static constexpr uint32_t kSettleMs = 2500;
  static constexpr uint32_t kRadioWindowMs = 15000;
  static constexpr uint32_t kFailoverWindowMs = 5000;
  static constexpr uint32_t kGnssTimeoutMs = 120000;
  static constexpr uint32_t kMinimumPacketsPerSecond = 8;
  static constexpr uint32_t kMaximumGapMs = 1000;
  static constexpr uint32_t kMinimumMeanLq = 80;
  static constexpr int8_t kSnrThreshold = -5;  // Every report must be strictly above this value, in dB.
  static constexpr uint32_t kDisplayLeadInMs = 2000;
  static constexpr uint32_t kDisplayPatternMs = 4000;
  static constexpr uint32_t kDisplayEndMs = kDisplayLeadInMs + 4 * kDisplayPatternMs;
  static constexpr uint32_t kGyroCalibrationMs = 5000;
  static constexpr float kMaximumGyroBias = 5.0F;
};

class SelfTest {
 public:
  static constexpr uint8_t kRequiredButtons = 0x6f;  // Up, down, left, right, A, B; center (bit 4) is not fitted.
  enum class Result : uint8_t { Pending, Pass, Fail, Incomplete };
  enum class Check : uint8_t {
    GyroCalibration,
    Firmware1,
    Firmware2,
    Gnss,
    Stationary,
    Magnetic,
    Dual,
    Swapped,
    Single,
    Receiver1Only,
    Receiver2Only,
    Storage,
    Motion,
    Buttons,
    Display,
    Battery,
    Usb,
    Restoration,
    Count
  };
  enum class Phase : uint8_t {
    Ready,
    GyroCalibration,
    Hardware,
    Telemetry,
    Motion,
    Buttons,
    Display,
    Restoring,
    Usb,
    Finished
  };
  enum class RadioConfiguration : int8_t {
    None = -1,
    Dual = 0,
    Swapped = 1,
    Single = 2,
    Receiver1Only = 3,
    Receiver2Only = 4,
    Restore = 5
  };
  struct CheckResult {
    Result result{Result::Pending};
    const char* reason{"Not tested"};
  };
  void start(uint32_t now, const SelfTestInput& input);
  void update(uint32_t now, const SelfTestInput& input);
  void cancel(uint32_t now);
  void setResult(Check check, Result result, const char* reason);
  [[nodiscard]] const CheckResult& check(Check check) const;
  [[nodiscard]] Result overall() const;
  [[nodiscard]] bool started() const { return phase != Phase::Ready; }
  [[nodiscard]] bool finished() const { return phase == Phase::Finished; }
  [[nodiscard]] const char* title() const;
  [[nodiscard]] const char* instruction() const;
  [[nodiscard]] uint32_t elapsedMs(uint32_t now) const { return now - startedAt; }
  [[nodiscard]] uint32_t phaseElapsedMs(uint32_t now) const { return now - phaseStartedAt; }
  [[nodiscard]] uint8_t displayPattern(uint32_t now) const;
  [[nodiscard]] RadioConfiguration takeRadioConfiguration();
  bool takeRadioReset();
  bool takeVersionRequest();
  bool takeGyroCalibration(std::array<float, 3>& bias);
  void completeGyroCalibration(bool saved, uint32_t now, const SelfTestInput& input);
  bool takeUsbStorageRequest();
  static const char* resultName(Result result);
  static const char* checkName(Check check);

  Phase phase{Phase::Ready};
  std::array<CheckResult, static_cast<size_t>(Check::Count)> results{};
  uint8_t radioStage{0};
  bool measuring{false};
  bool awaitingConfirmation{false};
  uint8_t completedButtons{0};
  uint8_t accelerationAxes{0};
  uint8_t gyroAxes{0};
  uint8_t magneticAxes{0};
  int16_t satellites{-1};
  uint32_t sensorErrors{0};
  bool cancelled{false};
  uint32_t gyroCalibrationSamples{0};

 private:
  void enter(Phase next, uint32_t now);
  void observeVersions(uint32_t now, const SelfTestInput& input);
  void observeGnss(uint32_t now, const SelfTestInput& input);
  void observeSensors(uint32_t now, const SelfTestSensorObservation& input);
  void updateRadio(uint32_t now, const SelfTestInput& input);
  void completeRadio(uint32_t now);
  void resetStationary();
  void observeGyroCalibration(uint32_t now, const SelfTestSensorObservation& input);
  void resetGyroCalibration();
  void finish(uint32_t now);
  RadioConfiguration radioRequest{RadioConfiguration::None};
  bool radioReset{false};
  bool versionRequest{false};
  bool gyroCalibrationPending{false};
  bool gyroCalibrationRequested{false};
  bool usbStorageRequest{false};
  bool usbStorageRequested{false};
  uint32_t usbReadBaseline{0};
  uint32_t gyroCalibrationStartedAt{0};
  uint32_t gyroCalibrationLastSample{0};
  std::array<float, 3> gyroCalibrationBias{};
  std::array<float, 3> calibrationAccelMin{};
  std::array<float, 3> calibrationAccelMax{};
  std::array<float, 3> calibrationGyroMin{};
  std::array<float, 3> calibrationGyroMax{};
  uint32_t startedAt{0};
  uint32_t phaseStartedAt{0};
  uint32_t radioStartedAt{0};
  uint32_t measurementStartedAt{0};
  uint32_t lastVersionRequest{0};
  uint32_t sensorSequence{0};
  uint32_t stationaryStartedAt{0};
  uint32_t stationarySamples{0};
  uint32_t initialFixCount{0};
  uint32_t initialUtcSeconds{0};
  uint32_t initialVersionReplies[2]{};
  uint32_t combinedBaseline{0};
  uint32_t lastCombinedCount{0};
  uint32_t lastCombinedAt{0};
  uint32_t combinedMaxGap{0};
  uint8_t pressedButtons{0};
  uint8_t blockedButtons{0x7f};
  uint8_t radioAttempts{0};
  bool radioChanged{false};
  bool batteryVoltageValid{true};
  bool batteryUsbSeen{false};
  Phase afterRestore{Phase::Motion};
  float accelSum{0};
  float accelSquareSum{0};
  std::array<float, 3> gyroSum{};
  std::array<float, 3> gyroSquareSum{};
  std::array<float, 3> accelerationMin{{100, 100, 100}};
  std::array<float, 3> accelerationMax{{-100, -100, -100}};
  std::array<float, 3> magneticMin{{32768, 32768, 32768}};
  std::array<float, 3> magneticMax{{-32768, -32768, -32768}};
};
