/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later
#include "self_test.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr uint8_t kA = 1U << 5;
constexpr uint8_t kB = 1U << 6;
constexpr SelfTest::Check kRadioChecks[] = {SelfTest::Check::Dual, SelfTest::Check::Swapped, SelfTest::Check::Single,
                                            SelfTest::Check::Receiver1Only, SelfTest::Check::Receiver2Only};
bool finite(const std::array<float, 3>& v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }
float magnitude(const std::array<float, 3>& v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
bool plausibleBattery(float voltage) { return std::isfinite(voltage) && voltage >= 3.0F && voltage <= 4.35F; }
}  // namespace

void SelfTest::start(uint32_t now, const SelfTestInput& input) {
  *this = SelfTest{};
  startedAt = now;
  initialFixCount = input.links[1].fixCount;
  initialUtcSeconds = input.links[1].utcSeconds;
  sensorSequence = input.sensors.sequence;
  enter(Phase::Hardware, now);
}

void SelfTest::setResult(Check item, Result result, const char* reason) {
  results[static_cast<size_t>(item)] = {result, reason};
}
const SelfTest::CheckResult& SelfTest::check(Check item) const { return results[static_cast<size_t>(item)]; }

SelfTest::Result SelfTest::overall() const {
  if (!finished()) return Result::Pending;
  for (const auto& item : results)
    if (item.result != Result::Pass) return Result::Fail;
  return cancelled ? Result::Fail : Result::Pass;
}

void SelfTest::enter(Phase next, uint32_t now) {
  phase = next;
  phaseStartedAt = now;
  blockedButtons = 0x7f;
  awaitingConfirmation = next == Phase::Telemetry || next == Phase::Motion || next == Phase::Buttons ||
                         next == Phase::Display || next == Phase::Leds || next == Phase::Unplug ||
                         next == Phase::Reconnect;
}

void SelfTest::observeVersions(uint32_t now, const SelfTestInput& input) {
  for (size_t i = 0; i < 2; ++i) {
    const auto item = i == 0 ? Check::Firmware1 : Check::Firmware2;
    if (check(item).result != Result::Pending) continue;
    if (input.links[i].versionReplies != initialVersionReplies[i]) {
      const bool present = input.links[i].version[0] != '\0';
      setResult(item, present ? Result::Pass : Result::Fail, present ? "Fresh version reply" : "Empty version reply");
    } else if (now - phaseStartedAt >= 15000U) {
      setResult(item, Result::Fail, "No version reply after retries");
    }
  }
  if (now - lastVersionRequest >= 1000U && now - phaseStartedAt < 15000U) {
    versionRequest = true;
    lastVersionRequest = now;
  }
}

void SelfTest::observeGnss(uint32_t now, const SelfTestInput& input) {
  const auto& gps = input.links[1];
  satellites = gps.satellites;
  if (check(Check::Gnss).result == Result::Pending) {
    const bool valid = std::isfinite(gps.latitude) && std::isfinite(gps.longitude) && std::fabs(gps.latitude) <= 90 &&
                       std::fabs(gps.longitude) <= 180 && (gps.latitude != 0 || gps.longitude != 0);
    if (valid && gps.fixCount - initialFixCount >= 5 && now - gps.lastFixMs < 1500U && gps.timeUpdates > 0 &&
        now - gps.lastTimeMs < 2000U && gps.utcSeconds != initialUtcSeconds) {
      setResult(Check::Gnss, Result::Pass, "Fresh fixes and advancing GNSS time");
    } else if (now - startedAt >= SelfTestProfile::kGnssTimeoutMs) {
      setResult(Check::Gnss, Result::Fail, "No fresh fix: retest with open sky");
    }
  }
}

void SelfTest::resetStationary() {
  stationarySamples = 0;
  accelSum = accelSquareSum = 0;
  gyroSum.fill(0);
  gyroSquareSum.fill(0);
}

void SelfTest::observeSensors(uint32_t now, const SelfTestSensorObservation& s) {
  if (s.sequence == sensorSequence || now - s.sampledAtMs > 500U) return;
  sensorSequence = s.sequence;
  if (s.readError) ++sensorErrors;
  const bool goodImu = s.imuDetected && s.accelerationFresh && s.gyroFresh && finite(s.acceleration) && finite(s.gyro);
  const bool goodMag =
      s.magnetometer != 0 && s.magnetometerFresh && finite(s.magnetic) && magnitude(s.magnetic) > 10 &&
      std::all_of(s.magnetic.begin(), s.magnetic.end(), [](float value) { return std::fabs(value) < 32000; });
  if (phase == Phase::Hardware && goodMag && check(Check::Magnetic).result == Result::Pending)
    setResult(Check::Magnetic, Result::Pass, "Detected; fresh, unsaturated raw data");
  if (phase == Phase::Hardware && check(Check::Stationary).result == Result::Pending) {
    const float gravity = magnitude(s.acceleration);
    const bool still = goodImu && gravity >= 0.8F && gravity <= 1.2F &&
                       std::all_of(s.gyro.begin(), s.gyro.end(), [](float value) { return std::fabs(value) <= 5.0F; });
    if (!still) {
      resetStationary();
      return;
    }
    if (stationarySamples == 0) stationaryStartedAt = now;
    ++stationarySamples;
    accelSum += gravity;
    accelSquareSum += gravity * gravity;
    for (size_t i = 0; i < 3; ++i) {
      gyroSum[i] += s.gyro[i];
      gyroSquareSum[i] += s.gyro[i] * s.gyro[i];
    }
    if (now - stationaryStartedAt >= 2000U && stationarySamples >= 50) {
      const float count = static_cast<float>(stationarySamples);
      const float accelerationMean = accelSum / count;
      const float accelerationNoise =
          std::sqrt(std::max(0.0F, accelSquareSum / count - accelerationMean * accelerationMean));
      float gyroNoise = 0;
      for (size_t i = 0; i < 3; ++i) {
        const float mean = gyroSum[i] / count;
        gyroNoise = std::max(gyroNoise, std::sqrt(std::max(0.0F, gyroSquareSum[i] / count - mean * mean)));
      }
      if (accelerationNoise <= 0.05F && gyroNoise <= 1.5F) {
        setResult(Check::Stationary, Result::Pass, "Gravity, gyro bias and noise within limits");
      } else
        resetStationary();
    }
  }
  if (phase != Phase::Motion) return;
  for (size_t i = 0; i < 3; ++i) {
    const auto bit = static_cast<uint8_t>(1U << i);
    if (goodImu) {
      accelerationMin[i] = std::min(accelerationMin[i], s.acceleration[i]);
      accelerationMax[i] = std::max(accelerationMax[i], s.acceleration[i]);
      if (accelerationMax[i] - accelerationMin[i] >= 0.7F) accelerationAxes |= bit;
      if (std::fabs(s.gyro[i]) >= 15 && std::fabs(s.gyro[i]) < 1900) gyroAxes |= bit;
    }
    if (goodMag) {
      magneticMin[i] = std::min(magneticMin[i], s.magnetic[i]);
      magneticMax[i] = std::max(magneticMax[i], s.magnetic[i]);
      if (magneticMax[i] - magneticMin[i] >= 100) magneticAxes |= bit;
    }
  }
}

void SelfTest::updateRadio(uint32_t now, const SelfTestInput& input) {
  if (!measuring) {
    if (now - radioStartedAt >= SelfTestProfile::kSettleMs && input.linksReady) {
      radioReset = true;
      measuring = true;
      measurementStartedAt = now;
      combinedBaseline = lastCombinedCount = input.combinedPackets;
      lastCombinedAt = now;
      combinedMaxGap = 0;
    } else if (now - radioStartedAt >= 10000U) {
      setResult(kRadioChecks[radioStage], Result::Fail, "Receiver configuration did not complete");
      completeRadio(now);
    }
    return;
  }
  if (input.combinedPackets != lastCombinedCount) {
    combinedMaxGap = std::max(combinedMaxGap, now - lastCombinedAt);
    lastCombinedAt = now;
    lastCombinedCount = input.combinedPackets;
  }
  const uint32_t duration = radioStage < 3 ? SelfTestProfile::kRadioWindowMs : SelfTestProfile::kFailoverWindowMs;
  if (now - measurementStartedAt < duration) return;
  const uint32_t measuredMs = now - measurementStartedAt;
  const uint32_t combinedPackets = input.combinedPackets - combinedBaseline;
  bool passed = true;
  for (size_t i = 0; i < 2; ++i) {
    if ((radioStage == 3 && i == 1) || (radioStage == 4 && i == 0)) continue;
    auto stats = input.links[i].radio;
    stats.maxGapMs = std::max(stats.maxGapMs, stats.packets == 0 ? measuredMs : now - stats.lastPacketMs);
    passed &= stats.packets >= measuredMs * SelfTestProfile::kMinimumPacketsPerSecond / 1000 &&
              stats.maxGapMs <= SelfTestProfile::kMaximumGapMs && stats.infoSamples > 0 &&
              stats.lqSum / stats.infoSamples >= SelfTestProfile::kMinimumMeanLq;
  }
  if (radioStage >= 2) {
    combinedMaxGap = std::max(combinedMaxGap, now - lastCombinedAt);
    passed &= combinedPackets >= measuredMs * SelfTestProfile::kMinimumPacketsPerSecond / 1000 &&
              combinedMaxGap <= SelfTestProfile::kMaximumGapMs;
  }
  setResult(kRadioChecks[radioStage], passed ? Result::Pass : Result::Fail,
            passed ? "Packet rate, gaps and quality within limits" : "Low packet rate, quality or reception gap");
  completeRadio(now);
}

void SelfTest::completeRadio(uint32_t now) {
  ++radioAttempts;
  measuring = false;
  if (check(kRadioChecks[radioStage]).result == Result::Fail && radioAttempts < 2) {
    setResult(kRadioChecks[radioStage], Result::Pending, "Retrying receiver test");
  } else if (++radioStage >= 5) {
    radioStage = 4;
    radioRequest = RadioConfiguration::Restore;
    afterRestore = Phase::Motion;
    enter(Phase::Restoring, now);
    return;
  } else {
    radioAttempts = 0;
  }
  radioStartedAt = now;
  radioRequest = static_cast<RadioConfiguration>(radioStage);
}

void SelfTest::update(uint32_t now, const SelfTestInput& input) {
  if (!started() || finished()) return;
  blockedButtons &= input.buttonsHeld;
  const uint8_t pressed = input.buttonsPressed & static_cast<uint8_t>(~blockedButtons);
  // A confirmation starts exactly one check. Ignore its press/release as test
  // input, and leave every measurement and timeout stopped while waiting.
  if (awaitingConfirmation) {
    if ((pressed & kA) != 0) {
      awaitingConfirmation = false;
      phaseStartedAt = now;
      blockedButtons = input.buttonsHeld;
      sensorSequence = input.sensors.sequence;
      if (phase == Phase::Telemetry) {
        for (size_t i = 0; i < 2; ++i) initialVersionReplies[i] = input.links[i].versionReplies;
        versionRequest = true;
        lastVersionRequest = radioStartedAt = now;
        radioRequest = RadioConfiguration::Dual;
        radioChanged = true;
      } else if (phase == Phase::Leds) {
        radioRequest = RadioConfiguration::Receiver1Only;
        radioChanged = true;
      } else if (phase == Phase::Reconnect) {
        usbDisconnected = !input.usbConnected;
      }
    }
    return;
  }
  if (phase == Phase::Hardware || phase == Phase::Motion) observeSensors(now, input.sensors);
  switch (phase) {
    case Phase::Hardware:
      observeGnss(now, input);
      if (now - phaseStartedAt >= 30000U) {
        if (check(Check::Stationary).result == Result::Pending)
          setResult(Check::Stationary, Result::Fail, "No stable fresh IMU measurement");
        if (check(Check::Magnetic).result == Result::Pending)
          setResult(Check::Magnetic, Result::Fail, "No valid magnetometer samples");
      }
      if (check(Check::Gnss).result != Result::Pending && check(Check::Stationary).result != Result::Pending &&
          check(Check::Magnetic).result != Result::Pending)
        enter(Phase::Telemetry, now);
      break;
    case Phase::Telemetry:
      observeVersions(now, input);
      updateRadio(now, input);
      break;
    case Phase::Motion:
      if (accelerationAxes == 7 && gyroAxes == 7 && magneticAxes == 7) {
        setResult(Check::Motion, Result::Pass, "All accelerometer, gyro and magnetic axes responded");
        enter(Phase::Buttons, now);
      } else if ((pressed & kA) != 0 || now - phaseStartedAt >= 60000U) {
        setResult(Check::Motion, (pressed & kA) != 0 ? Result::Incomplete : Result::Fail,
                  "Not all sensor axes demonstrated motion");
        enter(Phase::Buttons, now);
      }
      break;
    case Phase::Buttons:
      pressedButtons |= pressed;
      completedButtons |= pressedButtons & static_cast<uint8_t>(~input.buttonsHeld);
      if (completedButtons == 0x7f) {
        setResult(Check::Buttons, Result::Pass, "All seven buttons pressed and released");
        enter(Phase::Display, now);
      } else if (now - phaseStartedAt >= 60000U) {
        setResult(Check::Buttons, Result::Fail, "Missing button press or release");
        enter(Phase::Display, now);
      }
      break;
    case Phase::Display:
      if (now - phaseStartedAt >= 10000U && (pressed & (kA | kB)) != 0) {
        const bool passed = (pressed & kA) != 0;
        setResult(Check::Display, passed ? Result::Pass : Result::Fail,
                  passed ? "Operator confirmed all patterns" : "Operator reported display fault");
        enter(Phase::Leds, now);
      }
      break;
    case Phase::Leds:
      if (!ledSecond && now - phaseStartedAt >= 6000U) {
        ledSecond = true;
        radioRequest = RadioConfiguration::Receiver2Only;
      }
      if (now - phaseStartedAt >= 12000U && (pressed & (kA | kB)) != 0) {
        const bool passed = (pressed & kA) != 0;
        setResult(Check::Leds, passed ? Result::Pass : Result::Fail,
                  passed ? "Operator confirmed each receiver LED" : "Operator reported LED fault");
        radioRequest = RadioConfiguration::Restore;
        afterRestore = Phase::Unplug;
        enter(Phase::Restoring, now);
      }
      break;
    case Phase::Restoring:
      if (input.linksReady && now - phaseStartedAt >= SelfTestProfile::kSettleMs) {
        setResult(Check::Restoration, Result::Pass, "Normal receiver settings reapplied");
        radioChanged = false;
        if (afterRestore == Phase::Finished)
          finish(now);
        else
          enter(afterRestore, now);
      } else if (now - phaseStartedAt >= 10000U) {
        setResult(Check::Restoration, Result::Fail, "Receiver restoration timed out");
        finish(now);
      }
      break;
    case Phase::Unplug:
      if (!input.usbConnected) {
        batteryMinimum = input.batteryVoltage;
        enter(Phase::Battery, now);
      } else if ((pressed & kA) != 0) {
        setResult(Check::Battery, Result::Incomplete, "Battery operation skipped");
        setResult(Check::Usb, Result::Incomplete, "USB reconnect skipped");
        finish(now);
      }
      break;
    case Phase::Battery:
      batteryMinimum = std::min(batteryMinimum, input.batteryVoltage);
      if (input.usbConnected)
        enter(Phase::Unplug, now);
      else if (now - phaseStartedAt >= 5000U) {
        const bool passed = plausibleBattery(batteryMinimum);
        setResult(Check::Battery, passed ? Result::Pass : Result::Fail,
                  passed ? "Five seconds on battery" : "Battery voltage outside 3.0-4.35 V");
        enter(Phase::Reconnect, now);
      }
      break;
    case Phase::Reconnect:
      usbDisconnected |= !input.usbConnected;
      if (usbDisconnected && input.usbConnected) {
        setResult(Check::Usb, Result::Pass, "USB enumerated after battery operation");
        finish(now);
      } else if ((pressed & kA) != 0) {
        setResult(Check::Usb, Result::Incomplete, "USB reconnect skipped");
        finish(now);
      }
      break;
    default:
      break;
  }
}

void SelfTest::finish(uint32_t now) {
  if (sensorErrors > 5) setResult(Check::Motion, Result::Fail, "Repeated sensor read errors; inspect sensor bus");
  for (auto& item : results)
    if (item.result == Result::Pending) item = {Result::Incomplete, "Not completed"};
  enter(Phase::Finished, now);
}
void SelfTest::cancel(uint32_t now) {
  cancelled = true;
  if (!radioChanged) {
    if (check(Check::Restoration).result == Result::Pending)
      setResult(Check::Restoration, Result::Pass, "Normal receiver settings retained");
    finish(now);
    return;
  }
  afterRestore = Phase::Finished;
  radioRequest = RadioConfiguration::Restore;
  enter(Phase::Restoring, now);
}
SelfTest::RadioConfiguration SelfTest::takeRadioConfiguration() {
  const auto value = radioRequest;
  radioRequest = RadioConfiguration::None;
  return value;
}
bool SelfTest::takeRadioReset() {
  const bool value = radioReset;
  radioReset = false;
  return value;
}
bool SelfTest::takeVersionRequest() {
  const bool value = versionRequest;
  versionRequest = false;
  return value;
}
uint8_t SelfTest::displayPattern(uint32_t now) const {
  if (awaitingConfirmation || phase != Phase::Display || now - phaseStartedAt < 2000U || now - phaseStartedAt >= 10000U)
    return 0;
  return static_cast<uint8_t>(1U + (now - phaseStartedAt - 2000U) / 2000U);
}
const char* SelfTest::resultName(Result result) {
  static constexpr const char* names[] = {"WAIT", "PASS", "FAIL", "NOT TESTED"};
  return names[static_cast<size_t>(result)];
}
const char* SelfTest::checkName(Check item) {
  static constexpr const char* names[] = {"Telemetry 1 firmware",
                                          "Telemetry 2 firmware",
                                          "Onboard GNSS",
                                          "Stationary IMU",
                                          "Magnetometer",
                                          "Dual reception",
                                          "Dual swapped",
                                          "Single reception",
                                          "Receiver 1 alone",
                                          "Receiver 2 alone",
                                          "Storage",
                                          "Sensor movement",
                                          "Buttons",
                                          "Display",
                                          "Receiver LEDs",
                                          "Battery operation",
                                          "USB reconnect",
                                          "Settings restored"};
  return names[static_cast<size_t>(item)];
}
const char* SelfTest::title() const {
  static constexpr const char* names[] = {"Factory self-test",   "1. Ground Station",  "2. Telemetry",
                                          "3.1 Sensor movement", "3.2 Buttons",        "3.3 Display",
                                          "3.4 Receiver LEDs",   "Restoring settings", "3.5 Battery",
                                          "3.5 Battery",         "3.6 USB reconnect",  "Self-test results"};
  return names[static_cast<size_t>(phase)];
}
const char* SelfTest::instruction() const {
  if (awaitingConfirmation) return "Ready to start - press A when ready.";
  switch (phase) {
    case Phase::Ready:
      return "Rest on the bench with access to the sky.";
    case Phase::Hardware:
      return "Keep still. Checks run automatically.";
    case Phase::Telemetry:
      return "Testing both telemetry receivers.";
    case Phase::Motion:
      return "Tilt and turn around each axis.";
    case Phase::Buttons:
      return "Press and release each button, including B.";
    case Phase::Display:
      return "Inspect black, white and checker patterns.";
    case Phase::Leds:
      return ledSecond ? "Receiver 2 LED should blink." : "Receiver 1 LED should blink.";
    case Phase::Restoring:
      return "Restoring normal receiver settings.";
    case Phase::Unplug:
      return "Disconnect USB; keep the power switch on.";
    case Phase::Battery:
      return "Keep USB disconnected for 5 seconds.";
    case Phase::Reconnect:
      return usbDisconnected ? "Connect USB to the computer." : "Unplug USB, then reconnect it.";
    case Phase::Finished:
      return "Use up/down to view each result.";
  }
  return "";
}
