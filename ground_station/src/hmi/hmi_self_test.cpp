/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later
#include "hmi.hpp"
#include "telemetry/telemetry.hpp"

#include <algorithm>
#include <cstring>

extern Telemetry link1;
extern Telemetry link2;
extern Navigation navigation;

SelfTestInput Hmi::selfTestInput() {
  SelfTestInput input{};
  input.links = {link1.diagnostics(), link2.diagnostics()};
  input.linksReady = selfTestControlsOk && link1.selfTestReady() && link2.selfTestReady();
  input.sensors = navigation.selfTestSensors();
  input.usbConnected = Utils::isConnected();
  input.batteryVoltage = static_cast<float>(analogRead(18)) * 0.00062F;
  input.combinedPackets = combinedTelemetryPackets.load();
  Button* buttons[] = {&upButton, &downButton, &leftButton, &rightButton, &centerButton, &okButton, &backButton};
  for (size_t i = 0; i < 7; ++i) {
    if (buttons[i]->isPressed()) input.buttonsHeld |= static_cast<uint8_t>(1U << i);
    if (buttons[i]->wasPressed()) input.buttonsPressed |= static_cast<uint8_t>(1U << i);
  }
  return input;
}

void Hmi::initSelfTest() {
  state = SELF_TEST;
  selfTest = SelfTest{};
  selfTestResultIndex = 0;
  selfTestBackSince = 0;
  window.drawSelfTest(selfTest, millis(), 0);
}

void Hmi::startSelfTest() {
  if (recorder.getStatus().state != RecorderState::Idle) {
    window.initDataMessage("Self-test unavailable", "Finalize the active log first.");
    return;
  }
  // The queue barrier drains any in-flight packet before touching storage.
  recorder.disable();
  if (!recorder.sync() || recorder.getStatus().state != RecorderState::Idle) {
    recorder.enable();
    window.initDataMessage("Self-test unavailable", "Recorder is busy. Try again.");
    return;
  }
  automaticUsbSharePending = false;
  selfTestControlsOk = true;
  selfTestResultIndex = 0;
  selfTest.start(millis(), selfTestInput());
  navigation.enableSelfTestSampling(true);
  const bool storageOk = claimStorageForFirmware() && testStorage();
  selfTest.setResult(SelfTest::Check::Storage, storageOk ? SelfTest::Result::Pass : SelfTest::Result::Fail,
                     storageOk ? "Written, reopened and verified" : "Storage write/read check failed");
  window.drawSelfTest(selfTest, millis(), 0);
  selfTestLastDraw = millis();
}

void Hmi::applySelfTestRadio(SelfTest::RadioConfiguration configuration) {
  using Radio = SelfTest::RadioConfiguration;
  const char* first = SelfTestProfile::kPhrase1;
  const char* second = SelfTestProfile::kPhrase2;
  bool firstEnabled = true;
  bool secondEnabled = true;
  if (configuration == Radio::Restore) {
    // Include any ordinary Settings edits made before starting the test.
    link1.setLinkPhrase(systemConfig.config.linkPhrase1, kMaxPhraseLen);
    link2.setLinkPhrase(
        systemConfig.config.receiverMode == SINGLE ? systemConfig.config.linkPhrase1 : systemConfig.config.linkPhrase2,
        kMaxPhraseLen);
    first = second = nullptr;
    selfTestMode = -1;
  } else if (configuration == Radio::Swapped) {
    std::swap(first, second);
    selfTestMode = 1;
  } else if (configuration == Radio::Dual) {
    selfTestMode = 1;
  } else {
    second = first;
    firstEnabled = configuration != Radio::Receiver2Only;
    secondEnabled = configuration != Radio::Receiver1Only;
    selfTestMode = 0;
  }
  const bool firstOk = link1.setSelfTestOverride(first, firstEnabled);
  const bool secondOk = link2.setSelfTestOverride(second, secondEnabled);
  selfTestControlsOk = firstOk && secondOk;
}

bool Hmi::testStorage() {
  if (!Utils::isFilesystemAvailable()) return false;
  constexpr const char* path = "/self-test.tmp";
  uint8_t expected[256]{};
  for (size_t i = 0; i < sizeof(expected); ++i) expected[i] = static_cast<uint8_t>(i ^ 0xa5U);
  auto file = fatfs.open(path, O_WRONLY | O_CREAT | O_EXCL);
  if (!file) return false;
  const bool written = file.write(expected, sizeof(expected)) == sizeof(expected) && file.sync();
  file.close();
  uint8_t actual[256]{};
  file = fatfs.open(path, O_RDONLY);
  const bool read = file && file.read(actual, sizeof(actual)) == sizeof(actual) && file.size() == sizeof(actual);
  file.close();
  const bool removed = fatfs.remove(path);
  return written && read && removed && std::memcmp(expected, actual, sizeof(expected)) == 0;
}

void Hmi::leaveSelfTest() {
  navigation.enableSelfTestSampling(false);
  recorder.enable();
  automaticUsbSharePending = Utils::isConnected();
  state = SETTINGS;
  window.initBar();
  window.initSettings(settingSubMenu);
  window.updateSettings(settingIndex);
}

void Hmi::selfTestStep() {
  const uint32_t now = millis();
  if (!selfTest.started()) {
    if (backButton.wasPressed()) {
      leaveSelfTest();
      return;
    }
    if (okButton.wasPressed()) startSelfTest();
    return;
  }
  if (selfTest.finished()) {
    if (backButton.wasPressed()) {
      leaveSelfTest();
      return;
    }
    if (okButton.wasPressed()) {
      startSelfTest();
      return;
    }
    if (upButton.wasPressed() && selfTestResultIndex > 0) --selfTestResultIndex;
    if (downButton.wasPressed() && selfTestResultIndex < static_cast<int16_t>(SelfTest::Check::Count) - 1)
      ++selfTestResultIndex;
    if (upButton.wasPressed() || downButton.wasPressed()) window.drawSelfTest(selfTest, now, selfTestResultIndex);
    return;
  }
  if (backButton.isPressed()) {
    if (selfTestBackSince == 0) selfTestBackSince = now;
    if (now - selfTestBackSince >= 2000U && !selfTest.cancelled) selfTest.cancel(now);
  } else
    selfTestBackSince = 0;
  const auto previousPhase = selfTest.phase;
  const bool wasWaiting = selfTest.awaitingConfirmation;
  selfTest.update(now, selfTestInput());
  const auto radio = selfTest.takeRadioConfiguration();
  if (radio != SelfTest::RadioConfiguration::None) applySelfTestRadio(radio);
  if (selfTest.takeVersionRequest()) {
    link1.requestVersion();
    link2.requestVersion();
  }
  if (selfTest.takeRadioReset()) {
    link1.resetReceptionStats(now);
    link2.resetReceptionStats(now);
  }
  if (selfTest.finished()) {
    navigation.enableSelfTestSampling(false);
    if (Utils::isConnected()) recorder.shareWithMassStorage();
  }
  if (previousPhase != selfTest.phase || wasWaiting != selfTest.awaitingConfirmation ||
      now - selfTestLastDraw >= 200U) {
    window.drawSelfTest(selfTest, now, selfTestResultIndex);
    selfTestLastDraw = now;
  }
}
