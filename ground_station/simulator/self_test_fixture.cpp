// Simulated hardware observations for the production factory-test state machine.
// Acceptance and phase transitions live exclusively in src/self_test.cpp.
#include "hmi_controller.hpp"
#include <algorithm>
#include <cstring>

void HmiController::selfTestRadio(SelfTest::RadioConfiguration configuration) {
  using Radio = SelfTest::RadioConfiguration;
  selfTestRadio_ = configuration;
  const char* first = SelfTestProfile::kPhrase1;
  const char* second = SelfTestProfile::kPhrase2;
  if (configuration == Radio::Restore) {
    link1_.setLinkPhrase(config_.config().linkPhrase1);
    link2_.setLinkPhrase(config_.config().dualReceiver ? config_.config().linkPhrase2 : config_.config().linkPhrase1);
    if (config_.config().linkPhrase1.empty()) link1_.disable(); else link1_.enable();
    if ((config_.config().dualReceiver ? config_.config().linkPhrase2 : config_.config().linkPhrase1).empty()) link2_.disable(); else link2_.enable();
    emit("self_test_restored");
    return;
  }
  if (configuration == Radio::Swapped) std::swap(first, second);
  else if (configuration != Radio::Dual) second = first;
  link1_.setLinkPhrase(first);
  link2_.setLinkPhrase(second);
  if (configuration == Radio::Receiver2Only) link1_.disable(); else link1_.enable();
  if (configuration == Radio::Receiver1Only) link2_.disable(); else link2_.enable();
  emit("self_test_radio", 0, static_cast<int32_t>(configuration));
}

void HmiController::selfTestStep(uint64_t nowMs) {
  const auto now = static_cast<uint32_t>(nowMs);
  const auto device = device_.snapshot();
  selfTestInput_.buttonsHeld = selfTestInput_.buttonsPressed = 0;
  selfTestInput_.usbConnected = device.usb;
  selfTestInput_.usbStorageReady = device.usbStorageState == "host";
  selfTestInput_.usbStorageFault = device.usbStorageState == "fault";
  selfTestInput_.usbReadCount = device.usbReadCount;
  selfTestInput_.batteryVoltage = device.batteryVoltage;
  selfTestInput_.linksReady = true;
  for (size_t i = 0; i < 7; ++i) {
    if (input_.held[i]) selfTestInput_.buttonsHeld |= static_cast<uint8_t>(1U << i);
    if (pressed(static_cast<HmiButton>(i))) selfTestInput_.buttonsPressed |= static_cast<uint8_t>(1U << i);
  }
  if (!selfTest_.started() || selfTest_.finished()) {
    if (pressed(HmiButton::Back)) {
      screen_ = Screen::Settings;
      automaticUsbSharePending_ = device.usb;
      return;
    }
    if (selfTest_.finished()) {
      if (pressed(HmiButton::Down)) selfTestResultIndex_ = std::min<int16_t>(static_cast<int16_t>(SelfTest::Check::Count)-1, selfTestResultIndex_+1);
      if (pressed(HmiButton::Up)) selfTestResultIndex_ = std::max<int16_t>(0, selfTestResultIndex_-1);
    }
    if (!pressed(HmiButton::Ok)) return;
    if (logs_.recorderStatus().state != "idle") { emit("self_test_busy"); return; }
    device_.requestFirmwareStorage();
    selfTest_.start(now, selfTestInput_);
    selfTest_.setResult(SelfTest::Check::Storage, device.selfTestStorageFailure ? SelfTest::Result::Fail : SelfTest::Result::Pass,
                       device.selfTestStorageFailure ? "Storage write/read check failed" : "Written, reopened and verified");
    selfTestRadio_ = SelfTest::RadioConfiguration::Restore;
    selfTestLastPacket_ = selfTestLastFix_ = nowMs;
    selfTestResultIndex_ = 0;
    emit("self_test_started");
  }
  if (input_.held[6] && nowMs - heldSince_[6] >= 2000U && !selfTest_.cancelled) selfTest_.cancel(now);

  // Fixture packets arrive at 20 Hz; the normal combined stream is 10 Hz.
  if (nowMs - selfTestLastPacket_ >= 50) {
    selfTestLastPacket_ = nowMs;
    bool received = false;
    for (size_t i = 0; i < 2; ++i) {
      const bool enabled = (i == 0 ? link1_.snapshot() : link2_.snapshot()).enabled;
      if (!enabled || (device.selfTestMissingReceiver & (1U << i)) != 0) continue;
      auto& radio = selfTestInput_.links[i].radio;
      ++radio.packets;
      ++radio.infoSamples;
      radio.maxGapMs = std::max(radio.maxGapMs, now - radio.lastPacketMs);
      radio.lastPacketMs = now;
      radio.lqSum += device.selfTestLq;
      radio.minimumSnr = std::min(radio.minimumSnr, device.selfTestSnr);
      received = true;
    }
    if (received && selfTestRadio_ >= SelfTest::RadioConfiguration::Single && now % 100 < 60) ++selfTestInput_.combinedPackets;
  }
  if (device.selfTestGnss && nowMs - selfTestLastFix_ >= 1000) {
    selfTestLastFix_ = nowMs;
    auto& gps = selfTestInput_.links[1];
    gps.fixCount += 5;
    gps.lastFixMs = gps.lastTimeMs = now;
    ++gps.timeUpdates;
    gps.utcSeconds = now / 1000;
    gps.latitude = 46;
    gps.longitude = 8;
    gps.satellites = 9;
  }
  const auto nav = navigation_.snapshot();
  auto& sensor = selfTestInput_.sensors;
  ++sensor.sequence;
  sensor.sampledAtMs = now;
  sensor.imuDetected = sensor.accelerationFresh = sensor.gyroFresh = sensor.magnetometerFresh = !device.selfTestSensorFailure;
  sensor.magnetometer = device.selfTestSensorFailure ? 0 : 0x0d;
  sensor.readError = device.selfTestSensorFailure;
  sensor.acceleration = {nav.ax, nav.ay, nav.az};
  sensor.rawGyro = {nav.gx, nav.gy, nav.gz};
  for (size_t i = 0; i < 3; ++i) sensor.gyro[i] = sensor.rawGyro[i] - device.gyroBias[i];
  sensor.magnetic = {nav.mx, nav.my, nav.mz};
  // A default field allows a stationary healthy fixture; motion still requires
  // explicit navigation injections, so the motion test cannot pass by waiting.
  if (nav.mx == 0 && nav.my == 0 && nav.mz == 0) sensor.magnetic = {500, 300, 700};

  selfTest_.update(now, selfTestInput_);
  std::array<float, 3> bias{};
  if (selfTest_.takeGyroCalibration(bias))
    selfTest_.completeGyroCalibration(device_.saveGyroCalibration(bias), now, selfTestInput_);
  if (selfTest_.takeUsbStorageRequest() && !device_.requestMassStorage())
    selfTest_.setResult(SelfTest::Check::Usb, SelfTest::Result::Fail, "Could not share USB storage");
  const auto radio = selfTest_.takeRadioConfiguration();
  if (radio != SelfTest::RadioConfiguration::None) selfTestRadio(radio);
  if (selfTest_.takeVersionRequest()) {
    for (size_t i = 0; i < 2; ++i) if ((device.selfTestMissingReceiver & (1U << i)) == 0) {
      ++selfTestInput_.links[i].versionReplies;
      std::strcpy(selfTestInput_.links[i].version, "fixture-version");
    }
  }
  if (selfTest_.takeRadioReset()) {
    for (auto& link : selfTestInput_.links) { link.radio = {}; link.radio.lastPacketMs = now; }
  }
  if (selfTest_.finished() && device.usb) device_.requestMassStorage();
}
