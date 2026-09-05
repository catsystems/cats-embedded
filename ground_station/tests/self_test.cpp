// Contract tests for the production state machine, including manual start gates.
#include "self_test.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using Phase = SelfTest::Phase;
using Check = SelfTest::Check;
using Result = SelfTest::Result;
using Radio = SelfTest::RadioConfiguration;
constexpr uint8_t A = 1U << 5;
constexpr uint8_t B = 1U << 6;

struct Rig {
  SelfTest test;
  SelfTestInput input;
  uint32_t now{1000};
  uint32_t sinceStart{0};
  bool versions{true};
  bool gps{true};
  bool packets{true};
  uint8_t lq{90};
  int8_t snr{10};
  bool saveCalibration{true};
  unsigned calibrationSaves{0};
  std::array<float, 3> savedBias{};
  Radio configuration{Radio::Restore};
  std::array<unsigned, 6> requests{};

  Rig() {
    input.linksReady = true;
    input.batteryVoltage = 3.8F;
    input.sensors.imuDetected = true;
    input.sensors.magnetometer = 0x2c;
    input.sensors.accelerationFresh = input.sensors.gyroFresh = input.sensors.magnetometerFresh = true;
    input.sensors.acceleration = {0, 0, 1};
    input.sensors.magnetic = {500, 300, 700};
  }
  void start() {
    test.start(now, input);
    test.setResult(Check::Storage, Result::Pass, "fixture");
  }
  void tick() {
    now += 20;
    sinceStart += 20;
    ++input.sensors.sequence;
    input.sensors.sampledAtMs = now;
    for (size_t i = 0; i < 3; ++i) input.sensors.gyro[i] = input.sensors.rawGyro[i] - savedBias[i];
    if (packets && sinceStart % 100 == 0) {
      for (auto& link : input.links) {
        ++link.radio.packets;
        ++link.radio.infoSamples;
        link.radio.lqSum += lq;
        link.radio.minimumSnr = std::min(link.radio.minimumSnr, snr);
        link.radio.lastPacketMs = now;
        link.radio.maxGapMs = 100;
      }
      ++input.combinedPackets;
    }
    if (gps && sinceStart % 1000 == 0) {
      auto& gnss = input.links[1];
      gnss.fixCount += 5;
      gnss.latitude = 46;
      gnss.longitude = 8;
      gnss.lastFixMs = gnss.lastTimeMs = now;
      ++gnss.timeUpdates;
      ++gnss.utcSeconds;
    }
    test.update(now, input);
    std::array<float, 3> bias{};
    if (test.takeGyroCalibration(bias)) {
      if (saveCalibration) { savedBias = bias; ++calibrationSaves; }
      test.completeGyroCalibration(saveCalibration, now, input);
    }
    if (test.takeUsbStorageRequest()) input.usbStorageReady = true;
    input.buttonsPressed = 0;
    const auto request = test.takeRadioConfiguration();
    if (request != Radio::None) { configuration = request; ++requests[static_cast<size_t>(request)]; }
    if (test.takeVersionRequest() && versions) {
      for (auto& link : input.links) { std::strcpy(link.version, "any-version"); ++link.versionReplies; }
    }
    if (test.takeRadioReset()) for (auto& link : input.links) { link.radio = {}; link.radio.lastPacketMs = now; }
  }
  void advance(uint32_t duration) { for (uint32_t elapsed = 0; elapsed < duration; elapsed += 20) tick(); }
  void press(uint8_t button) { input.buttonsHeld |= button; input.buttonsPressed = button; tick(); }
  void release(uint8_t button) { input.buttonsHeld &= ~button; tick(); }
  void tap(uint8_t button) { press(button); release(button); }
  void hardware() {
    start(); advance(12000);
    assert(test.phase == Phase::Telemetry && test.awaitingConfirmation);
    assert(requests[0] == 0 && input.links[0].versionReplies == 0);
  }
  void radio() {
    hardware(); tap(A); advance(72000);
    assert(test.phase == Phase::Motion && test.awaitingConfirmation);
  }
  void gate(Phase phase) {
    assert(test.phase == phase && test.awaitingConfirmation);
    const auto before = test.results;
    const auto radioRequests = requests;
    advance(120000);  // Waiting must never consume the test timeout.
    tap(1U);         // An unrelated button must not start the check.
    assert(test.phase == phase && test.awaitingConfirmation);
    assert(test.displayPattern(now) == 0 && requests == radioRequests);
    for (size_t i = 0; i < before.size(); ++i) assert(test.results[i].result == before[i].result);
    tap(A);
    assert(test.phase == phase && !test.awaitingConfirmation);
  }
  void move() {
    for (size_t axis = 0; axis < 3; ++axis) for (float sign : {-1.0F, 1.0F}) {
      input.sensors.acceleration = {};
      input.sensors.acceleration[axis] = sign;
      input.sensors.rawGyro = {};
      input.sensors.rawGyro[axis] = 30;
      input.sensors.magnetic = {sign * 1000, sign * 800, sign * 600};
      tick();
    }
  }
};

int main() {
  {
    Rig rig;
    rig.hardware();
    assert(rig.test.check(Check::Battery).result == Result::Pass);
    rig.gate(Phase::Telemetry);
    rig.advance(72000);
    for (size_t i = 0; i <= static_cast<size_t>(Check::Storage); ++i)
      assert(rig.test.check(static_cast<Check>(i)).result == Result::Pass);
    rig.move();  // Sensor activity before confirmation does not count.
    assert(rig.test.accelerationAxes == 0 && rig.test.gyroAxes == 0 && rig.test.magneticAxes == 0);
    rig.gate(Phase::Motion);
    rig.move();
    rig.gate(Phase::Buttons);
    assert(rig.test.completedButtons == 0); // Start press AND release do not count as button A.
    rig.press(1U);
    assert(rig.test.completedButtons == 0);
    rig.release(1U);
    assert(rig.test.completedButtons == 1);
    for (unsigned bit : {1U, 2U, 3U, 5U}) rig.tap(1U << bit);
    assert(rig.test.phase == Phase::Buttons && rig.test.check(Check::Buttons).result == Result::Pending);
    rig.press(B);
    assert(rig.test.phase == Phase::Buttons);
    rig.release(B);
    assert(rig.test.check(Check::Buttons).result == Result::Pass); // All six physical buttons; no center input.
    rig.gate(Phase::Display);
    for (uint8_t pattern = 1; pattern <= 4; ++pattern) {
      rig.advance(pattern == 1 ? 2000 : 4000);
      assert(rig.test.displayPattern(rig.now) == pattern);
    }
    rig.advance(4100);
    rig.press(A); // Keep the approval button held through the next prompt.
    assert(rig.test.phase == Phase::Usb && rig.test.awaitingConfirmation);
    const auto requests = rig.requests;
    rig.advance(15000);
    assert(rig.test.awaitingConfirmation && rig.requests == requests);
    rig.release(A);
    assert(rig.test.phase == Phase::Usb && rig.test.awaitingConfirmation);
    rig.gate(Phase::Usb);
    rig.advance(1000);
    assert(!rig.test.finished() && !rig.input.usbStorageReady);
    rig.input.usbConnected = true; rig.tick();
    assert(!rig.test.finished() && rig.input.usbStorageReady); // Enumeration alone cannot pass.
    ++rig.input.usbReadCount; rig.tick();
    assert(rig.test.finished() && rig.test.overall() == Result::Pass);
    for (const auto& result : rig.test.results) assert(result.result == Result::Pass);
  }
  {
    Rig rig;
    rig.versions = false;
    rig.gps = false;
    for (auto& link : rig.input.links) { std::strcpy(link.version, "cached"); link.versionReplies = 1; }
    rig.input.links[1].fixCount = 10;
    rig.start(); rig.advance(127000);
    assert(rig.test.check(Check::Gnss).result == Result::Fail);
    assert(rig.test.phase == Phase::Telemetry && rig.test.awaitingConfirmation);
    rig.tap(A); rig.advance(72000);
    assert(rig.test.check(Check::Firmware1).result == Result::Fail);
    assert(rig.test.check(Check::Firmware2).result == Result::Fail);
    assert(rig.test.phase == Phase::Motion && rig.test.awaitingConfirmation);
  }
  {
    Rig rig;
    rig.lq = 10;
    rig.hardware(); rig.tap(A); rig.advance(140000);
    assert(rig.test.check(Check::Dual).result == Result::Fail && rig.requests[0] == 2);
    assert(rig.test.check(Check::Receiver2Only).result == Result::Fail);
    assert(rig.test.phase == Phase::Motion && rig.test.awaitingConfirmation);
  }
  {
    Rig rig;
    rig.hardware(); rig.tap(A); rig.advance(3000);
    rig.test.cancel(rig.now);
    rig.input.linksReady = false;
    rig.advance(5000);
    assert(!rig.test.finished() && rig.configuration == Radio::Restore);
    rig.input.linksReady = true; rig.tick();
    assert(rig.test.finished() && rig.test.overall() == Result::Fail);
    assert(rig.test.check(Check::Motion).result == Result::Incomplete);
  }
  {
    Rig rig;
    rig.start(); rig.test.cancel(rig.now);
    assert(rig.test.finished() && rig.test.takeRadioConfiguration() == Radio::None);
    assert(rig.test.check(Check::Restoration).result == Result::Pass);
  }
  {
    Rig rig;
    rig.now = UINT32_MAX - 3000;
    rig.radio(); // millis() wrap remains safe.
    rig.gate(Phase::Motion); rig.advance(60000);
    assert(rig.test.check(Check::Motion).result == Result::Fail);
    assert(rig.test.phase == Phase::Buttons && rig.test.awaitingConfirmation);
  }
  {
    Rig rig;
    rig.hardware(); rig.tap(A); rig.input.linksReady = false; rig.advance(115000);
    assert(rig.test.finished() && rig.test.check(Check::Restoration).result == Result::Fail);
    assert(rig.test.overall() == Result::Fail);
  }
  {
    Rig rig;
    rig.start(); rig.advance(5100); rig.input.sensors.readError = true; rig.advance(200);
    rig.input.sensors.readError = false; rig.advance(3000);
    rig.test.cancel(rig.now);
    assert(rig.test.finished() && rig.test.check(Check::Motion).result == Result::Fail);
  }
  {
    Rig rig;
    rig.radio(); rig.gate(Phase::Motion); rig.move(); rig.gate(Phase::Buttons);
    constexpr uint8_t center = 1U << 4; // Retained input must not affect the physical button checklist.
    rig.tap(center);
    assert(rig.test.completedButtons == 0);
    rig.press(center);
    for (unsigned bit : {0U, 1U, 2U, 3U, 5U, 6U}) rig.tap(1U << bit);
    assert(rig.test.phase == Phase::Display && rig.test.check(Check::Buttons).result == Result::Pass);
  }
  {
    SelfTest test;
    test.phase = Phase::Display;
    assert(test.displayPattern(1999) == 0);
    for (uint32_t pattern = 1; pattern <= 4; ++pattern) {
      const uint32_t begins = 2000 + (pattern - 1) * 4000;
      assert(test.displayPattern(begins) == pattern);
      assert(test.displayPattern(begins + 3999) == pattern);
    }
    assert(test.displayPattern(18000) == 0);
  }
  const auto radioWindowPasses = [](uint32_t packets, uint32_t lq, int8_t snr, uint32_t gap, uint32_t reports = 1,
                                    uint32_t elapsedMs = 15000) {
    Rig rig;
    rig.hardware(); rig.tap(A); rig.advance(2480);
    assert(rig.test.measuring);
    const uint32_t end = rig.now + elapsedMs;
    for (auto& link : rig.input.links) {
      link.radio.packets = packets;
      link.radio.infoSamples = reports;
      link.radio.lqSum = lq * reports;
      link.radio.minimumSnr = snr;
      link.radio.lastPacketMs = end;
      link.radio.maxGapMs = gap;
    }
    rig.test.update(end, rig.input);
    return rig.test.check(Check::Dual).result == Result::Pass;
  };
  assert(radioWindowPasses(120, 80, -4, 1000));
  assert(!radioWindowPasses(119, 80, -4, 1000));
  assert(!radioWindowPasses(120, 79, -4, 1000));
  assert(!radioWindowPasses(120, 80, -5, 1000));
  assert(!radioWindowPasses(120, 80, -6, 1000));
  assert(!radioWindowPasses(120, 80, -4, 1001));
  assert(!radioWindowPasses(120, 80, -4, 1000, 0));
  assert(!radioWindowPasses(120, 80, -4, 1000, 1, 15001)); // Do not round a rate below 8 packets/s into a pass.
  assert(radioWindowPasses(121, 80, -4, 1000, 1, 15001));
  {
    Rig rig;
    const std::array<float, 3> bias{1.2F, -0.6F, 0.3F};
    rig.input.sensors.rawGyro = bias;
    rig.hardware();
    assert(rig.calibrationSaves == 1 && rig.test.check(Check::GyroCalibration).result == Result::Pass);
    for (size_t i = 0; i < 3; ++i) {
      assert(std::fabs(rig.savedBias[i] - bias[i]) < 0.0001F);
      assert(std::fabs(rig.input.sensors.gyro[i]) < 0.0001F);
    }
    rig.hardware(); // A rerun measures raw bias, rather than subtracting the saved correction twice.
    assert(rig.calibrationSaves == 2);
    for (size_t i = 0; i < 3; ++i) assert(std::fabs(rig.savedBias[i] - bias[i]) < 0.0001F);
    rig.saveCalibration = false;
    rig.input.sensors.rawGyro = {2, 1, 0};
    rig.hardware();
    assert(rig.calibrationSaves == 2 && rig.test.check(Check::GyroCalibration).result == Result::Fail);
    for (size_t i = 0; i < 3; ++i) assert(std::fabs(rig.savedBias[i] - bias[i]) < 0.0001F);
  }
  {
    Rig rig;
    rig.start(); rig.advance(4000);
    assert(rig.calibrationSaves == 0);
    rig.input.sensors.acceleration = {1, 0, 0}; rig.tick(); // Tilt at constant 1 g restarts collection.
    assert(rig.test.gyroCalibrationSamples == 0);
    rig.advance(4000);
    assert(rig.calibrationSaves == 0);
    rig.now += 200; rig.tick(); // A sampling interruption also restarts the window.
    assert(rig.test.gyroCalibrationSamples == 1);
    rig.advance(5100);
    assert(rig.calibrationSaves == 1);
  }
  {
    Rig rig;
    rig.start(); rig.input.sensors.rawGyro = {6, 0, 0}; rig.advance(30100);
    assert(rig.calibrationSaves == 0 && rig.test.check(Check::GyroCalibration).result == Result::Fail);
    assert(rig.savedBias == (std::array<float, 3>{}));
    rig.gps = false;
    rig.advance(90000);
    assert(rig.test.check(Check::Gnss).result == Result::Pending); // Calibration time does not consume GNSS acquisition.
    rig.advance(31000);
    assert(rig.test.check(Check::Gnss).result == Result::Fail);
  }
  {
    Rig rig;
    rig.start(); rig.advance(4000); rig.test.cancel(rig.now);
    rig.advance(2000);
    assert(rig.calibrationSaves == 0 && rig.test.finished());
  }
  {
    Rig rig;
    rig.start(); rig.advance(5020);
    assert(rig.test.phase == Phase::Hardware);
    rig.advance(4980);
    assert(rig.test.check(Check::Battery).result == Result::Pending);
    rig.tick();
    assert(rig.test.check(Check::Battery).result == Result::Pass);
    assert(rig.test.phase == Phase::Telemetry && rig.test.awaitingConfirmation);
  }
  for (float voltage : {3.0F, 4.35F, 2.99F, 4.36F, std::numeric_limits<float>::quiet_NaN()}) {
    Rig rig;
    rig.input.batteryVoltage = voltage;
    rig.start(); rig.advance(5020);
    assert(rig.test.phase == Phase::Hardware);
    rig.input.batteryVoltage = 3.8F; // A later valid reading cannot hide an invalid first sample.
    rig.advance(5000);
    const auto expected = voltage >= 3.0F && voltage <= 4.35F ? Result::Pass : Result::Fail;
    assert(rig.test.check(Check::Battery).result == expected);
  }
  for (bool usb : {false, true}) {
    Rig rig;
    rig.start(); rig.advance(7000);
    assert(rig.test.phase == Phase::Hardware);
    rig.input.usbConnected = usb;
    if (!usb) rig.input.batteryVoltage = 4.5F;
    rig.tick(); // Even a transient USB connection or invalid voltage fails the window.
    rig.input.usbConnected = false;
    rig.input.batteryVoltage = 3.8F;
    rig.advance(5000);
    assert(rig.test.check(Check::Battery).result == Result::Fail);
  }
  for (bool storageFault : {false, true}) {
    Rig rig;
    rig.start();
    rig.test.phase = Phase::Usb;
    rig.test.awaitingConfirmation = true;
    rig.input.usbConnected = false;
    rig.input.usbReadCount = 10; // Reads from before this test must not count.
    rig.tick(); // Release the entry gate before pressing Start.
    rig.tap(A);
    rig.input.usbConnected = true; rig.tick();
    assert(rig.input.usbStorageReady && !rig.test.finished());
    rig.input.usbStorageFault = storageFault;
    rig.advance(30100);
    assert(rig.test.check(Check::Usb).result == Result::Fail);
  }
  {
    Rig rig;
    rig.start(); rig.test.phase = Phase::Usb; rig.test.awaitingConfirmation = true;
    rig.input.usbConnected = true;
    rig.input.usbReadCount = 10;
    rig.tick();
    assert(!rig.input.usbStorageReady); // An early connection cannot start the gated check.
    rig.tap(A); // Already connected at Start: share without an unnecessary unplug/reconnect cycle.
    assert(rig.input.usbStorageReady && !rig.test.finished()); // Old reads cannot pass.
    ++rig.input.usbReadCount; rig.tick();
    assert(rig.test.check(Check::Usb).result == Result::Pass);
  }
  std::cout << "Three-phase self-test and all manual confirmation gates passed\n";
}
