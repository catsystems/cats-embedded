// Contract tests for the production state machine, including manual start gates.
#include "self_test.hpp"
#include <cassert>
#include <cstring>
#include <iostream>

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
  Radio configuration{Radio::Restore};
  std::array<unsigned, 6> requests{};

  Rig() {
    input.linksReady = input.usbConnected = true;
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
    if (packets && sinceStart % 100 == 0) {
      for (auto& link : input.links) {
        ++link.radio.packets;
        ++link.radio.infoSamples;
        link.radio.lqSum += lq;
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
    start(); advance(3000);
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
      input.sensors.gyro = {};
      input.sensors.gyro[axis] = 30;
      input.sensors.magnetic = {sign * 1000, sign * 800, sign * 600};
      tick();
    }
  }
};

int main() {
  {
    Rig rig;
    rig.hardware();
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
    for (unsigned bit = 1; bit < 7; ++bit) rig.tap(1U << bit);
    rig.gate(Phase::Display);
    for (uint8_t pattern = 1; pattern <= 4; ++pattern) {
      rig.advance(2000);
      assert(rig.test.displayPattern(rig.now) == pattern);
    }
    rig.advance(2100);
    rig.press(A); // Keep the approval button held through the next prompt.
    assert(rig.test.phase == Phase::Leds && rig.test.awaitingConfirmation);
    const auto requests = rig.requests;
    rig.advance(15000);
    assert(rig.test.awaitingConfirmation && rig.requests == requests);
    rig.release(A);
    rig.gate(Phase::Leds);
    assert(rig.configuration == Radio::Receiver1Only);
    rig.advance(6000);
    assert(rig.configuration == Radio::Receiver2Only);
    rig.advance(6100);
    rig.tap(A);
    rig.advance(2600);
    assert(rig.test.phase == Phase::Unplug && rig.test.awaitingConfirmation);
    rig.input.usbConnected = false;
    rig.advance(10000);
    assert(rig.test.phase == Phase::Unplug && rig.test.check(Check::Battery).result == Result::Pending);
    rig.input.usbConnected = true;
    rig.gate(Phase::Unplug);
    rig.input.usbConnected = false;
    rig.advance(5100);
    assert(rig.test.phase == Phase::Reconnect && rig.test.awaitingConfirmation);
    rig.input.usbConnected = true; // An early reconnection cannot complete the gated test.
    rig.gate(Phase::Reconnect);
    rig.advance(1000);
    assert(!rig.test.finished());
    rig.input.usbConnected = false; rig.tick();
    rig.input.usbConnected = true; rig.tick();
    assert(rig.test.finished() && rig.test.overall() == Result::Pass);
    for (const auto& result : rig.test.results) assert(result.result == Result::Pass);
  }
  {
    Rig rig;
    rig.versions = false;
    rig.gps = false;
    for (auto& link : rig.input.links) { std::strcpy(link.version, "cached"); link.versionReplies = 1; }
    rig.input.links[1].fixCount = 10;
    rig.start(); rig.advance(121000);
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
    rig.start(); rig.input.sensors.readError = true; rig.advance(200);
    rig.input.sensors.readError = false; rig.advance(3000);
    rig.test.cancel(rig.now);
    assert(rig.test.finished() && rig.test.check(Check::Motion).result == Result::Fail);
  }
  std::cout << "Three-phase self-test and all manual confirmation gates passed\n";
}
