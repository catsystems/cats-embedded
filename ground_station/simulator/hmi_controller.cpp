/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "hmi_controller.hpp"

#include "hmi/location_qr.hpp"

#include <algorithm>
#include <cstdio>

namespace {
size_t buttonIndex(HmiButton button) { return static_cast<size_t>(button); }
}  // namespace

HmiController::HmiController(IHmiRenderer& renderer, ITelemetryLink& link1, ITelemetryLink& link2,
                             INavigation& navigation, ILogStore& logs, IConfigStore& config, IDeviceStatus& device,
                             IClock& clock)
    : renderer_(renderer),
      link1_(link1),
      link2_(link2),
      navigation_(navigation),
      logs_(logs),
      config_(config),
      device_(device),
      clock_(clock) {}

void HmiController::start() {
  nowMs_ = clock_.nowMs();
  screen_ = Screen::Logo;
  testingState_ = TestingState::Disclaimer;
  calibrationState_ = CalibrationState::Idle;
  menuSelection_ = 0;
  settingsPage_ = 0;
  settingsSelection_ = -1;
  dataSelection_ = 0;
  testingSelection_ = 0;
  dataStatistics_ = false;
  recoveryLocations_ = {};
  clearQr();
  liveDownrange_ = false;
  keyboardActive_ = false;
  keyboardSelection_ = 0;
  keyboardUppercase_ = false;
  input_ = {};
  previousInput_ = {};
  heldSince_.fill(0);
  lastRepeat_.fill(0);
  actions_.clear();
  renderer_.begin();
  render();
}

void HmiController::step(const HmiInput& input, uint64_t nowMs) {
  nowMs_ = nowMs;
  previousInput_ = input_;
  input_ = input;

  updateRecoveryLocations();

  for (size_t i = 0; i < input_.held.size(); ++i) {
    if (input_.held[i] && !previousInput_.held[i]) {
      heldSince_[i] = nowMs_;
      lastRepeat_[i] = nowMs_;
    } else if (!input_.held[i]) {
      heldSince_[i] = 0;
      lastRepeat_[i] = 0;
    }
  }

  if (screen_ == Screen::Logo && nowMs_ >= kBootDurationMs) {
    enter(Screen::Menu);
  }

  switch (screen_) {
    case Screen::Menu:
      menuStep(nowMs_);
      break;
    case Screen::Live:
      liveStep(nowMs_);
      break;
    case Screen::Testing:
      testingStep(nowMs_);
      break;
    case Screen::Data:
      dataStep(nowMs_);
      break;
    case Screen::Sensors:
      sensorsStep(nowMs_);
      break;
    case Screen::Settings:
      settingsStep(nowMs_);
      break;
    case Screen::Recovery:
      if (pressed(HmiButton::Right)) {
        if (qrView_ == "none") {
          if (!showRecoveryLocation(0)) {
            (void)showRecoveryLocation(1);
          }
        } else if (qrView_ == "recovery_link_1") {
          (void)showRecoveryLocation(1);
        }
      }
      if (pressed(HmiButton::Left) && qrView_ != "none") {
        if (qrView_ == "recovery_link_2" &&
            LocationQr::IsValid(recoveryLocations_[0].lastLatitude, recoveryLocations_[0].lastLongitude)) {
          (void)showRecoveryLocation(0);
        } else {
          clearQr();
        }
      }
      if (qrView_ == "recovery_link_1") {
        (void)showRecoveryLocation(0);
      } else if (qrView_ == "recovery_link_2") {
        (void)showRecoveryLocation(1);
      }
      if (pressed(HmiButton::Back)) {
        enter(Screen::Menu);
      }
      break;
    case Screen::Bootloader:
      if (pressed(HmiButton::Back)) {
        enter(Screen::Settings);
      }
      break;
    case Screen::Logo:
      break;
  }

  render();
  previousInput_ = input_;
}

bool HmiController::pressed(HmiButton button) const {
  const size_t index = buttonIndex(button);
  return input_.pressed[index] || (input_.held[index] && !previousInput_.held[index]);
}

bool HmiController::repeated(HmiButton button, uint64_t nowMs) {
  const size_t index = buttonIndex(button);
  if (!input_.held[index] || heldSince_[index] == 0 || nowMs - heldSince_[index] < kLongPressMs) {
    return false;
  }
  if (nowMs - lastRepeat_[index] < 100) {
    return false;
  }
  lastRepeat_[index] = nowMs;
  return true;
}

void HmiController::enter(Screen screen) {
  screen_ = screen;
  clearQr();
  if (screen_ == Screen::Testing) {
    testingState_ = TestingState::Disclaimer;
    testingSelection_ = 0;
  }
  if (screen_ == Screen::Sensors) {
    calibrationState_ = CalibrationState::Idle;
  }
  if (screen_ == Screen::Settings) {
    settingsPage_ = 0;
    settingsSelection_ = -1;
    keyboardActive_ = false;
  }
}

void HmiController::menuStep(uint64_t nowMs) {
  (void)nowMs;
  const int16_t oldSelection = menuSelection_;
  if (pressed(HmiButton::Right) && menuSelection_ % 3 < 2) {
    ++menuSelection_;
  }
  if (pressed(HmiButton::Left) && menuSelection_ % 3 > 0) {
    --menuSelection_;
  }
  if (pressed(HmiButton::Down) && menuSelection_ < 3) {
    menuSelection_ += 3;
  }
  if (pressed(HmiButton::Up) && menuSelection_ > 2) {
    menuSelection_ -= 3;
  }
  if (oldSelection != menuSelection_) {
    emit("menu_selection", 0, menuSelection_);
  }
  if (pressed(HmiButton::Ok) || pressed(HmiButton::Center)) {
    enter(static_cast<Screen>(static_cast<uint8_t>(Screen::Live) + static_cast<uint8_t>(menuSelection_)));
  }
}

void HmiController::liveStep(uint64_t nowMs) {
  const auto link1 = link1_.snapshot();
  const auto link2 = link2_.snapshot();
  if (link1.telemetry.updated && link1.info.updated) {
    if (link1.telemetry.state > 2 && (config_.config().neverStopLogging || link1.telemetry.state < 7)) {
      logs_.record(link1.telemetry, 1);
    }
    link1_.clearUpdates();
  }
  if (link2.telemetry.updated && link2.info.updated) {
    if (link2.telemetry.state > 2 && (config_.config().neverStopLogging || link2.telemetry.state < 7)) {
      logs_.record(link2.telemetry, 2);
    }
    link2_.clearUpdates();
  }
  if (pressed(HmiButton::Left)) {
    liveDownrange_ = false;
    emit("live_view", 0, 0, "gnss");
  }
  if (pressed(HmiButton::Right)) {
    liveDownrange_ = true;
    emit("live_view", 0, 0, "downrange");
  }
  if (pressed(HmiButton::Back)) {
    enter(Screen::Menu);
  }
  (void)nowMs;
}

void HmiController::testingStep(uint64_t nowMs) {
  if (testingState_ == TestingState::ConfirmEvent) {
    if (pressed(HmiButton::Ok)) {
      link1_.triggerEvent(static_cast<uint8_t>(testingSelection_ + 1));
      emit("event_triggered", 1, testingSelection_ + 1);
      testingState_ = TestingState::Started;
    }
    if (pressed(HmiButton::Back)) {
      testingState_ = TestingState::Started;
    }
    return;
  }
  if (pressed(HmiButton::Back)) {
    if (testingState_ == TestingState::Waiting || testingState_ == TestingState::Started ||
        testingState_ == TestingState::ConfirmEvent) {
      link1_.exitTesting();
      link2_.enable();
      emit("testing_exit", 1);
    }
    enter(Screen::Menu);
    return;
  }
  switch (testingState_) {
    case TestingState::Disclaimer:
      if (pressed(HmiButton::Ok)) {
        if (link1_.snapshot().connected) {
          testingState_ = TestingState::CanStart;
          emit("testing_connection", 1, 1);
        } else {
          testingState_ = TestingState::CannotStart;
          emit("testing_connection", 1, 0);
        }
      }
      break;
    case TestingState::CanStart:
      if (pressed(HmiButton::Ok)) {
        link2_.disable();
        link1_.enterTesting();
        testingStartedMs_ = nowMs;
        testingState_ = TestingState::Waiting;
        emit("link_disabled", 2);
        emit("testing_enter", 1);
      }
      break;
    case TestingState::Waiting:
      if (link1_.snapshot().telemetry.testingMode && link1_.snapshot().telemetry.state == 1) {
        testingState_ = TestingState::Started;
        emit("testing_started", 1);
      } else if (nowMs - testingStartedMs_ > kTestingTimeoutMs) {
        link1_.disable();
        testingState_ = TestingState::Failed;
        emit("testing_timeout", 1);
      }
      break;
    case TestingState::Started:
      if (pressed(HmiButton::Up) && testingSelection_ % 4 > 0) {
        --testingSelection_;
      }
      if (pressed(HmiButton::Down) && testingSelection_ % 4 < 3) {
        ++testingSelection_;
      }
      if (pressed(HmiButton::Left) && testingSelection_ > 3) {
        --testingSelection_;
      }
      if (pressed(HmiButton::Right) && testingSelection_ < 4) {
        ++testingSelection_;
      }
      if (pressed(HmiButton::Ok)) {
        testingState_ = TestingState::ConfirmEvent;
      }
      if (!link1_.snapshot().connected || link1_.snapshot().telemetry.state != 1) {
        testingState_ = TestingState::Failed;
        link1_.exitTesting();
        emit("testing_connection_lost", 1);
      }
      break;
    case TestingState::CannotStart:
    case TestingState::Failed:
      break;
    case TestingState::ConfirmEvent:
      break;
  }
}

void HmiController::dataStep(uint64_t nowMs) {
  const auto logs = logs_.listLogs();
  const int16_t maxIndex = std::max<int16_t>(0, std::min<int16_t>(11, static_cast<int16_t>(logs.size())) - 1);
  if (dataStatistics_) {
    if (pressed(HmiButton::Back)) {
      dataStatistics_ = false;
      clearQr();
      return;
    }
    if ((pressed(HmiButton::Left) || pressed(HmiButton::Right)) && dataSelection_ >= 0 &&
        dataSelection_ < static_cast<int16_t>(logs.size())) {
      const auto& log = logs[static_cast<size_t>(dataSelection_)];
      const auto stats1 = logs_.statistics(log, 1);
      const auto stats2 = logs_.statistics(log, 2);
      const bool link1Valid = LocationQr::IsValid(stats1.lastLatitude, stats1.lastLongitude);
      const bool link2Valid = LocationQr::IsValid(stats2.lastLatitude, stats2.lastLongitude);

      if (pressed(HmiButton::Right)) {
        if (qrView_ == "none") {
          if (link1Valid) {
            (void)showQr("log_link_1", stats1.lastLatitude, stats1.lastLongitude);
          } else if (link2Valid) {
            (void)showQr("log_link_2", stats2.lastLatitude, stats2.lastLongitude);
          }
        } else if (qrView_ == "log_link_1" && link2Valid) {
          (void)showQr("log_link_2", stats2.lastLatitude, stats2.lastLongitude);
        }
      } else if (pressed(HmiButton::Left)) {
        if (qrView_ == "log_link_2" && link1Valid) {
          (void)showQr("log_link_1", stats1.lastLatitude, stats1.lastLongitude);
        } else if (qrView_ != "none") {
          clearQr();
        }
      }
    }
    return;
  }
  if (pressed(HmiButton::Down) && dataSelection_ < maxIndex) {
    ++dataSelection_;
  }
  if (pressed(HmiButton::Up) && dataSelection_ > 0) {
    --dataSelection_;
  }
  if (pressed(HmiButton::Ok) && !logs.empty()) {
    dataStatistics_ = true;
    clearQr();
    emit("flight_statistics", 0, dataSelection_);
  }
  if (pressed(HmiButton::Back)) {
    dataSelection_ = 0;
    enter(Screen::Menu);
  }
  (void)nowMs;
}

void HmiController::updateRecoveryLocations() {
  const std::array<LinkSnapshot, 2> links = {link1_.snapshot(), link2_.snapshot()};
  for (size_t index = 0; index < links.size(); ++index) {
    const TelemetrySample &telemetry = links[index].telemetry;
    if (LocationQr::IsValid(telemetry.latitude, telemetry.longitude)) {
      recoveryLocations_[index].lastLatitude = telemetry.latitude;
      recoveryLocations_[index].lastLongitude = telemetry.longitude;
    }
  }
}

bool HmiController::showRecoveryLocation(size_t linkIndex) {
  if (linkIndex >= recoveryLocations_.size()) {
    return false;
  }
  const FlightStatisticsSnapshot &location = recoveryLocations_[linkIndex];
  const char *view = linkIndex == 0 ? "recovery_link_1" : "recovery_link_2";
  return showQr(view, location.lastLatitude, location.lastLongitude);
}

void HmiController::sensorsStep(uint64_t nowMs) {
  switch (calibrationState_) {
    case CalibrationState::Idle:
      if (pressed(HmiButton::Ok)) {
        calibrationState_ = CalibrationState::Prepare;
      }
      if (pressed(HmiButton::Back)) {
        enter(Screen::Menu);
      }
      break;
    case CalibrationState::Prepare:
      if (pressed(HmiButton::Ok)) {
        navigation_.startCalibration();
        calibrationState_ = CalibrationState::Calibrating;
        emit("calibration_started");
      }
      if (pressed(HmiButton::Back)) {
        calibrationState_ = CalibrationState::Idle;
      }
      break;
    case CalibrationState::Calibrating:
      if (pressed(HmiButton::Back)) {
        navigation_.cancelCalibration();
        calibrationState_ = CalibrationState::Idle;
        emit("calibration_cancelled");
      } else if (navigation_.snapshot().calibrationState == 3 ||
                 navigation_.snapshot().calibrationPercentage >= 100.0F) {
        calibrationState_ = CalibrationState::Concluded;
        emit("calibration_completed");
      }
      break;
    case CalibrationState::Concluded:
      if (pressed(HmiButton::Ok) || pressed(HmiButton::Back)) {
        calibrationState_ = CalibrationState::Idle;
        config_.save();
        emit("configuration_saved");
      }
      break;
  }
  (void)nowMs;
}

void HmiController::settingsStep(uint64_t nowMs) {
  const int16_t counts[3] = {3, 4, 2};
  if (keyboardActive_) {
    const bool moveRight = pressed(HmiButton::Right) || repeated(HmiButton::Right, nowMs);
    const bool moveLeft = pressed(HmiButton::Left) || repeated(HmiButton::Left, nowMs);
    const bool moveDown = pressed(HmiButton::Down) || repeated(HmiButton::Down, nowMs);
    const bool moveUp = pressed(HmiButton::Up) || repeated(HmiButton::Up, nowMs);

    if (moveRight && keyboardSelection_ != 9 && keyboardSelection_ != 19 && keyboardSelection_ != 28 &&
        keyboardSelection_ != 37) {
      ++keyboardSelection_;
    }
    if (moveLeft && keyboardSelection_ != -1 && keyboardSelection_ != 0 && keyboardSelection_ != 10 &&
        keyboardSelection_ != 20 && keyboardSelection_ != 29) {
      --keyboardSelection_;
    }
    if (moveDown) {
      if (keyboardSelection_ == -1) {
        keyboardSelection_ = 7;
      } else if (keyboardSelection_ < 15) {
        keyboardSelection_ += 10;
      } else if (keyboardSelection_ < 29) {
        keyboardSelection_ += 9;
      }
    }
    if (moveUp) {
      if (keyboardSelection_ > 9) {
        if (keyboardSelection_ < 25) {
          keyboardSelection_ -= 10;
        } else if (keyboardSelection_ < 38) {
          keyboardSelection_ -= 9;
        }
      } else {
        keyboardSelection_ = -1;
      }
    }
    if (pressed(HmiButton::Ok)) {
      const char alphabet[] = "1234567890QWERTYUIOPASDFGHJKL ZXCVBNM_";
      std::string* phrase = &config_.config().linkPhrase1;
      if (settingsSelection_ == 2) phrase = &config_.config().linkPhrase2;
      if (settingsSelection_ == 3) phrase = &config_.config().testingPhrase;

      if (keyboardSelection_ == -1) {
        if (!phrase->empty()) phrase->pop_back();
      } else if (keyboardSelection_ == 29) {
        keyboardUppercase_ = !keyboardUppercase_;
      } else if (keyboardSelection_ >= 0 && keyboardSelection_ < static_cast<int16_t>(sizeof(alphabet) - 1) &&
                 phrase->size() < kKeyboardMaxLength) {
        char selected = alphabet[keyboardSelection_];
        if (!keyboardUppercase_ && keyboardSelection_ > 9 && keyboardSelection_ != 37) {
          selected = static_cast<char>(selected + ('a' - 'A'));
        }
        phrase->push_back(selected);
      }
    }
    if (pressed(HmiButton::Back)) {
      keyboardActive_ = false;
      emit("settings_string_done");
    }
    return;
  }
  if (settingsSelection_ < 0) {
    if (pressed(HmiButton::Right) && settingsPage_ < 2) {
      ++settingsPage_;
    }
    if (pressed(HmiButton::Left) && settingsPage_ > 0) {
      --settingsPage_;
    }
  } else {
    const bool increment = pressed(HmiButton::Right) || repeated(HmiButton::Right, nowMs);
    const bool decrement = pressed(HmiButton::Left) || repeated(HmiButton::Left, nowMs);
    if (settingsPage_ == 0 && settingsSelection_ == 0) {
      if (increment) config_.config().neverStopLogging = true;
      if (decrement) config_.config().neverStopLogging = false;
    } else if (settingsPage_ == 1 && settingsSelection_ == 0) {
      if (increment) config_.config().dualReceiver = true;
      if (decrement) config_.config().dualReceiver = false;
    } else if (settingsPage_ == 1 && settingsSelection_ >= 1 && settingsSelection_ <= 3 && pressed(HmiButton::Ok)) {
      keyboardActive_ = true;
      keyboardSelection_ = 0;
      keyboardUppercase_ = false;
    } else if (settingsPage_ == 2 && settingsSelection_ == 0) {
      if (increment) config_.config().timeZoneOffset = std::min<int16_t>(12, config_.config().timeZoneOffset + 1);
      if (decrement) config_.config().timeZoneOffset = std::max<int16_t>(-12, config_.config().timeZoneOffset - 1);
    } else if (settingsPage_ == 2 && settingsSelection_ == 1) {
      if (pressed(HmiButton::Left) || pressed(HmiButton::Right)) config_.config().imperialUnits = !config_.config().imperialUnits;
    }
    if (pressed(HmiButton::Ok) && settingsPage_ == 0 && settingsSelection_ == 2) {
      enter(Screen::Bootloader);
      emit("bootloader_requested");
    }
  }
  if (pressed(HmiButton::Down) && settingsSelection_ < counts[settingsPage_] - 1) {
    ++settingsSelection_;
  }
  if (pressed(HmiButton::Up) && settingsSelection_ > -1) {
    --settingsSelection_;
  }
  if (pressed(HmiButton::Back)) {
    if (settingsSelection_ >= 0) {
      settingsSelection_ = -1;
    } else {
      link1_.setLinkPhrase(config_.config().linkPhrase1);
      link2_.setLinkPhrase(config_.config().dualReceiver ? config_.config().linkPhrase2 : config_.config().linkPhrase1);
      link1_.setTestingPhrase(config_.config().testingPhrase);
      config_.save();
      emit("configuration_saved");
      enter(Screen::Menu);
    }
  }
}

void HmiController::emit(const char* type, uint8_t link, int32_t value, const std::string& text) {
  actions_.push_back(PlatformAction{type, link, value, text});
}

bool HmiController::showQr(const char* view, float latitude, float longitude) {
  char url[LocationQr::kGoogleMapsUrlSize] = {};
  if (!LocationQr::BuildGoogleMapsUrl(latitude, longitude, url, sizeof(url))) {
    return false;
  }
  qrView_ = view;
  qrUrl_ = url;
  return true;
}

void HmiController::clearQr() {
  qrView_ = "none";
  qrUrl_.clear();
}

void HmiController::render() { renderer_.render(snapshot()); }

std::string HmiController::screenName() const {
  static constexpr const char* names[] = {"logo", "menu", "live", "recovery", "testing", "data", "sensors", "settings", "bootloader"};
  return names[static_cast<size_t>(screen_)];
}

std::string HmiController::testingName() const {
  static constexpr const char* names[] = {"disclaimer", "can_start", "cannot_start", "waiting", "failed", "started", "confirm_event"};
  return names[static_cast<size_t>(testingState_)];
}

std::string HmiController::calibrationName() const {
  static constexpr const char* names[] = {"idle", "prepare", "calibrating", "concluded"};
  return names[static_cast<size_t>(calibrationState_)];
}

HmiSnapshot HmiController::snapshot() const {
  HmiSnapshot result;
  result.screen = screenName();
  result.testingState = testingName();
  result.calibrationState = calibrationName();
  result.settingsState = keyboardActive_ ? "keyboard" : "list";
  result.inputState = "idle";
  result.liveView = liveDownrange_ ? "downrange" : "gnss";
  for (const bool held : input_.held) {
    if (held) {
      result.inputState = "held";
      break;
    }
  }
  result.menuSelection = menuSelection_;
  result.testingSelection = testingSelection_;
  result.keyboardSelection = keyboardSelection_;
  result.keyboardUppercase = keyboardUppercase_;
  result.settingsPage = settingsPage_;
  result.settingsSelection = settingsSelection_;
  result.dataSelection = dataSelection_;
  result.dataStatistics = dataStatistics_;
  result.qrView = qrView_;
  result.qrUrl = qrUrl_;
  result.recoveryLocations = recoveryLocations_;
  result.virtualTimeMs = nowMs_;
  result.configuration = config_.config();
  result.links[0] = link1_.snapshot();
  result.links[1] = link2_.snapshot();
  result.navigation = navigation_.snapshot();
  result.device = device_.snapshot();
  result.logs = logs_.listLogs();
  if (dataStatistics_ && dataSelection_ >= 0 && dataSelection_ < static_cast<int16_t>(result.logs.size())) {
    result.flightStatistics[0] = logs_.statistics(result.logs[static_cast<size_t>(dataSelection_)], 1);
    result.flightStatistics[1] = logs_.statistics(result.logs[static_cast<size_t>(dataSelection_)], 2);
  }
  result.actions = actions_;
  result.framebufferRevision = renderer_.revision();
  return result;
}
