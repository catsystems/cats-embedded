/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "hmi_controller.hpp"

#include "hmi/location_qr.hpp"

#include <algorithm>
#include <cmath>
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
  startupStartedMs_ = nowMs_;
  lastStartupFrame_ = UINT32_MAX;
  screen_ = Screen::Logo;
  testingState_ = TestingState::Disclaimer;
  calibrationState_ = CalibrationState::Idle;
  menuSelection_ = 0;
  settingsPage_ = 0;
  settingsSelection_ = -1;
  dataSelection_ = 0;
  testingSelection_ = 0;
  dataSubview_ = DataSubview::List;
  logScrollOffset_ = 0;
  recoveryLocations_ = {};
  clearQr();
  liveDownrange_ = false;
  keyboardActive_ = false;
  keyboardSelection_ = 0;
  keyboardUppercase_ = false;
  usbPreviouslyConnected_ = false;
  automaticUsbSharePending_ = false;
  previousRecorderState_ = logs_.recorderStatus().state;
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
  ingestTelemetry();
  updateAutomaticUsbStorage();

  for (size_t i = 0; i < input_.held.size(); ++i) {
    if (input_.held[i] && !previousInput_.held[i]) {
      heldSince_[i] = nowMs_;
      lastRepeat_[i] = nowMs_;
    } else if (!input_.held[i]) {
      heldSince_[i] = 0;
      lastRepeat_[i] = 0;
    }
  }

  const uint64_t startupDurationMs =
      config_.config().startupAnimation ? StartupIntro::kDurationMs : StartupIntro::kStaticLogoDurationMs;
  if (screen_ == Screen::Logo && nowMs_ - startupStartedMs_ >= startupDurationMs) {
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
      if (qrView_ == "none" && config_.config().dualReceiver &&
          (pressed(HmiButton::Up) || pressed(HmiButton::Down))) {
        selectedRecoveryLink_ = static_cast<int8_t>(1 - selectedRecoveryLink_);
      }
      if (pressed(HmiButton::Right)) {
        if (qrView_ == "none") {
          if (config_.config().dualReceiver) {
            (void)showRecoveryLocation(static_cast<size_t>(selectedRecoveryLink_));
          } else {
            const auto nav = navigation_.snapshot();
            (void)showQr("recovery_fused", nav.rocketLatitude, nav.rocketLongitude);
          }
        } else if (config_.config().dualReceiver) {
          const size_t other = qrView_ == "recovery_link_1" ? 1U : 0U;
          if (showRecoveryLocation(other)) selectedRecoveryLink_ = static_cast<int8_t>(other);
        }
      }
      if (pressed(HmiButton::Left) && qrView_ != "none") {
        clearQr();
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
    case Screen::UsbStorage: {
      const std::string storageState = device_.snapshot().usbStorageState;
      if (usbStorageSession_ && storageState == "firmware") {
        usbStorageSession_ = false;
        emit("usb_storage_reclaimed");
        enter(Screen::Settings);
        settingsPage_ = 0;
        settingsSelection_ = 2;
      } else if (usbStorageSession_ && storageState == "fault") {
        usbStorageSession_ = false;
        usbStorageMessage_ = "Storage could not be remounted.";
      } else if (usbStorageSession_ && storageState == "host" && pressed(HmiButton::Back)) {
        usbStorageSession_ = false;
        enter(Screen::Settings);
        settingsPage_ = 0;
        settingsSelection_ = 2;
      } else if (usbStorageSession_ &&
                 ((storageState == "host" && pressed(HmiButton::Ok)) ||
                  (storageState == "preparing" && pressed(HmiButton::Back)))) {
        device_.requestFirmwareStorage();
      } else if (!usbStorageSession_ && pressed(HmiButton::Back)) {
        enter(Screen::Settings);
        settingsPage_ = 0;
        settingsSelection_ = 2;
      }
      break;
    }
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
  if (screen_ == Screen::Recovery) {
    const bool link1Valid = LocationQr::IsValid(recoveryLocations_[0].lastLatitude, recoveryLocations_[0].lastLongitude);
    const bool link2Valid = LocationQr::IsValid(recoveryLocations_[1].lastLatitude, recoveryLocations_[1].lastLongitude);
    selectedRecoveryLink_ = link1Valid ? 0 : (link2Valid ? 1 : 0);
  }
  if (screen_ == Screen::Data) {
    automaticUsbSharePending_ = false;
    if (device_.snapshot().usbStorageState != "firmware") {
      device_.requestFirmwareStorage();
    }
    dataSubview_ = DataSubview::List;
    const auto entries = logs_.listLogs();
    if (entries.empty()) dataSelection_ = 0;
    else dataSelection_ = std::min<int16_t>(dataSelection_, static_cast<int16_t>(entries.size() - 1U));
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

void HmiController::ingestTelemetry() {
  logs_.configure(config_.config().dualReceiver, config_.config().neverStopLogging);
  const auto first = link1_.snapshot();
  const auto second = link2_.snapshot();
  if (first.telemetry.updated) {
    if (first.telemetry.state > 2 && device_.snapshot().usbStorageState != "firmware") {
      device_.requestFirmwareStorage();
    }
    logs_.record(first.telemetry, 1);
    link1_.clearUpdates();
  }
  if (second.telemetry.updated) {
    if (second.telemetry.state > 2 && device_.snapshot().usbStorageState != "firmware") {
      device_.requestFirmwareStorage();
    }
    logs_.record(second.telemetry, 2);
    link2_.clearUpdates();
  }
}

void HmiController::updateAutomaticUsbStorage() {
  const DeviceStatusSnapshot device = device_.snapshot();
  const std::string recorderState = logs_.recorderStatus().state;
  if (!device.usb) {
    usbPreviouslyConnected_ = false;
    automaticUsbSharePending_ = false;
    previousRecorderState_ = recorderState;
    return;
  }
  if (!usbPreviouslyConnected_) {
    usbPreviouslyConnected_ = true;
    automaticUsbSharePending_ = recorderState == "idle";
  }
  if (previousRecorderState_ != "idle" && recorderState == "idle") {
    automaticUsbSharePending_ = true;
  }
  if (recorderState != "idle") {
    automaticUsbSharePending_ = false;
  }
  previousRecorderState_ = recorderState;

  if (automaticUsbSharePending_ && screen_ != Screen::Data && device.usbStorageState == "firmware" &&
      device_.requestMassStorage()) {
    automaticUsbSharePending_ = false;
    emit("usb_storage_shared_automatically");
  }
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
  const int16_t maxIndex = std::max<int16_t>(0, static_cast<int16_t>(logs.size()) - 1);
  if (dataSubview_ == DataSubview::Details) {
    if (pressed(HmiButton::Back) || (pressed(HmiButton::Up) && qrView_ == "none")) {
      dataSubview_ = DataSubview::List;
      clearQr();
      return;
    }
    if (pressed(HmiButton::Down) && qrView_ == "none") {
      dataSubview_ = DataSubview::Options;
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
  if (dataSubview_ == DataSubview::Options) {
    if (pressed(HmiButton::Back) || pressed(HmiButton::Up)) dataSubview_ = DataSubview::Details;
    else if (pressed(HmiButton::Ok) && dataSelection_ >= 0 && dataSelection_ < static_cast<int16_t>(logs.size())) {
      dataSubview_ = logs[static_cast<size_t>(dataSelection_)].active ? DataSubview::ConfirmFinalize
                                                                     : DataSubview::ConfirmDelete;
    }
    return;
  }
  if (dataSubview_ == DataSubview::ConfirmFinalize || dataSubview_ == DataSubview::ConfirmDelete) {
    if (pressed(HmiButton::Back)) {
      dataSubview_ = DataSubview::Options;
      return;
    }
    if (pressed(HmiButton::Ok)) {
      const bool deleting = dataSubview_ == DataSubview::ConfirmDelete;
      if (deleting && device_.snapshot().usbStorageState == "host") {
        dataMessageTitle_ = "USB Connected";
        dataMessageText_ = "Disconnect USB before deleting.";
        dataSubview_ = DataSubview::Message;
      } else {
        const bool success = deleting ? logs_.remove(logs[static_cast<size_t>(dataSelection_)].name) : logs_.finalize();
        if (success) {
          const auto updated = logs_.listLogs();
          dataSelection_ = updated.empty() ? 0 : std::min<int16_t>(dataSelection_, static_cast<int16_t>(updated.size() - 1U));
          logScrollOffset_ = std::min<size_t>(logScrollOffset_, static_cast<size_t>(dataSelection_));
          dataSubview_ = DataSubview::List;
        } else {
          dataMessageTitle_ = deleting ? "Delete Failed" : "Finalize Failed";
          dataMessageText_ = deleting ? "Log was not deleted" : "Log is still active";
          dataSubview_ = DataSubview::Message;
        }
      }
    }
    return;
  }
  if (dataSubview_ == DataSubview::Message) {
    if (pressed(HmiButton::Back)) dataSubview_ = DataSubview::Options;
    return;
  }
  if (pressed(HmiButton::Down) && dataSelection_ < maxIndex) {
    ++dataSelection_;
    if (static_cast<size_t>(dataSelection_) >= logScrollOffset_ + 11U) {
      logScrollOffset_ = static_cast<size_t>(dataSelection_) - 10U;
    }
  }
  if (pressed(HmiButton::Up) && dataSelection_ > 0) {
    --dataSelection_;
    if (static_cast<size_t>(dataSelection_) < logScrollOffset_) {
      logScrollOffset_ = static_cast<size_t>(dataSelection_);
    }
  }
  if (pressed(HmiButton::Ok) && !logs.empty()) {
    dataSubview_ = DataSubview::Details;
    clearQr();
    emit("flight_statistics", 0, dataSelection_);
  }
  if (pressed(HmiButton::Back)) {
    dataSelection_ = 0;
    logScrollOffset_ = 0;
    automaticUsbSharePending_ = device_.snapshot().usb;
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
        device_.requestFirmwareStorage();
        config_.save();
        automaticUsbSharePending_ = device_.snapshot().usb;
        emit("configuration_saved");
      }
      break;
  }
  (void)nowMs;
}

void HmiController::settingsStep(uint64_t nowMs) {
  const int16_t counts[3] = {4, 4, 3};
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
    } else if (settingsPage_ == 2 && settingsSelection_ == 2) {
      if (increment) config_.config().startupAnimation = true;
      if (decrement) config_.config().startupAnimation = false;
    }
    if (pressed(HmiButton::Ok) && settingsPage_ == 0 && settingsSelection_ == 2) {
      usbStorageMessage_.clear();
      const DeviceStatusSnapshot device = device_.snapshot();
      if (!device.usb) {
        usbStorageMessage_ = "Connect USB cable first.";
      } else if (logs_.recorderStatus().state != "idle") {
        usbStorageMessage_ = "Finalize the active log first.";
      } else if (device.usbStorageState == "host" || device.usbStorageState == "preparing") {
        usbStorageSession_ = true;
      } else if (!device_.requestMassStorage()) {
        usbStorageMessage_ = "USB storage is unavailable.";
      } else {
        usbStorageSession_ = true;
        emit("usb_storage_shared");
      }
      enter(Screen::UsbStorage);
      return;
    }
    if (pressed(HmiButton::Ok) && settingsPage_ == 0 && settingsSelection_ == 3) {
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
      device_.requestFirmwareStorage();
      config_.save();
      automaticUsbSharePending_ = device_.snapshot().usb;
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

void HmiController::render() {
  if (screen_ == Screen::Logo) {
    const uint64_t startupDurationMs =
        config_.config().startupAnimation ? StartupIntro::kDurationMs : StartupIntro::kStaticLogoDurationMs;
    const auto elapsedMs =
        static_cast<uint32_t>(std::min<uint64_t>(nowMs_ - startupStartedMs_, startupDurationMs));
    const uint32_t frame = config_.config().startupAnimation ? elapsedMs / StartupIntro::kFrameIntervalMs : 0U;
    if (frame == lastStartupFrame_) {
      return;
    }
    lastStartupFrame_ = frame;
  }
  renderer_.render(snapshot());
}

std::string HmiController::screenName() const {
  static constexpr const char* names[] = {"logo", "menu", "live", "recovery", "testing", "data", "sensors",
                                           "settings", "bootloader", "usb_storage"};
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

std::string HmiController::dataSubviewName() const {
  static constexpr const char* names[] = {"list", "details", "options", "confirm_finalize", "confirm_delete", "message"};
  return names[static_cast<size_t>(dataSubview_)];
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
  result.dataStatistics = dataSubview_ == DataSubview::Details;
  result.currentDataSubview = dataSubviewName();
  result.logScrollOffset = logScrollOffset_;
  result.dataMessageTitle = dataMessageTitle_;
  result.dataMessageText = dataMessageText_;
  result.usbStorageMessage = usbStorageMessage_;
  result.qrView = qrView_;
  result.qrUrl = qrUrl_;
  result.recoveryLocations = recoveryLocations_;
  result.virtualTimeMs = nowMs_;
  result.configuration = config_.config();
  const uint64_t startupDurationMs =
      result.configuration.startupAnimation ? StartupIntro::kDurationMs : StartupIntro::kStaticLogoDurationMs;
  const auto startupElapsed =
      static_cast<uint32_t>(std::min<uint64_t>(nowMs_ - startupStartedMs_, startupDurationMs));
  result.startupElapsedMs = startupElapsed;
  result.startupPhase = screen_ != Screen::Logo
                            ? StartupIntro::PhaseName(StartupIntro::Phase::kComplete)
                            : (result.configuration.startupAnimation
                                   ? StartupIntro::PhaseName(StartupIntro::PhaseAt(startupElapsed))
                                   : "static_logo");
  result.links[0] = link1_.snapshot();
  result.links[1] = link2_.snapshot();
  result.navigation = navigation_.snapshot();
  result.device = device_.snapshot();
  result.recorder = logs_.recorderStatus();
  result.device.logging = result.recorder.state == "recording";
  result.device.recorderFault = result.recorder.state == "fault";
  result.logs = logs_.listLogs();
  if (dataSubview_ != DataSubview::List && dataSelection_ >= 0 && dataSelection_ < static_cast<int16_t>(result.logs.size())) {
    result.flightStatistics[0] = logs_.statistics(result.logs[static_cast<size_t>(dataSelection_)], 1);
    result.flightStatistics[1] = logs_.statistics(result.logs[static_cast<size_t>(dataSelection_)], 2);
    const bool anyParticipant = result.flightStatistics[0].participant || result.flightStatistics[1].participant;
    const bool complete = anyParticipant &&
        (!result.flightStatistics[0].participant || result.flightStatistics[0].complete) &&
        (!result.flightStatistics[1].participant || result.flightStatistics[1].complete);
    result.selectedLogHealth = result.logs[static_cast<size_t>(dataSelection_)].active ? "active" :
                               (complete ? "complete" : "incomplete");
  }
  result.selectedRecoveryLink = result.configuration.dualReceiver ? selectedRecoveryLink_ : -1;
  const float targetLatitude = result.configuration.dualReceiver
                                   ? recoveryLocations_[static_cast<size_t>(selectedRecoveryLink_)].lastLatitude
                                   : result.navigation.rocketLatitude;
  const float targetLongitude = result.configuration.dualReceiver
                                    ? recoveryLocations_[static_cast<size_t>(selectedRecoveryLink_)].lastLongitude
                                    : result.navigation.rocketLongitude;
  result.recoverySolution.latitude = targetLatitude;
  result.recoverySolution.longitude = targetLongitude;
  result.recoverySolution.valid = LocationQr::IsValid(targetLatitude, targetLongitude) &&
                                  LocationQr::IsValid(result.navigation.homeLatitude, result.navigation.homeLongitude);
  if (result.recoverySolution.valid) {
    constexpr float kRadians = 0.017453292519943295F;
    constexpr float kEarthRadius = 6378100.0F;
    const float dy = (targetLatitude - result.navigation.homeLatitude) * kRadians * kEarthRadius;
    const float dx = (targetLongitude - result.navigation.homeLongitude) * kRadians *
                     std::cos(result.navigation.homeLatitude * kRadians) * kEarthRadius;
    result.recoverySolution.distanceM = std::sqrt(dx * dx + dy * dy);
    result.recoverySolution.azimuthRad = std::atan2(dx, dy);
  }
  result.actions = actions_;
  result.framebufferRevision = renderer_.revision();
  return result;
}
