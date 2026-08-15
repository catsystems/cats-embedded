#include "window_hmi_renderer.hpp"

#include "hmi/location_qr.hpp"

#include <algorithm>
#include <cstring>

// The production settings table is bound to this configuration object. The
// simulator synchronizes it from IConfigStore before every deterministic draw.
Config systemConfig{};

WindowHmiRenderer::WindowHmiRenderer(SimulatorDisplay& display, IClock& clock)
    : display_(display), window_(display, systemConfig, clock) {}

void WindowHmiRenderer::begin() { window_.begin(); }

void WindowHmiRenderer::syncConfiguration(const GsConfigSnapshot& source) {
  systemConfig.config.timeZoneOffset = source.timeZoneOffset;
  systemConfig.config.neverStopLogging = source.neverStopLogging;
  systemConfig.config.startupAnimation = source.startupAnimation;
  systemConfig.config.receiverMode = source.dualReceiver ? DUAL : SINGLE;
  systemConfig.config.unitSystem = source.imperialUnits ? UnitSystem::kImperial : UnitSystem::kMetric;

  const auto copyPhrase = [](char* destination, const std::string& value) {
    std::strncpy(destination, value.c_str(), kMaxPhraseLen);
    destination[kMaxPhraseLen] = '\0';
  };
  copyPhrase(systemConfig.config.linkPhrase1, source.linkPhrase1);
  copyPhrase(systemConfig.config.linkPhrase2, source.linkPhrase2);
  copyPhrase(systemConfig.config.testingPhrase, source.testingPhrase);
}

void WindowHmiRenderer::syncNavigation(const NavigationSnapshot& source) {
  navigation_.setPointA(source.homeLatitude, source.homeLongitude);
  navigation_.setPointB(source.rocketLatitude, source.rocketLongitude);
  navigation_.setOrientation(source.northRad, source.azimuthRad, source.elevationRad);
  navigation_.setDistance(source.distanceM);
  navigation_.setAcceleration(source.ax, source.ay, source.az);
  navigation_.setGyroscope(source.gx, source.gy, source.gz);
  navigation_.setMagnetometer(source.mx, source.my, source.mz);
  navigation_.setCalibrationPercentage(source.calibrationPercentage);
}

void WindowHmiRenderer::syncLink(const LinkSnapshot& source, size_t index) {
  telemetry_[index].set(std::min<uint8_t>(7, source.telemetry.state), source.telemetry.errors,
                        source.telemetry.altitudeM, source.telemetry.velocityMps, source.telemetry.latitude,
                        source.telemetry.longitude, source.telemetry.voltage, source.telemetry.pyroContinuity,
                        source.telemetry.testingMode);
  linkInfo_[index].set(source.info.linkQuality, source.info.rssi, source.info.snr);
}

void WindowHmiRenderer::syncStatistics(const FlightStatisticsSnapshot& source, size_t index) {
  statistics_[index].set(source.maxAltitudeM, source.timeToApogeeS, source.maxVelocityMps,
                         source.drogueDescentRateMps, source.mainDescentRateMps, source.lastLatitude,
                         source.lastLongitude, source.flightTimeS, source.maxAltitudeValid,
                         source.timeToApogeeValid, source.maxVelocityValid, source.drogueRateValid,
                         source.mainRateValid, source.lastLocationValid, source.flightTimeValid);
}

void WindowHmiRenderer::drawStatusBar(const DeviceStatusSnapshot& status) {
  window_.updateBar(status.batteryVoltage, status.usb, status.logging, status.gnss, status.clockValid,
                    status.freeStoragePercent, status.recorderFault);
}

void WindowHmiRenderer::render(const HmiSnapshot& state) {
  syncConfiguration(state.configuration);
  syncNavigation(state.navigation);
  syncLink(state.links[0], 0);
  syncLink(state.links[1], 1);
  syncStatistics(state.flightStatistics[0], 0);
  syncStatistics(state.flightStatistics[1], 1);

  // A complete draw from the same production Window methods keeps resets and
  // repeated scenario runs independent of prior framebuffer history.
  window_.begin();

  if (state.screen == "logo") {
    if (state.configuration.startupAnimation) {
      window_.drawStartupIntroFrame(state.startupElapsedMs);
    } else {
      window_.logo();
    }
    return;
  }
  if (state.screen == "bootloader") {
    window_.Bootloader();
    return;
  }

  window_.initBar();

  if (state.screen == "menu") {
    window_.initMenu(state.menuSelection);
  } else if (state.screen == "live") {
    window_.initLive();
    if (state.liveView == "downrange") {
      window_.UpdateLiveState(&telemetry_[0], &telemetry_[1], &navigation_, Window::LiveState::kShowDownRange);
    }
    for (size_t index = 0; index < state.links.size(); ++index) {
      if (state.links[index].connected) {
        window_.updateLive(&telemetry_[index], &navigation_, &linkInfo_[index], static_cast<int16_t>(index));
      }
    }
  } else if (state.screen == "recovery") {
    if (state.qrView == "recovery_link_1") {
      window_.showLocationQr(state.recoveryLocations[0].lastLatitude, state.recoveryLocations[0].lastLongitude,
                             "[Link 1] Last Location", true,
                             LocationQr::IsValid(state.recoveryLocations[1].lastLatitude,
                                                 state.recoveryLocations[1].lastLongitude));
    } else if (state.qrView == "recovery_link_2") {
      window_.showLocationQr(state.recoveryLocations[1].lastLatitude, state.recoveryLocations[1].lastLongitude,
                             "[Link 2] Last Location", true, false);
    } else if (state.qrView == "recovery_fused") {
      window_.showLocationQr(state.navigation.rocketLatitude, state.navigation.rocketLongitude, "Last Location", true,
                             false);
    } else {
      const bool hasLastLocation =
          LocationQr::IsValid(state.recoveryLocations[0].lastLatitude, state.recoveryLocations[0].lastLongitude) ||
          LocationQr::IsValid(state.recoveryLocations[1].lastLatitude, state.recoveryLocations[1].lastLongitude);
      window_.initRecovery(hasLastLocation);
      if (state.configuration.dualReceiver) {
        const EarthPoint3D target(state.recoverySolution.latitude, state.recoverySolution.longitude);
        window_.updateRecoveryTarget(&navigation_, target,
                                     LocationQr::IsValid(target.lat, target.lon), state.selectedRecoveryLink, true,
                                     hasLastLocation);
      } else {
        window_.updateRecovery(&navigation_, hasLastLocation);
      }
    }
  } else if (state.screen == "testing") {
    if (state.testingState == "disclaimer") {
      window_.initTesting();
    } else if (state.testingState == "can_start") {
      window_.initTestingConfirmed(true, state.links[0].telemetry.testingMode);
    } else if (state.testingState == "cannot_start") {
      window_.initTestingConfirmed(false, false);
    } else if (state.testingState == "waiting") {
      window_.initTestingWait();
    } else if (state.testingState == "failed") {
      const bool connectionLost = std::any_of(state.actions.begin(), state.actions.end(), [](const PlatformAction& action) {
        return action.type == "testing_connection_lost";
      });
      if (connectionLost) window_.initTestingLost();
      else window_.initTestingFailed();
    } else {
      window_.initTestingReady();
      window_.updateTesting(state.testingSelection);
      if (state.testingState == "confirm_event") window_.initTestingBox(state.testingSelection);
    }
  } else if (state.screen == "data") {
    if (state.qrView == "log_link_1") {
      window_.showLocationQr(state.flightStatistics[0].lastLatitude, state.flightStatistics[0].lastLongitude,
                             "[Link 1] Last Location", true,
                             LocationQr::IsValid(state.flightStatistics[1].lastLatitude,
                                                 state.flightStatistics[1].lastLongitude));
    } else if (state.qrView == "log_link_2") {
      window_.showLocationQr(state.flightStatistics[1].lastLatitude, state.flightStatistics[1].lastLongitude,
                             "[Link 2] Last Location", true, false);
    } else if (state.currentDataSubview == "details" && !state.logs.empty()) {
      const size_t selected = std::min<size_t>(static_cast<size_t>(std::max<int16_t>(0, state.dataSelection)),
                                               state.logs.size() - 1U);
      window_.dataShowFlightStatistics(statistics_[0], statistics_[1], state.logs[selected].name.c_str(),
                                       LocationQr::IsValid(state.flightStatistics[0].lastLatitude,
                                                           state.flightStatistics[0].lastLongitude) ||
                                           LocationQr::IsValid(state.flightStatistics[1].lastLatitude,
                                                               state.flightStatistics[1].lastLongitude));
    } else if (state.currentDataSubview == "options" && !state.logs.empty()) {
      const size_t selected = std::min<size_t>(static_cast<size_t>(std::max<int16_t>(0, state.dataSelection)),
                                               state.logs.size() - 1U);
      window_.initDataOptions(state.logs[selected].name.c_str(), state.logs[selected].active);
    } else if ((state.currentDataSubview == "confirm_finalize" || state.currentDataSubview == "confirm_delete") &&
               !state.logs.empty()) {
      window_.initData(true);
      window_.initBox(state.currentDataSubview == "confirm_finalize" ? "Finalize this log?" : "Delete this log?");
    } else if (state.currentDataSubview == "message") {
      window_.initDataMessage(state.dataMessageTitle.c_str(), state.dataMessageText.c_str());
    } else {
      window_.initData(!state.logs.empty());
      const size_t end = std::min<size_t>(state.logs.size(), state.logScrollOffset + 11U);
      for (size_t index = state.logScrollOffset; index < end; ++index) {
        std::string label = state.logs[index].name + (state.logs[index].active ? "  [ACTIVE]" : "");
        window_.listFileName(label.c_str(), static_cast<uint16_t>(index - state.logScrollOffset));
      }
      if (!state.logs.empty()) {
        const size_t selected = std::min<size_t>(static_cast<size_t>(std::max<int16_t>(0, state.dataSelection)), state.logs.size() - 1U);
        std::string label = state.logs[selected].name + (state.logs[selected].active ? "  [ACTIVE]" : "");
        window_.dataHighlight(label.c_str(), static_cast<uint16_t>(selected - state.logScrollOffset), true);
      }
      window_.dataScrollIndicators(state.logScrollOffset > 0U, end < state.logs.size(),
                                   state.logs.empty() ? -1 : static_cast<int16_t>(state.dataSelection - state.logScrollOffset));
      window_.refresh();
    }
  } else if (state.screen == "usb_storage") {
    if (!state.usbStorageMessage.empty()) {
      window_.initDataMessage("USB Drive", state.usbStorageMessage.c_str());
    } else {
      window_.initUsbStorage(state.device.usbStorageState == "host");
    }
  } else if (state.screen == "sensors") {
    if (state.calibrationState == "prepare") {
      window_.initSensorPrepareCalibrate();
    } else if (state.calibrationState == "calibrating") {
      window_.initSensorCalibrate();
      window_.updateSensorCalibrate(&navigation_);
    } else if (state.calibrationState == "concluded") {
      window_.initSensorCalibrateDone();
    } else {
      window_.initSensors();
      window_.updateSensors(&navigation_);
    }
  } else if (state.screen == "settings") {
    if (state.settingsState == "keyboard") {
      const std::string* phrase = &state.configuration.linkPhrase1;
      if (state.settingsSelection == 2) phrase = &state.configuration.linkPhrase2;
      if (state.settingsSelection == 3) phrase = &state.configuration.testingPhrase;
      std::strncpy(keyboardText_, phrase->c_str(), kMaxPhraseLen);
      keyboardText_[kMaxPhraseLen] = '\0';
      window_.drawKeyboard(keyboardText_, state.keyboardSelection, state.keyboardUppercase, kMaxPhraseLen);
    } else {
      window_.initSettings(state.settingsPage);
      window_.updateSettings(state.settingsSelection);
    }
  }

  drawStatusBar(state.device);
}
