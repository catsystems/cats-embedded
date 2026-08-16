/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "hmi.hpp"

#include <TimeLib.h>
#include <algorithm>
#include <cstring>

#include <freertos/task.h>

#include "console.hpp"
#include "hmi/location_qr.hpp"
#include "logging/flightStatistics.hpp"
#include "navigation.hpp"
#include "telemetry/telemetry.hpp"
#include "utils.hpp"

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
extern Telemetry link1;
extern Telemetry link2;

extern Navigation navigation;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

void Hmi::begin() {
  upButton.begin();
  downButton.begin();
  leftButton.begin();
  rightButton.begin();
  centerButton.begin();

  okButton.begin();
  backButton.begin();

  recorder.begin();
  recorder.configure(systemConfig.config.receiverMode, systemConfig.config.neverStopLogging);
  recorder.enable();
  link1.setPacketSink(&recorder, 1);
  link2.setPacketSink(&recorder, 2);

  window.begin();
  initialized = true;
  xTaskCreate(update, "task_hmi", 8196, this, 1, nullptr);
}

void Hmi::fsm() {
  recorder.configure(systemConfig.config.receiverMode, systemConfig.config.neverStopLogging);
  const RecorderStatus recorderStatus = recorder.getStatus();
  isLogging = recorderStatus.state == RecorderState::Recording;
  updateAutomaticUsbStorage(recorderStatus);
  updateRecoveryLocations();
  switch (state) {
    case MENU:
      menu();
      break;

    case LIVE:
      live();
      break;

    case RECOVERY:
      recovery();
      break;

    case TESTING:
      testing();
      break;

    case DATA:
      data();
      break;

    case SENSORS:
      sensors();
      break;

    case SETTINGS:
      settings();
      break;

    case USB_STORAGE:
      usbStorage();
      break;

    default:
      break;
  }
}

/* MENU */

void Hmi::initMenu() { window.initMenu(menuIndex); }

void Hmi::menu() {
  const uint32_t oldIndex = menuIndex;
  if (rightButton.wasPressed() && (menuIndex % 3) < 2) {
    menuIndex++;
  }

  if (leftButton.wasPressed() && (menuIndex % 3) > 0) {
    menuIndex--;
  }

  if (downButton.wasPressed() && menuIndex < 3) {
    menuIndex += 3;
  }

  if (upButton.wasPressed() && menuIndex > 2) {
    menuIndex -= 3;
  }

  if (menuIndex != oldIndex) {
    window.updateMenu(menuIndex);
  }

  if (okButton.wasPressed() || centerButton.wasPressed()) {
    state = static_cast<State>(menuIndex + 1);
    if (state == LIVE) {
      initLive();
    } else if (state == RECOVERY) {
      initRecovery();
    } else if (state == TESTING) {
      initTesting();
    } else if (state == DATA) {
      initData();
    } else if (state == SENSORS) {
      initSensors();
    } else if (state == SETTINGS) {
      initSettings();
    }
  }
}

/* LIVE */

void Hmi::initLive() { window.initLive(); }

void Hmi::live() {
  bool updated = false;

  if (link1.data.isUpdated() && link1.info.isUpdated()) {
    window.updateLive(&link1.data, &navigation, &link1.info, 0);
    updated = true;
  } else if (link1.info.isUpdated()) {
    window.updateLive(&link1.info, 0);
    updated = true;
  }

  if (link2.data.isUpdated() && link2.info.isUpdated()) {
    window.updateLive(&link2.data, &navigation, &link2.info, 1);
    updated = true;
  } else if (link2.info.isUpdated()) {
    window.updateLive(&link2.info, 1);
    updated = true;
  }

  isLogging = recorder.getStatus().state == RecorderState::Recording;

  if (updated) {
    window.refresh();
  }

  if (rightButton.wasPressed()) {
    window.UpdateLiveState(&link1.data, &link2.data, &navigation, Window::LiveState::kShowDownRange);
  }

  if (leftButton.wasPressed()) {
    window.UpdateLiveState(&link1.data, &link2.data, &navigation, Window::LiveState::kShowGnss);
  }

  if (backButton.wasPressed()) {
    state = MENU;
    window.initMenu(menuIndex);
  }
}

/* RECOVERY */

void Hmi::initRecovery() {
  recoveryQrLink = -1;
  const bool hasLastLocation = recoveryLocationValid[0] || recoveryLocationValid[1];
  window.initRecovery(hasLastLocation);
  if (systemConfig.config.receiverMode == DUAL) {
    selectedRecoveryLink = static_cast<int8_t>(recoveryLocationValid[0] ? 0 : (recoveryLocationValid[1] ? 1 : 0));
    window.updateRecoveryTarget(&navigation, recoveryLocations[selectedRecoveryLink],
                                recoveryLocationValid[selectedRecoveryLink], selectedRecoveryLink, true,
                                hasLastLocation);
  } else {
    window.updateRecovery(&navigation, hasLastLocation);
  }
}

void Hmi::updateRecoveryLocations() {
  const packedRXMessage *const messages[2] = {&link1.data.getRxData(), &link2.data.getRxData()};
  for (uint8_t index = 0; index < 2; ++index) {
    const float latitude = static_cast<float>(messages[index]->lat) / 10000.0F;
    const float longitude = static_cast<float>(messages[index]->lon) / 10000.0F;
    if (LocationQr::IsValid(latitude, longitude)) {
      recoveryLocations[index] = EarthPoint3D(latitude, longitude);
      recoveryLocationValid[index] = true;
    }
  }
}

bool Hmi::showRecoveryLocation(int8_t linkIndex) {
  if (linkIndex < 0 || linkIndex > 1 || !recoveryLocationValid[linkIndex]) {
    return false;
  }
  const EarthPoint3D &location = recoveryLocations[linkIndex];
  const char *title = linkIndex == 0 ? "[Link 1] Last Location" : "[Link 2] Last Location";
  const bool hasNextPage = recoveryLocationValid[1 - linkIndex];
  if (!window.showLocationQr(location.lat, location.lon, title, true, hasNextPage)) {
    return false;
  }
  recoveryQrLink = linkIndex;
  recoveryQrPoint = location;
  return true;
}

void Hmi::recovery() {
  const bool dualMode = systemConfig.config.receiverMode == DUAL;

  if (backButton.wasPressed()) {
    recoveryQrLink = -1;
    state = MENU;
    window.initMenu(menuIndex);
    return;
  }

  if (recoveryQrLink < 0 && dualMode && (upButton.wasPressed() || downButton.wasPressed())) {
    selectedRecoveryLink = static_cast<int8_t>(1 - selectedRecoveryLink);
    const bool hasLastLocation = recoveryLocationValid[0] || recoveryLocationValid[1];
    window.updateRecoveryTarget(&navigation, recoveryLocations[selectedRecoveryLink],
                                recoveryLocationValid[selectedRecoveryLink], selectedRecoveryLink, true,
                                hasLastLocation);
    return;
  }

  if (rightButton.wasPressed()) {
    if (recoveryQrLink < 0) {
      if (dualMode) {
        (void)showRecoveryLocation(selectedRecoveryLink);
      } else {
        const EarthPoint3D target = navigation.getPointB();
        if (LocationQr::IsValid(target.lat, target.lon) &&
            window.showLocationQr(target.lat, target.lon, "Last Location", true, false)) {
          recoveryQrLink = 0;
          recoveryQrPoint = target;
        }
      }
    } else if (dualMode && recoveryLocationValid[1 - recoveryQrLink]) {
      selectedRecoveryLink = static_cast<int8_t>(1 - recoveryQrLink);
      (void)showRecoveryLocation(selectedRecoveryLink);
    }
    return;
  }

  if (leftButton.wasPressed() && recoveryQrLink >= 0) {
    recoveryQrLink = -1;
    const bool hasLastLocation = recoveryLocationValid[0] || recoveryLocationValid[1];
    window.initRecovery(hasLastLocation);
    if (dualMode) {
      window.updateRecoveryTarget(&navigation, recoveryLocations[selectedRecoveryLink],
                                  recoveryLocationValid[selectedRecoveryLink], selectedRecoveryLink, true,
                                  hasLastLocation);
    } else {
      window.updateRecovery(&navigation, hasLastLocation);
    }
    return;
  }

  // A display transfer blocks button polling. Let the existing debounce logic
  // sample a held Link-selection button again before starting another transfer.
  if (dualMode && recoveryQrLink < 0 && (digitalRead(kUpButtonPin) == LOW || digitalRead(kDownButtonPin) == LOW)) {
    return;
  }

  if (recoveryQrLink >= 0) {
    if (dualMode) {
      const EarthPoint3D &location = recoveryLocations[recoveryQrLink];
      if (location.lat != recoveryQrPoint.lat || location.lon != recoveryQrPoint.lon) {
        (void)showRecoveryLocation(recoveryQrLink);
      }
    } else {
      const EarthPoint3D location = navigation.getPointB();
      if ((location.lat != recoveryQrPoint.lat || location.lon != recoveryQrPoint.lon) &&
          LocationQr::IsValid(location.lat, location.lon) &&
          window.showLocationQr(location.lat, location.lon, "Last Location", true, false)) {
        recoveryQrPoint = location;
      }
    }
  } else if (navigation.isUpdated()) {
    const bool hasLastLocation = recoveryLocationValid[0] || recoveryLocationValid[1];
    if (dualMode) {
      window.updateRecoveryTarget(&navigation, recoveryLocations[selectedRecoveryLink],
                                  recoveryLocationValid[selectedRecoveryLink], selectedRecoveryLink, true,
                                  hasLastLocation);
    } else {
      window.updateRecovery(&navigation, hasLastLocation);
    }
  }
}

/* TESTING */

void Hmi::initTesting() { window.initTesting(); }

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Hmi::testing() {
  if (boxWindow) {
    bool exit = false;
    if (okButton.wasPressed()) {
      link1.triggerEvent(testingIndex + 1);
      exit = true;
    }

    if (backButton.wasPressed() || exit) {
      window.initTestingReady();
      window.updateTesting(testingIndex);
      boxWindow = false;
    }
    return;
  }

  if (backButton.wasPressed()) {
    state = MENU;
    if (testingState >= WAIT_FOR_START) {
      link1.exitTesting();
      link2.enable();
    }
    testingState = DISCLAIMER;

    window.initMenu(menuIndex);
  }

  switch (testingState) {
    case DISCLAIMER: {
      if (okButton.wasPressed()) {
        bool connected = false;
        if ((link1.data.getLastUpdateTime() + 1000) > xTaskGetTickCount()) {
          connected = true;
        }

        window.initTestingConfirmed(connected, link1.data.testingMode());
        if (connected) {
          testingState = CAN_START;
        } else {
          testingState = CAN_NOT_START;
        }
      }
    } break;

    case CAN_START: {
      if (okButton.wasPressed()) {
        // Disable Link2
        link2.disable();
        link1.enterTesting();

        window.initTestingWait();

        startTestingTime = xTaskGetTickCount();
        testingState = WAIT_FOR_START;
      }
    } break;

    case WAIT_FOR_START: {
      static uint32_t counter = 0;
      if (link1.data.isUpdated()) {
        // In testing mode state indicates if we sucessfully started the mode
        if ((link1.data.getLastUpdateTime() + 200) > xTaskGetTickCount()) {
          counter++;
          if (link1.data.state() == 1 && counter > 5) {
            window.initTestingReady();
            window.updateTesting(0);
            testingIndex = 0;
            testingState = STARTED;
            counter = 0;
          }
        } else {
          counter = 0;
        }
      }
      if ((startTestingTime + 10000) < xTaskGetTickCount()) {
        link1.disable();
        window.initTestingFailed();
        testingState = FAILED;
        counter = 0;
      }
    } break;

    case CAN_NOT_START:
    case FAILED:
      break;

    case STARTED: {
      const uint32_t oldIndex = testingIndex;
      if (upButton.wasPressed() && (testingIndex % 4) > 0) {
        testingIndex--;
      } else if (downButton.wasPressed() && (testingIndex % 4) < 3) {
        testingIndex++;
      } else if (rightButton.wasPressed() && (testingIndex < 4)) {
        testingIndex += 4;
      } else if (leftButton.wasPressed() && (testingIndex > 3)) {
        testingIndex -= 4;
      }

      if (link1.data.isUpdated()) {
        if (link1.data.state() != 1) {
          testingState = FAILED;
          link1.exitTesting();
          window.initTestingLost();
        }
      }

      if ((link1.data.getLastUpdateTime() + 1000) < xTaskGetTickCount()) {
        testingState = FAILED;
        link1.exitTesting();
        window.initTestingLost();
      }

      if (oldIndex != testingIndex) {
        window.updateTesting(testingIndex);
      }

      if (okButton.wasPressed()) {
        window.initTestingBox(testingIndex);
        boxWindow = true;
      }
    } break;

    default:
      break;
  }
}

/* DATA */

void Hmi::initData() {
  dataQrLink = -1;
  dataView = DataView::List;
  if (!claimStorageForFirmware()) {
    dataCatalog.clear();
    dataMessageReturn = DataView::List;
    dataView = DataView::Message;
    window.initDataMessage("Log Error", "Unable to reclaim USB storage");
    return;
  }
  if (!recorder.refreshCatalog(dataCatalog)) {
    dataCatalog.clear();
    dataMessageReturn = DataView::List;
    dataView = DataView::Message;
    window.initDataMessage("Log Error", "Unable to read log catalog");
    return;
  }
  if (dataCatalog.empty()) {
    dataIndex = 0;
    dataWindowStart = 0;
  } else {
    dataIndex = std::min(dataIndex, dataCatalog.size() - 1U);
    dataWindowStart = std::min(dataWindowStart, dataIndex);
    if (dataIndex >= dataWindowStart + 11U) {
      dataWindowStart = dataIndex - 10U;
    }
  }
  drawDataList();
}

void Hmi::drawDataList() {
  window.initData(!dataCatalog.empty());
  const auto end = std::min(dataCatalog.size(), dataWindowStart + 11U);
  for (size_t index = dataWindowStart; index < end; ++index) {
    const auto row = static_cast<uint16_t>(index - dataWindowStart);
    char label[kLogFilenameSize + 12U]{};
    snprintf(label, sizeof(label), "%s%s", dataCatalog[index].name, dataCatalog[index].active ? "  [ACTIVE]" : "");
    if (index == dataIndex) {
      window.dataHighlight(label, row, true);
    } else {
      window.listFileName(label, row);
    }
  }
  window.dataScrollIndicators(dataWindowStart > 0U, end < dataCatalog.size(),
                              dataCatalog.empty() ? -1 : static_cast<int16_t>(dataIndex - dataWindowStart));
  window.refresh();
}

void Hmi::openSelectedLog() {
  if (dataIndex >= dataCatalog.size()) {
    return;
  }
  const LogEntry &entry = dataCatalog[dataIndex];
  strncpy(dataLogName, entry.name, sizeof(dataLogName) - 1U);
  dataLogName[sizeof(dataLogName) - 1U] = '\0';
  if (entry.active && !recorder.sync()) {
    dataMessageReturn = DataView::List;
    dataView = DataView::Message;
    window.initDataMessage("Log Error", "Unable to sync active log");
    return;
  }
  if (!dataAnalysis.parse(recorder.getDirectory(), entry.name)) {
    dataMessageReturn = DataView::List;
    dataView = DataView::Message;
    window.initDataMessage("Log Error", "Unable to read log");
    return;
  }
  dataStatistics[0] = dataAnalysis.summaries[0];
  dataStatistics[1] = dataAnalysis.summaries[1];
  dataView = DataView::Details;
  showDataStatistics();
}

bool Hmi::showDataLocation(int8_t linkIndex) {
  if (linkIndex < 0 || linkIndex > 1 || !dataStatistics[linkIndex].hasLastLocation()) {
    return false;
  }
  FlightStatistics &stats = dataStatistics[linkIndex];
  const char *title = linkIndex == 0 ? "[Link 1] Last Location" : "[Link 2] Last Location";
  const auto hasNextPage = linkIndex == 0 && dataStatistics[1].hasLastLocation();
  if (!window.showLocationQr(stats.getLastLatitude(), stats.getLastLongitude(), title, true, hasNextPage)) {
    return false;
  }
  dataQrLink = linkIndex;
  return true;
}

void Hmi::showDataStatistics() {
  dataQrLink = -1;
  window.dataShowFlightStatistics(dataStatistics[0], dataStatistics[1], dataLogName,
                                  dataStatistics[0].hasLastLocation() || dataStatistics[1].hasLastLocation());
}

// The branches directly mirror the small, explicit Data screen state machine.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Hmi::data() {
  if (dataView == DataView::Details) {
    if (backButton.wasPressed() || (upButton.wasPressed() && dataQrLink < 0)) {
      initData();
      return;
    }
    if (downButton.wasPressed() && dataQrLink < 0 && dataIndex < dataCatalog.size()) {
      dataView = DataView::Options;
      window.initDataOptions(dataCatalog[dataIndex].name, dataCatalog[dataIndex].active);
      return;
    }
    if (rightButton.wasPressed()) {
      if (dataQrLink < 0) {
        if (!showDataLocation(0)) {
          (void)showDataLocation(1);
        }
      } else if (dataQrLink == 0 && dataStatistics[1].hasLastLocation()) {
        (void)showDataLocation(1);
      }
    }

    if (leftButton.wasPressed() && dataQrLink >= 0) {
      if (dataQrLink == 1 && dataStatistics[0].hasLastLocation()) {
        (void)showDataLocation(0);
      } else {
        showDataStatistics();
      }
    }
    return;
  }

  if (dataView == DataView::Options) {
    if (backButton.wasPressed() || upButton.wasPressed()) {
      dataView = DataView::Details;
      showDataStatistics();
    } else if (okButton.wasPressed() && dataIndex < dataCatalog.size()) {
      dataView = dataCatalog[dataIndex].active ? DataView::ConfirmFinalize : DataView::ConfirmDelete;
      window.initBox(dataCatalog[dataIndex].active ? "Finalize this log?" : "Delete this log?");
    }
    return;
  }

  if (dataView == DataView::ConfirmFinalize || dataView == DataView::ConfirmDelete) {
    if (backButton.wasPressed()) {
      dataView = DataView::Options;
      window.initDataOptions(dataCatalog[dataIndex].name, dataCatalog[dataIndex].active);
      return;
    }
    if (okButton.wasPressed()) {
      const bool deleting = dataView == DataView::ConfirmDelete;
      if (deleting && !Utils::isFilesystemAvailable()) {
        dataMessageReturn = DataView::Options;
        dataView = DataView::Message;
        window.initDataMessage("USB Connected", "Disconnect USB before deleting.");
        return;
      }
      const auto success =
          deleting ? recorder.deleteLog(dataCatalog[dataIndex].name) : recorder.finalize(FinalizeReason::UserRequested);
      if (!success) {
        dataMessageReturn = DataView::Options;
        dataView = DataView::Message;
        if (deleting && !Utils::isFilesystemAvailable()) {
          window.initDataMessage("USB Connected", "Disconnect USB before deleting.");
        } else {
          window.initDataMessage(deleting ? "Delete Failed" : "Finalize Failed",
                                 deleting ? "Log was not deleted" : "Log is still active");
        }
        return;
      }
      initData();
    }
    return;
  }

  if (dataView == DataView::Message) {
    if (backButton.wasPressed()) {
      if (dataMessageReturn == DataView::List) {
        initData();
      } else {
        dataView = DataView::Options;
        window.initDataOptions(dataCatalog[dataIndex].name, dataCatalog[dataIndex].active);
      }
    }
    return;
  }

  if (dataView == DataView::List) {
    if (backButton.wasPressed()) {
      state = MENU;
      window.initMenu(menuIndex);
      dataIndex = 0;
      dataWindowStart = 0;
      automaticUsbSharePending = Utils::isConnected();
      return;
    }
    if ((downButton.wasPressed() || downButton.pressedFor(500)) && !dataCatalog.empty() &&
        dataIndex + 1U < dataCatalog.size()) {
      dataIndex++;
      if (dataIndex >= dataWindowStart + 11U) {
        dataWindowStart = dataIndex - 10U;
      }
      drawDataList();
    }
    if ((upButton.wasPressed() || upButton.pressedFor(500)) && dataIndex > 0) {
      dataIndex--;
      if (dataIndex < dataWindowStart) {
        dataWindowStart = dataIndex;
      }
      drawDataList();
    }

    if (okButton.wasPressed() && !dataCatalog.empty()) {
      openSelectedLog();
    }
  }
}

/* SENSORS */

void Hmi::initSensors() {
  lastSensorRefresh = 0;
  window.initSensors();
}

void Hmi::sensors() {
  switch (calibrationState) {
    case IDLE: {
      if (backButton.wasPressed()) {
        state = MENU;
        window.initMenu(menuIndex);
        return;
      }

      if (okButton.wasPressed()) {
        window.initSensorPrepareCalibrate();
        calibrationState = PREPARE;
        return;
      }

      const uint32_t now = millis();
      if (now - lastSensorRefresh >= kSensorRefreshIntervalMs) {
        // A full framebuffer transfer blocks this task. Leave a
        // quiet interval between sensor redraws so button polling stays fast.
        window.updateSensors(&navigation);
        lastSensorRefresh = millis();
      }
      break;
    }
    case PREPARE:
      if (okButton.wasPressed()) {
        window.initSensorCalibrate();
        calibrationState = CALIBRATING;
        navigation.setCalibrationState(navigation.CALIB_ONGOING);
      }
      if (backButton.wasPressed()) {
        calibrationState = IDLE;
        window.initSensors();
      }
      break;
    case CALIBRATING:
      window.updateSensorCalibrate(&navigation);
      if (backButton.wasPressed()) {
        navigation.setCalibrationState(navigation.CALIB_CANCELLED);
        calibrationState = IDLE;
        window.initSensors();
        return;
      }
      if (navigation.getCalibrationState() == navigation.CALIB_CONCLUDED) {
        calibrationState = CONCLUDED;
        window.initSensorCalibrateDone();
      }
      break;
    case CONCLUDED:
      if (okButton.wasPressed() || backButton.wasPressed()) {
        calibrationState = IDLE;
        if (claimStorageForFirmware()) {
          systemConfig.save();
          automaticUsbSharePending = Utils::isConnected();
        }
        window.initSensors();
      }
      break;
    default:
      break;
  }
}

/* SETTINGS */

void Hmi::initSettings() {
  settingSubMenu = 0;
  settingIndex = -1;
  window.initSettings(settingSubMenu);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Hmi::settings() {
  static bool keyboardActive = false;
  static bool configChanged = false;
  static int32_t i = 0;
  if (keyboardActive) {
    if (rightButton.wasPressed() || rightButton.pressedFor(500)) {
      if (i != 9 && i != 19 && i != 28 && i != 37) {
        i++;
        window.updateKeyboard(keyboardString, i);
      }
    }
    if (leftButton.wasPressed() || leftButton.pressedFor(500)) {
      if (i != -1 && i != 0 && i != 10 && i != 20 && i != 29) {
        i--;
        window.updateKeyboard(keyboardString, i);
      }
    }
    if (downButton.wasPressed() || downButton.pressedFor(500)) {
      if (i == -1) {
        i = 7;
        window.updateKeyboard(keyboardString, i);
      } else if (i < 15) {
        i += 10;
        window.updateKeyboard(keyboardString, i);
      } else if (i < 29) {
        i += 9;
        window.updateKeyboard(keyboardString, i);
      }
    }
    if (upButton.wasPressed() || upButton.pressedFor(500)) {
      if (i > 9) {
        if (i < 25) {
          i -= 10;
          window.updateKeyboard(keyboardString, i);
        } else if (i < 38) {
          i -= 9;
          window.updateKeyboard(keyboardString, i);
        }
      } else {
        i = -1;
        window.updateKeyboard(keyboardString, i);
      }
    }
    if (okButton.wasPressed() || okButton.pressedFor(500)) {
      window.updateKeyboard(keyboardString, i, true);
    }

    if (backButton.wasPressed()) {
      memcpy(static_cast<char *>(settingsTable[settingSubMenu][settingIndex].dataPtr), keyboardString, kMaxPhraseLen);
      keyboardString[kMaxPhraseLen] = '\0';
      window.initSettings(settingSubMenu);
      configChanged = true;
      window.updateSettings(settingIndex);
      keyboardActive = false;
    }

  } else {
    if (settingIndex == -1) {
      if (rightButton.wasPressed() && settingSubMenu < (kSettingPages - 1)) {
        settingSubMenu++;
        window.initSettings(settingSubMenu);
      }

      if (leftButton.wasPressed() && settingSubMenu > 0) {
        settingSubMenu--;
        window.initSettings(settingSubMenu);
      }
    } else {
      void *data_ptr = settingsTable[settingSubMenu][settingIndex].dataPtr;
      const settings_limits_u &cfg = settingsTable[settingSubMenu][settingIndex].config;
      switch (settingsTable[settingSubMenu][settingIndex].type) {
        case NUMBER: {
          if ((rightButton.wasPressed() || rightButton.pressedFor(500)) &&
              *static_cast<int16_t *>(data_ptr) < cfg.minmax.max) {
            (*static_cast<int16_t *>(data_ptr))++;
            configChanged = true;
            window.updateSettings(settingIndex);
          }
          if ((leftButton.wasPressed() || leftButton.pressedFor(500)) &&
              *static_cast<int16_t *>(data_ptr) > cfg.minmax.min) {
            (*static_cast<int16_t *>(data_ptr))--;
            configChanged = true;
            window.updateSettings(settingIndex);
          }
          break;
        }
        case TOGGLE: {
          if (rightButton.wasPressed() && !*static_cast<bool *>(data_ptr)) {
            (*static_cast<bool *>(data_ptr)) = true;
            configChanged = true;
            window.updateSettings(settingIndex);
          }
          if (leftButton.wasPressed() && *static_cast<bool *>(data_ptr)) {
            (*static_cast<bool *>(data_ptr)) = false;
            configChanged = true;
            window.updateSettings(settingIndex);
          }
          break;
        }
        case STRING: {
          if (okButton.wasPressed()) {
            memcpy(keyboardString, static_cast<char *>(data_ptr), kMaxPhraseLen);
            keyboardString[kMaxPhraseLen] = '\0';

            window.initKeyboard(keyboardString, cfg.stringLength);
            keyboardActive = true;
          }
          break;
        }
        case BUTTON: {
          if (okButton.wasPressed()) {
            switch (cfg.buttonAction) {
              case BUTTON_ACTION_USB_STORAGE:
                initUsbStorage();
                return;
              case BUTTON_ACTION_START_BOOTLOADER:
                window.Bootloader();
                Utils::startBootloader();
                return;
              case BUTTON_ACTION_NONE:
                break;
            }
          }
          break;
        }
        default: {
          break;
        }
      }
    }

    if (downButton.wasPressed() && settingIndex < settingsTableValueCount[settingSubMenu] - 1) {
      settingIndex++;
      window.updateSettings(settingIndex);
    }

    if (upButton.wasPressed() && settingIndex > -1) {
      settingIndex--;
      window.updateSettings(settingIndex);
    }

    if (backButton.wasPressed()) {
      state = MENU;
      if (configChanged) {
        configChanged = false;
        if (systemConfig.config.receiverMode == SINGLE) {
          // Set both link phrases to the same
          link1.setLinkPhrase(systemConfig.config.linkPhrase1, kMaxPhraseLen);
          link2.setLinkPhrase(systemConfig.config.linkPhrase1, kMaxPhraseLen);
        } else {
          // Use two different link phrases
          link1.setLinkPhrase(systemConfig.config.linkPhrase1, kMaxPhraseLen);
          link2.setLinkPhrase(systemConfig.config.linkPhrase2, kMaxPhraseLen);
        }

        link1.setTestingPhrase(systemConfig.config.testingPhrase, kMaxPhraseLen);
        if (claimStorageForFirmware()) {
          systemConfig.save();
          automaticUsbSharePending = Utils::isConnected();
        }
      }
      window.initMenu(menuIndex);
    }
  }
}

void Hmi::initUsbStorage() {
  state = USB_STORAGE;
  automaticUsbSharePending = false;
  displayedUsbStorageState = Utils::getMassStorageState();

  if (displayedUsbStorageState == UsbStorageState::HostOwned) {
    usbStorageSession = true;
    window.initUsbStorage(true);
    return;
  }
  if (displayedUsbStorageState == UsbStorageState::Preparing) {
    usbStorageSession = true;
    window.initUsbStorage(false);
    return;
  }
  if (displayedUsbStorageState == UsbStorageState::Reclaiming) {
    usbStorageSession = true;
    window.initDataMessage("USB Drive", "Closing USB drive...");
    return;
  }

  usbStorageSession = false;

  if (!Utils::isConnected()) {
    window.initDataMessage("USB Drive", "Connect USB cable first.");
    return;
  }

  const RecorderStatus recorderStatus = recorder.getStatus();
  if (recorderStatus.state != RecorderState::Idle) {
    window.initDataMessage("USB Drive", "Finalize the active log first.");
    return;
  }

  if (!recorder.shareWithMassStorage()) {
    window.initDataMessage("USB Drive", "USB storage is unavailable.");
    return;
  }

  usbStorageSession = true;
  displayedUsbStorageState = UsbStorageState::Preparing;
  window.initUsbStorage(false);
}

void Hmi::usbStorage() {
  if (!usbStorageSession) {
    if (backButton.wasPressed()) {
      state = SETTINGS;
      window.initSettings(settingSubMenu);
      window.updateSettings(settingIndex);
    }
    return;
  }

  const UsbStorageState storageState = Utils::getMassStorageState();
  if (storageState != displayedUsbStorageState) {
    displayedUsbStorageState = storageState;
    if (storageState == UsbStorageState::HostOwned) {
      window.initUsbStorage(true);
    } else if (storageState == UsbStorageState::Reclaiming) {
      window.initDataMessage("USB Drive", "Closing USB drive...");
    } else if (storageState == UsbStorageState::FirmwareOwned) {
      usbStorageSession = false;
      state = SETTINGS;
      window.initSettings(settingSubMenu);
      window.updateSettings(settingIndex);
      return;
    } else if (storageState == UsbStorageState::Fault) {
      usbStorageSession = false;
      window.initDataMessage("USB Drive", "Storage could not be remounted.");
      return;
    }
  }

  if (storageState == UsbStorageState::HostOwned && backButton.wasPressed()) {
    usbStorageSession = false;
    state = SETTINGS;
    window.initSettings(settingSubMenu);
    window.updateSettings(settingIndex);
    return;
  }

  if ((storageState == UsbStorageState::HostOwned && okButton.wasPressed()) ||
      (storageState == UsbStorageState::Preparing && backButton.wasPressed())) {
    Utils::requestFirmwareStorage();
    displayedUsbStorageState = UsbStorageState::Reclaiming;
    window.initDataMessage("USB Drive", "Closing USB drive...");
  }
}

void Hmi::updateAutomaticUsbStorage(const RecorderStatus &recorderStatus) {
  const bool connected = Utils::isConnected();
  if (!connected) {
    usbPreviouslyConnected = false;
    automaticUsbSharePending = false;
    previousRecorderState = recorderStatus.state;
    return;
  }

  if (!usbPreviouslyConnected) {
    usbPreviouslyConnected = true;
    automaticUsbSharePending = recorderStatus.state == RecorderState::Idle;
  }
  if (previousRecorderState != RecorderState::Idle && recorderStatus.state == RecorderState::Idle) {
    automaticUsbSharePending = true;
  }
  if (recorderStatus.state != RecorderState::Idle) {
    automaticUsbSharePending = false;
  }
  previousRecorderState = recorderStatus.state;

  if (!automaticUsbSharePending || state == DATA || Utils::getMassStorageState() != UsbStorageState::FirmwareOwned) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastAutomaticUsbShareAttempt < 500U) {
    return;
  }
  lastAutomaticUsbShareAttempt = now;
  if (recorder.shareWithMassStorage()) {
    automaticUsbSharePending = false;
  }
}

bool Hmi::claimStorageForFirmware() {
  automaticUsbSharePending = false;
  return Utils::claimFirmwareStorage();
}

void Hmi::update(void *pvParameter) {
  auto *ref = static_cast<Hmi *>(pvParameter);

  if (systemConfig.config.startupAnimation) {
    const uint32_t introStartMs = millis();
    TickType_t introLastTick = xTaskGetTickCount();
    while (millis() - introStartMs < StartupIntro::kDurationMs) {
      ref->window.drawStartupIntroFrame(millis() - introStartMs);
      vTaskDelayUntil(&introLastTick, pdMS_TO_TICKS(StartupIntro::kFrameIntervalMs));
    }
  } else {
    ref->window.logo();
    vTaskDelay(pdMS_TO_TICKS(StartupIntro::kStaticLogoDurationMs));
  }

  ref->window.initBar();
  ref->initMenu();

  uint32_t barUpdate = millis();
  bool timeValid = false;

  while (ref->initialized) {
    TickType_t task_last_tick = xTaskGetTickCount();

    ref->upButton.read();
    ref->downButton.read();
    ref->leftButton.read();
    ref->rightButton.read();
    ref->centerButton.read();

    ref->okButton.read();
    ref->backButton.read();

    ref->fsm();

    if (link1.data.isUpdated()) {
      // ref->window.updateBar(link1.data.ts());
    }

    if (millis() - barUpdate >= 1000) {
      barUpdate = millis();
      const float voltage = static_cast<float>(analogRead(18)) * 0.00062F;  // 0.00059154929F;
      if (!ref->isLogging && Utils::isFilesystemAvailable()) {
        ref->flashFreeMemory = Utils::getFlashMemoryUsage();
      }
      if (link2.time.isUpdated()) {
        setTime(link2.time.hour(), link2.time.minute(), link2.time.second(), 0, 0, 0);
        adjustTime(systemConfig.config.timeZoneOffset * 3600L);
        timeValid = true;
      }
      const RecorderStatus recorderStatus = ref->recorder.getStatus();
      ref->window.updateBar(voltage, static_cast<bool>(digitalRead(21)),
                            recorderStatus.state == RecorderState::Recording, link2.location.isValid(), timeValid,
                            ref->flashFreeMemory, recorderStatus.state == RecorderState::Fault);
    }

    vTaskDelayUntil(&task_last_tick, static_cast<TickType_t>(1000) / 50);
  }
}
