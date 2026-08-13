/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "JC_Button.h"
#include "clock.hpp"
#include "config.hpp"
#include "logging/recorder.hpp"
#include "window.hpp"

#include <vector>

class Hmi {
 public:
  explicit Hmi(const char* dir)
      : recorder(dir),
        upButton(kUpButtonPin, kButtonDebounceMs),
        downButton(kDownButtonPin, kButtonDebounceMs),
        leftButton(2, kButtonDebounceMs),
        rightButton(5, kButtonDebounceMs),
        centerButton(1, kButtonDebounceMs),
        okButton(7, kButtonDebounceMs),
        backButton(6, kButtonDebounceMs),
        window(display, systemConfig, clock) {}

  void begin();

 private:
  static constexpr uint8_t kUpButtonPin = 3;
  static constexpr uint8_t kDownButtonPin = 4;
  static constexpr uint32_t kButtonDebounceMs = 15;
  static constexpr uint32_t kSensorRefreshIntervalMs = 200;

  enum State {
    MENU = 0,
    LIVE = 1,
    RECOVERY = 2,
    TESTING = 3,
    DATA = 4,
    SENSORS = 5,
    SETTINGS = 6,
    USB_STORAGE = 7,
  };

  enum TestingState {
    DISCLAIMER = 0,
    CAN_START = 1,
    CAN_NOT_START = 2,
    WAIT_FOR_START = 3,
    FAILED = 4,
    STARTED = 5,
  };

  enum CalibrationState {
    IDLE = 0,
    PREPARE = 1,
    CALIBRATING = 2,
    CONCLUDED = 3,
  };

  State state = MENU;

  TestingState testingState = DISCLAIMER;
  uint32_t startTestingTime = 0;
  int16_t testingIndex = 0;

  CalibrationState calibrationState = IDLE;
  uint32_t lastSensorRefresh = 0;

  Recorder recorder;

  uint16_t oldTimeStamp = 0;

  int16_t settingSubMenu = 0;
  int16_t settingIndex = -1;
  char keyboardString[kMaxPhraseLen + 1] = {};

  static void update(void* pvParameter);

  void fsm();
  void initMenu();
  void menu();
  void initLive();
  void live();
  void initRecovery();
  void recovery();
  void updateRecoveryLocations();
  bool showRecoveryLocation(int8_t linkIndex);
  void initTesting();
  void testing();
  void initData();
  void drawDataList();
  void openSelectedLog();
  void data();
  bool showDataLocation(int8_t linkIndex);
  void showDataStatistics();
  void initSensors();
  void sensors();
  void initSettings();
  void settings();
  void initUsbStorage();
  void usbStorage();
  void updateAutomaticUsbStorage(const RecorderStatus& recorderStatus);
  bool claimStorageForFirmware();

  bool initialized = false;
  bool isLogging = false;
  bool boxWindow = false;
  bool enableTestMode = false;
  bool triggerTouchdown = false;
  bool isCalibrating = false;

  Button upButton;
  Button downButton;
  Button leftButton;
  Button rightButton;
  Button centerButton;

  Button okButton;
  Button backButton;

  FirmwareDisplay display;
  FirmwareClock clock;
  Window window;

  int16_t menuIndex = 0;

  enum class DataView : uint8_t { List, Details, Options, ConfirmFinalize, ConfirmDelete, Message };
  DataView dataView{DataView::List};
  DataView dataMessageReturn{DataView::Options};
  size_t dataIndex{0};
  size_t dataWindowStart{0};
  std::vector<LogEntry> dataCatalog{};
  FlightLogAnalysis dataAnalysis{};
  FlightStatistics dataStatistics[2];
  int8_t dataQrLink = -1;
  char dataLogName[kLogFilenameSize] = {};

  int8_t recoveryQrLink = -1;
  int8_t selectedRecoveryLink = 0;
  EarthPoint3D recoveryLocations[2];
  bool recoveryLocationValid[2] = {false, false};
  EarthPoint3D recoveryQrPoint;

  uint32_t flashFreeMemory = 100;

  bool usbStorageSession = false;
  UsbStorageState displayedUsbStorageState{UsbStorageState::FirmwareOwned};
  RecorderState previousRecorderState{RecorderState::Idle};
  bool usbPreviouslyConnected = false;
  bool automaticUsbSharePending = false;
  uint32_t lastAutomaticUsbShareAttempt = 0;
};
