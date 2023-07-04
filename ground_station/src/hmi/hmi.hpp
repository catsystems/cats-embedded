#pragma once

#include "JC_Button.h"
#include "config.hpp"
#include "logging/recorder.hpp"
#include "window.hpp"

class Hmi {
 public:
  Hmi(const char* dir)
      : recorder(dir),
        upButton(3),
        downButton(4),
        leftButton(2),
        rightButton(5),
        centerButton(1),
        okButton(7),
        backButton(6) {}

  void begin();

 private:
  enum State {
    MENU = 0,
    LIVE = 1,
    RECOVERY = 2,
    TESTING = 3,
    DATA = 4,
    SENSORS = 5,
    SETTINGS = 6,
  };

  enum TestingState {
    DISCLAIMER = 0,
    CAN_START = 1,
    CAN_NOT_START = 2,
    WAIT_FOR_START = 3,
    FAILED = 4,
    STARTED = 5,
  };

  State state = MENU;

  TestingState testingState = DISCLAIMER;
  uint32_t startTestingTime = 0;
  uint32_t testingIndex = 0;

  Recorder recorder;

  uint16_t oldTimeStamp = 0;

  uint32_t settingSubMenu = 0;
  int32_t settingIndex = -1;
  char keyboardString[17] = {};

  static void update(void* pvParameter);

  void fsm();
  void initMenu();
  void menu();
  void initLive();
  void live();
  void initRecovery();
  void recovery();
  void initTesting();
  void testing();
  void initData();
  void data();
  void initSensors();
  void sensors();
  void initSettings();
  void settings();

  bool initialized = false;
  bool isLogging = false;
  bool boxWindow = false;
  bool enableTestMode = false;
  bool triggerTouchdown = false;

  Button upButton;
  Button downButton;
  Button leftButton;
  Button rightButton;
  Button centerButton;

  Button okButton;
  Button backButton;

  Window window;

  uint32_t menuIndex = 0;
};
