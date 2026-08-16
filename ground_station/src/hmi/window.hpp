/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Adafruit_GFX.h>
#include <ctime>

#include "clock.hpp"
#include "display.hpp"
#include "logging/flightStatistics.hpp"
#include "navigation.hpp"
#include "settings.hpp"
#include "startup_intro.hpp"
#include "telemetry/telemetryData.hpp"

inline constexpr uint16_t BLACK = 0;
inline constexpr uint16_t WHITE = 1;

inline constexpr uint8_t SHARP_SCK = 36;
inline constexpr uint8_t SHARP_MOSI = 35;
inline constexpr uint8_t SHARP_SS = 34;

struct topBarData {
  time_t time;
  uint32_t storage;
  bool saving;
  bool locationLock;
  bool usbDetection;
};

class Window {
 public:
  Window(IDisplay &surface, Config &config, IClock &clock)
      : surface(surface), display(surface.gfx()), config(config), clock(clock) {}

  enum class LiveState { kShowGnss, kShowDownRange };

  void begin();
  void logo();
  void drawStartupIntroFrame(uint32_t elapsedMs);
  void Bootloader();

  void initBar();
  void updateBar(float batteryVoltage, bool usb = false, bool logging = false, bool location = false, bool time = false,
                 uint32_t free_memory = 100, bool recorderFault = false);

  void initMenu(int16_t index);
  void updateMenu(int16_t index);
  void drawMenuBitmap(int16_t index, uint16_t color);
  void drawMenuHighlight(int16_t index, bool highlight);

  void initLive();
  void updateLive(TelemetryInfo *info, int16_t index);
  void updateLive(TelemetryData *data, Navigation *navigation, TelemetryInfo *info, int16_t index);
  void updateLive(TelemetryData *data, Navigation *navigation, int16_t index);
  void UpdateLiveState(TelemetryData *data1, TelemetryData *data2, Navigation *navigation, LiveState state);

  void initRecovery(bool hasLastLocation);
  void updateRecovery(Navigation *navigation, bool hasLastLocation);
  void updateRecoveryTarget(Navigation *navigation, const EarthPoint3D &target, bool targetValid, int8_t selectedLink,
                            bool dualMode, bool hasLastLocation);
  bool showLocationQr(float latitude, float longitude, const char *label, bool hasPreviousPage, bool hasNextPage);

  void initTesting();
  void initTestingConfirmed(bool connected, bool testingEnabled);
  void initTestingFailed();
  void initTestingWait();
  void initTestingReady();
  void initTestingLost();
  void updateTesting(int16_t index);
  void initTestingBox(int16_t index);

  void initData(bool fileAvailable);
  void dataScrollIndicators(bool hasPrevious, bool hasNext, int16_t selectedRow = -1);
  void initDataOptions(const char *logName, bool active);
  void initDataMessage(const char *title, const char *message);
  void initUsbStorage(bool active);

  void initSensors();
  void initSensorPrepareCalibrate();
  void initSensorCalibrate();
  void updateSensorCalibrate(Navigation *navigation);
  void initSensorCalibrateDone();
  void updateSensors(Navigation *navigation);

  void initSettings(int16_t submenuIdx);
  void updateSettings(int16_t index);
  void drawSettingsTriangles(int16_t submenuIdx, int16_t color);

  void initBox(const char *text);

  void initKeyboard(char *text, uint32_t maxLength = 0);
  void drawKeyboard(char *text, int32_t keyHighlight, bool uppercase, uint32_t maxLength = 0);
  void updateKeyboard(char *text, int32_t keyHighlight, bool keyPressed = false);

  void listFileName(const char *fileName, uint16_t index, uint16_t color = BLACK);
  void dataHighlight(const char *fileName, uint16_t index, bool highlight);
  void dataShowFlightStatistics(FlightStatistics &stats1, FlightStatistics &stats2, const char *logName,
                                bool hasNextPage);

  void refresh() { surface.present(); }

  static constexpr uint8_t kShiftIdx = 29;
  static constexpr uint8_t kUnderscoreIdx = 37;

 private:
  void updateLiveData(TelemetryData *data, Navigation *navigation, int16_t index, uint16_t color);
  void updateLiveInfo(TelemetryInfo *info, int16_t index, uint16_t color);
  void drawCentreString(const char *buf, int16_t x, int16_t y);
  void drawCentreString(String &buf, int16_t x, int16_t y);
  void drawPageHeader(const char *title, bool hasPreviousPage, bool hasNextPage);
  void drawVerticalNavigationTile(bool pointsUp);
  void drawRecoveryHint(bool showHint);
  void drawIntroCloud(int16_t x, int16_t y, uint8_t scale);
  void drawIntroRocket(int16_t centerX, int16_t centerY, uint32_t frameNumber);
  void drawIntroLogo(int16_t y);

  void addSettingEntry(uint32_t settingIndex, const device_settings_t *setting, uint16_t color = BLACK);
  void highlightSetting(int16_t index, uint16_t color, bool updateDescription);

  void highlightKeyboardKey(int32_t key, uint16_t color);
  void updateKeyboardText(char *text, uint16_t color);

  void dataShowFlightStatisticsSide(FlightStatistics &stats, uint16_t index);

  void clearMainScreen();

  IDisplay &surface;
  Adafruit_GFX &display;
  Config &config;
  IClock &clock;

  bool connected[2]{};
  uint32_t lastTeleData[2]{};
  uint32_t dataAge[2]{};
  topBarData barData{};
  int32_t oldBarHour = 0;
  int32_t oldBarMinute = 0;
  bool oldBarUsbStatus = false;
  bool oldBarLoggingStatus = false;
  bool oldBarRecorderFault = false;
  uint32_t oldBarFreeMemory = 0;
  bool barBlinkStatus = false;
  TelemetryData teleData[2]{};
  TelemetryInfo infoData[2]{};
  LiveState livestate{LiveState::kShowGnss};
  float old_bearing[2] = {0.0F, 0.0F};
  int32_t old_downrange[2] = {0, 0};

  int16_t oldSettingsIndex{0};
  int16_t subMenuSettingIndex{0};
  int16_t oldMenuHighlight{0};
  int16_t oldTestingIndex{0};

  bool upperCase{false};
  int32_t oldKey{0};
  uint32_t keyboardTextMaxLength{0};
  float oldCalibrationPercentage = 0.0F;

  const char *eventName[9] = {"Ready", "Liftoff", "Burnout", "Apogee", "Main", "Touchdown", "Custom 1", "Custom 2"};
};
