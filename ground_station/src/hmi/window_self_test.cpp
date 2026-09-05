/// Copyright (C) 2026 Control and Telemetry Systems GmbH
/// SPDX-License-Identifier: GPL-3.0-or-later
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "window.hpp"

void Window::drawSelfTest(const SelfTest& test, uint32_t now, int16_t selectedResult) {
  const uint8_t pattern = test.displayPattern(now);
  surface.clearBuffer();
  display.setTextWrap(false);
  if (pattern != 0) {
    display.fillRect(0, 0, 400, 240, pattern == 1 ? BLACK : WHITE);
    if (pattern >= 3) {
      for (int16_t y = 0; y < 240; y += 10) {
        for (int16_t x = 0; x < 400; x += 10) {
          if (((x / 10 + y / 10) % 2 == 0) == (pattern == 3)) display.fillRect(x, y, 10, 10, BLACK);
        }
      }
    }
    surface.present();
    return;
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setFont(&FreeSans12pt7b);
  display.fillRect(0, 0, 400, 30, BLACK);
  drawCentreString(test.title(), 200, 23);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BLACK);
  const auto text = [this](int16_t y, const char* value) {
    display.setCursor(10, y);
    display.print(value);
  };
  const auto rightText = [this](int16_t y, const char* value) {
    int16_t x1 = 0, y1 = 0;
    uint16_t width = 0, height = 0;
    display.getTextBounds(value, 0, y, &x1, &y1, &width, &height);
    display.setCursor(static_cast<int16_t>(390 - width - x1), y);
    display.print(value);
  };
  // Result reasons wrap at words, keeping the same readable font and margins.
  const auto reason = [this, &text](const char* value) {
    for (int16_t y = 148; *value != '\0' && y <= 168; y += 20) {
      char row[96]{};
      std::snprintf(row, sizeof(row), "%s", value);
      size_t length = std::strlen(row);
      int16_t x1 = 0, y1 = 0;
      uint16_t width = 0, height = 0;
      display.getTextBounds(row, 0, y, &x1, &y1, &width, &height);
      while (width > 380 && length > 0) {
        row[--length] = '\0';
        display.getTextBounds(row, 0, y, &x1, &y1, &width, &height);
      }
      if (value[length] != '\0') {
        const char* space = std::strrchr(row, ' ');
        if (space != nullptr) length = static_cast<size_t>(space - row);
        row[length] = '\0';
      }
      text(y, row);
      value += length;
      while (*value == ' ') ++value;
    }
  };
  const char* leftHint = "Cancel: hold B 2s";
  const char* rightHint = "";
  char line[80]{};
  if (!test.finished()) text(51, test.instruction());
  if (!test.started()) {
    text(78, "0. Gyro calibration: keep still");
    text(102, "1. Ground Station: GNSS, sensors, storage");
    text(126, "2. Telemetry: two test Vegas and LEDs");
    text(150, "3. Manual: confirm before each check");
    text(181, "Gyro offsets are saved for future starts.");
    text(202, "Settings and compass calibration are kept.");
    leftHint = "Back (B)";
    rightHint = "Start (A)";
  } else if (test.finished()) {
    std::snprintf(line, sizeof(line), "%s%s", SelfTest::resultName(test.overall()),
                  test.cancelled ? " - cancelled" : "");
    display.setFont(&FreeSansBold12pt7b);
    text(57, line);
    display.setFont(&FreeSans9pt7b);
    const int16_t first = std::max<int16_t>(0, selectedResult - 1);
    for (int16_t row = 0; row < 3 && first + row < static_cast<int16_t>(SelfTest::Check::Count); ++row) {
      const auto item = static_cast<SelfTest::Check>(first + row);
      const auto baseline = static_cast<int16_t>(84 + row * 20);
      const bool selected = first + row == selectedResult;
      if (selected) display.fillRect(0, baseline - 16, 400, 20, BLACK);
      display.setTextColor(selected ? WHITE : BLACK);
      text(baseline, SelfTest::checkName(item));
      rightText(baseline, SelfTest::resultName(test.check(item).result));
    }
    display.setTextColor(BLACK);
    reason(test.check(static_cast<SelfTest::Check>(selectedResult)).reason);
    text(189, "All checks must pass for an overall PASS.");
    text(209, test.check(SelfTest::Check::Restoration).result == SelfTest::Result::Pass
                  ? "Normal receiver settings restored."
                  : "Check receiver settings before use.");
    drawCentreString("Up/down", 200, 232);
    leftHint = "Back (B)";
    rightHint = "Rerun (A)";
  } else if (test.awaitingConfirmation) {
    rightHint = "Start (A)";
    switch (test.phase) {
      case SelfTest::Phase::Telemetry:
        text(84, "Power both test Vegas.");
        text(109, "Vega 1: cats_test_1");
        text(134, "Vega 2: cats_test_2");
        text(168, "Single, dual and each receiver separately.");
        text(194, "Radio limits: provisional bench profile");
        break;
      case SelfTest::Phase::Motion:
        text(90, "Pick up the Ground Station.");
        text(117, "Tilt and rotate around all three axes.");
        break;
      case SelfTest::Phase::Buttons:
        text(90, "Press and release all six buttons.");
        text(117, "The Start press does not count.");
        break;
      case SelfTest::Phase::Display:
        text(90, "Four patterns will fill the display.");
        text(117, "Each pattern stays for four seconds.");
        text(154, "Afterward: A = good, B = fault.");
        break;
      case SelfTest::Phase::Usb:
        text(90, "After Start, connect USB to the PC.");
        text(117, "The GS will share its USB drive.");
        text(154, "Open the CATS GS drive on the computer.");
        break;
      default:
        break;
    }
    text(211, "Waiting for your confirmation.");
  } else if (test.phase == SelfTest::Phase::GyroCalibration) {
    text(87, "Measuring the three stationary offsets.");
    text(115, "Keep still for five seconds.");
    text(143, "Movement restarts the measurement.");
    const int16_t progress = static_cast<int16_t>(std::min<uint32_t>(test.gyroCalibrationSamples, 250U) * 360U / 250U);
    display.drawRect(19, 164, 362, 18, BLACK);
    display.fillRect(20, 165, progress, 16, BLACK);
    text(211, "Offsets are saved when measurement passes.");
  } else if (test.phase == SelfTest::Phase::Hardware || test.phase == SelfTest::Phase::Telemetry) {
    const SelfTest::Check hardware[] = {SelfTest::Check::Gnss, SelfTest::Check::Stationary, SelfTest::Check::Magnetic,
                                        SelfTest::Check::Storage, SelfTest::Check::Battery};
    const SelfTest::Check telemetry[] = {SelfTest::Check::Firmware1,    SelfTest::Check::Firmware2,
                                         SelfTest::Check::Dual,         SelfTest::Check::Swapped,
                                         SelfTest::Check::Single,       SelfTest::Check::Receiver1Only,
                                         SelfTest::Check::Receiver2Only};
    const bool isHardware = test.phase == SelfTest::Phase::Hardware;
    const auto* checks = isHardware ? hardware : telemetry;
    const size_t count = isHardware ? 5 : 7;
    for (size_t row = 0; row < count; ++row) {
      const auto baseline = static_cast<int16_t>(77 + row * 19);
      text(baseline, SelfTest::checkName(checks[row]));
      rightText(baseline, SelfTest::resultName(test.check(checks[row]).result));
    }
    if (isHardware) {
      std::snprintf(line, sizeof(line), "GNSS satellites: %d", test.satellites);
      text(185, line);
    }
    std::snprintf(line, sizeof(line), "Elapsed: %lu s", static_cast<unsigned long>(test.phaseElapsedMs(now) / 1000));
    text(211, line);
  } else {
    if (test.phase == SelfTest::Phase::Motion) {
      const uint8_t masks[] = {test.accelerationAxes, test.gyroAxes, test.magneticAxes};
      const char* names[] = {"Acceleration", "Gyroscope", "Magnetometer"};
      for (size_t i = 0; i < 3; ++i) {
        const auto baseline = static_cast<int16_t>(87 + i * 32);
        text(baseline, names[i]);
        std::snprintf(line, sizeof(line), "X[%c]   Y[%c]   Z[%c]", masks[i] & 1 ? '+' : ' ', masks[i] & 2 ? '+' : ' ',
                      masks[i] & 4 ? '+' : ' ');
        rightText(baseline, line);
      }
      text(187, "Completes when every axis responds.");
      rightHint = "Skip (A)";
    } else if (test.phase == SelfTest::Phase::Buttons) {
      const char* names[] = {"Up", "Down", "Left", "Right", "Center", "A", "B"};
      int16_t y = 76;
      for (size_t i = 0; i < 7; ++i) {
        if ((SelfTest::kRequiredButtons & (1U << i)) == 0) continue;
        std::snprintf(line, sizeof(line), "[%c] %s", test.completedButtons & (1U << i) ? '+' : ' ', names[i]);
        text(y, line);
        y += 18;
      }
      text(207, "Next check waits for confirmation.");
    } else if (test.phase == SelfTest::Phase::Display) {
      text(95, "Watch four full-screen patterns.");
      text(120, "Check for missing pixels, lines or noise.");
      if (test.phaseElapsedMs(now) >= SelfTestProfile::kDisplayEndMs) {
        text(190, "Hold B for 2 seconds to cancel.");
        leftHint = "Display fault (B)";
        rightHint = "Looks good (A)";
      }
    } else if (test.phase == SelfTest::Phase::Usb) {
      text(100, "Open the CATS GS drive on the computer.");
      text(130, "Pass requires a read from the computer.");
      text(160, "The connection alone does not pass.");
      rightHint = "Skip (A)";
    }
  }
  text(232, leftHint);
  rightText(232, rightHint);
  display.setTextWrap(true);
  surface.present();
}
