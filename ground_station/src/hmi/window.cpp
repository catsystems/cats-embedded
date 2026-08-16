/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "window.hpp"
#include "bmp.hpp"
#include "config.hpp"
#include "location_qr.hpp"
#include "utils.hpp"

#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <algorithm>
#include <cmath>

uint16_t GetNegativeColor(uint16_t color) {
  if (color == BLACK) {
    return WHITE;
  }
  return BLACK;
}

void Window::begin() {
  surface.begin();
  surface.clear();
  display.setRotation(0);
  oldBarHour = 0;
  oldBarMinute = 0;
  oldBarUsbStatus = false;
  oldBarLoggingStatus = false;
  oldBarRecorderFault = false;
  oldBarFreeMemory = 0;
  barBlinkStatus = false;
  oldMenuHighlight = 0;
  oldTestingIndex = 0;
  oldCalibrationPercentage = 0.0F;
  livestate = LiveState::kShowGnss;
  old_bearing[0] = old_bearing[1] = 0.0F;
  old_downrange[0] = old_downrange[1] = 0;
  connected[0] = connected[1] = false;
  lastTeleData[0] = lastTeleData[1] = 0;
  dataAge[0] = dataAge[1] = 0;
}

void Window::logo() {
  drawIntroLogo(45);
  surface.present();
}

void Window::drawIntroCloud(int16_t x, int16_t y, uint8_t scale) {
  static constexpr int8_t kCloudOutline[][2] = {
      {0, 11}, {6, 11}, {7, 7},  {10, 4}, {14, 3},  {18, 6},  {19, 8},
      {23, 8}, {25, 5}, {29, 4}, {33, 6}, {35, 10}, {40, 11}, {0, 11},
  };
  constexpr size_t kPointCount = sizeof(kCloudOutline) / sizeof(kCloudOutline[0]);
  for (size_t index = 1; index < kPointCount; ++index) {
    const auto x0 = static_cast<int16_t>(x + kCloudOutline[index - 1][0] * scale);
    const auto y0 = static_cast<int16_t>(y + kCloudOutline[index - 1][1] * scale);
    const auto x1 = static_cast<int16_t>(x + kCloudOutline[index][0] * scale);
    const auto y1 = static_cast<int16_t>(y + kCloudOutline[index][1] * scale);
    display.drawLine(x0, y0, x1, y1, BLACK);
  }
}

void Window::drawIntroRocket(int16_t centerX, int16_t centerY, uint32_t frameNumber) {
  struct Point {
    int16_t x;
    int16_t y;
  };
  const auto transform = [centerX, centerY](int16_t forward, int16_t sideways) -> Point {
    return {
        static_cast<int16_t>(centerX + (5 * forward + 9 * sideways) / 10),
        static_cast<int16_t>(centerY + (-9 * forward + 5 * sideways) / 10),
    };
  };

  const Point tip = transform(28, 0);
  const Point noseLeft = transform(12, -8);
  const Point noseRight = transform(12, 8);
  const Point tailLeft = transform(-15, -8);
  const Point tailRight = transform(-15, 8);
  display.fillTriangle(tip.x, tip.y, noseLeft.x, noseLeft.y, noseRight.x, noseRight.y, BLACK);
  display.fillTriangle(noseLeft.x, noseLeft.y, noseRight.x, noseRight.y, tailLeft.x, tailLeft.y, BLACK);
  display.fillTriangle(noseRight.x, noseRight.y, tailLeft.x, tailLeft.y, tailRight.x, tailRight.y, BLACK);

  const Point innerFrontLeft = transform(10, -5);
  const Point innerFrontRight = transform(10, 5);
  const Point innerTailLeft = transform(-11, -5);
  const Point innerTailRight = transform(-11, 5);
  display.fillTriangle(innerFrontLeft.x, innerFrontLeft.y, innerFrontRight.x, innerFrontRight.y, innerTailLeft.x,
                       innerTailLeft.y, WHITE);
  display.fillTriangle(innerFrontRight.x, innerFrontRight.y, innerTailLeft.x, innerTailLeft.y, innerTailRight.x,
                       innerTailRight.y, WHITE);

  const Point windowCenter = transform(5, 0);
  display.fillCircle(windowCenter.x, windowCenter.y, 3, BLACK);
  display.fillCircle(windowCenter.x, windowCenter.y, 1, WHITE);

  const Point bandLeft = transform(-3, -5);
  const Point bandRight = transform(-3, 5);
  display.drawLine(bandLeft.x, bandLeft.y, bandRight.x, bandRight.y, BLACK);

  const Point finLeft = transform(-11, -7);
  const Point finLeftTip = transform(-21, -16);
  const Point finLeftFront = transform(-2, -8);
  const Point finRight = transform(-11, 7);
  const Point finRightTip = transform(-21, 16);
  const Point finRightFront = transform(-2, 8);
  display.fillTriangle(finLeft.x, finLeft.y, finLeftTip.x, finLeftTip.y, finLeftFront.x, finLeftFront.y, BLACK);
  display.fillTriangle(finRight.x, finRight.y, finRightTip.x, finRightTip.y, finRightFront.x, finRightFront.y, BLACK);

  const int16_t flameWobble = (frameNumber % 2U == 0U) ? 2 : -2;
  const Point flameRoot = transform(-16, 0);
  const Point flameTip = transform(-34, flameWobble);
  const Point flameFork = transform(-28, static_cast<int16_t>(-5 - flameWobble));
  const Point flameForkRight = transform(-27, static_cast<int16_t>(5 + flameWobble));
  display.drawLine(flameRoot.x, flameRoot.y, flameTip.x, flameTip.y, BLACK);
  display.drawLine(flameRoot.x, flameRoot.y, flameFork.x, flameFork.y, BLACK);
  display.drawLine(flameRoot.x, flameRoot.y, flameForkRight.x, flameForkRight.y, BLACK);
}

void Window::drawIntroLogo(int16_t y) {
  constexpr int16_t kSourceWidth = 120;
  constexpr int16_t kOutputWidth = 90;
  constexpr int16_t kOutputHeight = 150;
  // The source bitmap spans its full box but has more visual weight on the
  // left, so a small optical offset looks better than geometric centering.
  constexpr int16_t kOutputX = (400 - kOutputWidth) / 2 + 7;
  constexpr int16_t kSourceRowBytes = kSourceWidth / 8;

  display.startWrite();
  for (int16_t outputY = 0; outputY < kOutputHeight; ++outputY) {
    const auto sourceY = static_cast<int16_t>((outputY * 4) / 3);
    for (int16_t outputX = 0; outputX < kOutputWidth; ++outputX) {
      const auto sourceX = static_cast<int16_t>((outputX * 4) / 3);
      const auto sourceByte = cats_logo[sourceY * kSourceRowBytes + sourceX / 8];
      if ((static_cast<uint32_t>(sourceByte) & (0x80U >> (static_cast<uint16_t>(sourceX) % 8U))) != 0U) {
        display.writePixel(static_cast<int16_t>(kOutputX + outputX), static_cast<int16_t>(y + outputY), BLACK);
      }
    }
  }
  display.endWrite();
}

void Window::drawStartupIntroFrame(uint32_t elapsedMs) {
  elapsedMs = std::min(elapsedMs, StartupIntro::kDurationMs);
  elapsedMs -= elapsedMs % StartupIntro::kFrameIntervalMs;
  const StartupIntro::Phase phase = StartupIntro::PhaseAt(elapsedMs);

  // Clear only the backing buffer. Sending the Sharp Memory LCD clear command
  // for every frame would introduce visible flashing before the refresh.
  surface.clearBuffer();

  if (phase == StartupIntro::Phase::kRocketFlight || phase == StartupIntro::Phase::kCloudTransition) {
    const auto drift = static_cast<int16_t>(elapsedMs / 85U);
    drawIntroCloud(static_cast<int16_t>(258 - drift), 35, 2);
    drawIntroCloud(static_cast<int16_t>(35 - drift / 2), 136, 1);

    if (phase == StartupIntro::Phase::kRocketFlight) {
      // Match the 5:-9 rocket axis so the exhaust trails directly behind the flight path.
      const auto rocketX =
          static_cast<int16_t>(20 + (172 * static_cast<int32_t>(elapsedMs)) / StartupIntro::kRocketFlightEndMs);
      const auto rocketY =
          static_cast<int16_t>(265 - (310 * static_cast<int32_t>(elapsedMs)) / StartupIntro::kRocketFlightEndMs);
      drawIntroRocket(rocketX, rocketY, elapsedMs / StartupIntro::kFrameIntervalMs);
    }

    // This foreground cloud briefly masks the rocket, making the flight read
    // as passing through the cloud layer rather than over a flat backdrop.
    drawIntroCloud(static_cast<int16_t>(90 - drift), 83, 2);
  } else {
    const uint32_t cloudElapsed = elapsedMs - StartupIntro::kLogoDescentStartMs;
    const uint32_t cloudDuration = StartupIntro::kLogoSettleEndMs - StartupIntro::kLogoDescentStartMs;
    const auto cloudTravel = static_cast<int16_t>((140U * std::min(cloudElapsed, cloudDuration)) / cloudDuration);
    drawIntroCloud(static_cast<int16_t>(52 - cloudTravel), 45, 2);
    drawIntroCloud(static_cast<int16_t>(292 + cloudTravel), 139, 1);

    int16_t logoY = 45;
    if (phase == StartupIntro::Phase::kLogoDescent) {
      const auto progress = static_cast<int32_t>(elapsedMs - StartupIntro::kLogoDescentStartMs);
      const auto duration = static_cast<int32_t>(StartupIntro::kLogoDescentEndMs - StartupIntro::kLogoDescentStartMs);
      const int64_t easedNumerator = static_cast<int64_t>(progress) * (2 * duration - progress);
      logoY = static_cast<int16_t>(-155 + (202 * easedNumerator) / (static_cast<int64_t>(duration) * duration));
    } else if (phase == StartupIntro::Phase::kLogoSettle) {
      const auto progress = static_cast<int32_t>(elapsedMs - StartupIntro::kLogoDescentEndMs);
      const auto duration = static_cast<int32_t>(StartupIntro::kLogoSettleEndMs - StartupIntro::kLogoDescentEndMs);
      logoY = static_cast<int16_t>(47 - (2LL * progress * progress) / (static_cast<int64_t>(duration) * duration));
    }
    drawIntroLogo(logoY);

    if (phase == StartupIntro::Phase::kLogoDescent) {
      const auto foregroundX = static_cast<int16_t>(170 - (220U * cloudElapsed) / cloudDuration);
      drawIntroCloud(foregroundX, 82, 2);
    }
  }

  surface.present();
}

void Window::Bootloader() {
  surface.clear();
  display.drawBitmap(136, 56, usb_logo, 128, 128, BLACK);
  surface.present();
}

void Window::drawCentreString(const char *buf, int16_t x, int16_t y) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  display.getTextBounds(buf, 0, y, &x1, &y1, &w, &h);  // calc width of new
                                                       // string
  display.setCursor(static_cast<int16_t>(x - w / 2), static_cast<int16_t>(y));
  display.print(buf);
}

void Window::drawCentreString(String &buf, int16_t x, int16_t y) {
  int16_t x1 = 0;
  int16_t y1 = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  display.getTextBounds(buf, 0, y, &x1, &y1, &w, &h);  // calc width of new
                                                       // string
  display.setCursor(static_cast<int16_t>(x - w / 2), static_cast<int16_t>(y));
  display.print(buf);
}

void Window::initBar() {
  // Memory
  display.setFont(nullptr);
  display.drawBitmap(5, 1, bar_memory, 16, 16, BLACK);
  display.setTextColor(BLACK);
  display.setTextSize(2);

  // Battery
  display.drawRoundRect(371, 3, 24, 12, 2, BLACK);
  display.fillRect(395, 5, 3, 8, BLACK);

  display.drawLine(0, 18, 400, 18, BLACK);
}

void Window::updateBar(float batteryVoltage, bool usb, bool logging, bool location, bool time, uint32_t free_memory,
                       bool recorderFault) {
  // Logging
  if (logging != oldBarLoggingStatus || recorderFault != oldBarRecorderFault) {
    display.fillRect(74, 0, 19, 18, WHITE);
    oldBarLoggingStatus = logging;
    oldBarRecorderFault = recorderFault;
  }
  if (logging) {
    display.drawBitmap(75, 1, bar_download, 16, 16, static_cast<uint16_t>(barBlinkStatus));
  } else if (recorderFault) {
    display.drawBitmap(75, 1, bar_download, 16, 16, static_cast<uint16_t>(barBlinkStatus));
    if (!barBlinkStatus) {
      display.drawLine(75, 1, 90, 16, BLACK);
      display.drawLine(90, 1, 75, 16, BLACK);
    }
  }

  // Location
  if (location) {
    display.drawBitmap(329, 1, bar_location, 16, 16, static_cast<uint16_t>(!location));
  }

  // Memory Usage
  if (free_memory != oldBarFreeMemory) {
    display.setFont(nullptr);
    display.setTextSize(2);

    display.setTextColor(WHITE);

    String t = String(oldBarFreeMemory) + "%";
    drawCentreString(t, 50, 2);

    oldBarFreeMemory = free_memory;
    display.setTextColor(BLACK);

    t = String(oldBarFreeMemory) + "%";
    drawCentreString(t, 50, 2);
  }

  if ((clock.minute() != oldBarMinute || clock.hour() != oldBarHour) && time) {
    display.setFont(nullptr);
    display.setTextSize(2);

    display.setTextColor(WHITE);

    String t = String(oldBarHour) + ":";
    if (oldBarMinute < 10) {
      t += '0';
    }
    t += String(oldBarMinute);

    drawCentreString(t, 200, 2);

    oldBarHour = clock.hour();
    oldBarMinute = clock.minute();
    display.setTextColor(BLACK);

    t = String(oldBarHour) + ":";
    if (oldBarMinute < 10) {
      t += '0';
    }
    t += String(oldBarMinute);

    drawCentreString(t, 200, 2);
  }

  // USB
  if (usb != oldBarUsbStatus) {
    oldBarUsbStatus = usb;

    display.fillRect(373, 5, 6, 8, WHITE);
    display.fillRect(380, 5, 6, 8, WHITE);
    display.fillRect(387, 5, 6, 8, WHITE);

    display.drawBitmap(376, 1, bar_flash, 16, 16, static_cast<int16_t>(!usb));
  }

  if (batteryVoltage != 0.0F && !usb) {
    const bool veryLow = batteryVoltage <= 3.3F;
    display.fillRect(373, 5, 6, 8, veryLow ? WHITE : BLACK);
    display.drawRoundRect(371, 3, 24, 12, 2, veryLow ? static_cast<uint16_t>(barBlinkStatus) : BLACK);
    display.fillRect(395, 5, 3, 8, veryLow ? static_cast<uint16_t>(barBlinkStatus) : BLACK);

    // Bar 1: > 3.5 V
    display.fillRect(373, 5, 6, 8, batteryVoltage > 3.5F ? BLACK : WHITE);
    // Bar 2: > 3.7 V
    display.fillRect(380, 5, 6, 8, batteryVoltage > 3.7F ? BLACK : WHITE);
    // Bar 3: > 3.9 V
    display.fillRect(387, 5, 6, 8, batteryVoltage > 3.9F ? BLACK : WHITE);
  }

  barBlinkStatus = !barBlinkStatus;

  surface.present();
}

void Window::initMenu(int16_t index) {
  clearMainScreen();

  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  drawCentreString("Live", 75, 127);
  drawCentreString("Recovery", 200, 127);
  drawCentreString("Testing", 325, 127);

  drawCentreString("Data", 75, 233);
  drawCentreString("Sensors", 200, 233);
  drawCentreString("Settings", 325, 233);

  for (int16_t i = 0; i < 6; i++) {
    drawMenuHighlight(i, false);
    drawMenuBitmap(i, BLACK);
  }
  updateMenu(index);
}

void Window::updateMenu(int16_t index) {
  drawMenuHighlight(oldMenuHighlight, false);
  drawMenuBitmap(oldMenuHighlight, BLACK);

  drawMenuHighlight(index, true);
  drawMenuBitmap(index, WHITE);

  surface.present();
  oldMenuHighlight = index;
}

void Window::drawMenuBitmap(int16_t index, uint16_t color) {
  switch (index) {
    case 0:
      display.drawBitmap(43, 38, menu_live, 64, 64, color);
      break;
    case 1:
      display.drawBitmap(168, 38, menu_recover, 64, 64, color);
      break;
    case 2:
      display.drawBitmap(293, 38, menu_testing, 64, 64, color);
      break;
    case 3:
      display.drawBitmap(43, 143, menu_data, 64, 64, color);
      break;
    case 4:
      display.drawBitmap(168, 143, menu_sensors, 64, 64, color);
      break;
    case 5:
      display.drawBitmap(293, 143, menu_settings, 64, 64, color);
      break;
    default:
      break;
  }
}

void Window::drawMenuHighlight(int16_t index, bool highlight) {
  auto xPos = static_cast<int16_t>((index % 3) * 125 + 36);
  auto yPos = static_cast<int16_t>((index / 3) * 105 + 31);
  if (highlight) {
    display.fillRoundRect(xPos, yPos, 78, 78, 9, BLACK);
  } else {
    display.fillRoundRect(xPos, yPos, 78, 78, 9, WHITE);
    display.drawRoundRect(xPos, yPos, 78, 78, 9, BLACK);
  }
}

void Window::initLive() {
  clearMainScreen();

  display.drawLine(199, 18, 199, 240, BLACK);
  display.drawLine(200, 18, 200, 240, BLACK);

  display.drawLine(0, 49, 400, 49, BLACK);

  display.drawBitmap(3, 50, live_altitude, 24, 24, BLACK);
  display.drawBitmap(3, 75, data_speed, 24, 24, BLACK);
  display.drawBitmap(3, 100, live_lat, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(3, 125, live_lon, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(158, 100, right_arrow, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(158, 125, right_arrow, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(1, 150, live_battery, 24, 24, BLACK);

  display.drawBitmap(120, 148, live_one, 24, 24, BLACK);
  display.drawBitmap(158, 148, live_two, 24, 24, BLACK);

  display.drawBitmap(320, 148, live_one, 24, 24, BLACK);
  display.drawBitmap(358, 148, live_two, 24, 24, BLACK);

  display.drawBitmap(203, 50, live_altitude, 24, 24, BLACK);
  display.drawBitmap(203, 75, data_speed, 24, 24, BLACK);
  display.drawBitmap(203, 100, live_lat, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(203, 125, live_lon, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(358, 100, right_arrow, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(358, 125, right_arrow, 24, 24,
                     BLACK); /* If this is changed, also need to change in function UpdateLiveState */
  display.drawBitmap(201, 150, live_battery, 24, 24, BLACK);

  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.fillRect(0, 202, 399, 240, BLACK);
  display.setTextColor(WHITE);

  connected[0] = false;
  connected[1] = false;

  display.setCursor(245, 227);
  display.print("Disconnected");

  display.setCursor(45, 227);
  display.print("Disconnected");

  display.setTextColor(BLACK);
  display.setFont(nullptr);
  surface.present();
}

void Window::UpdateLiveState(TelemetryData *data1, TelemetryData *data2, Navigation *navigation, LiveState state) {
  if (livestate == state) {
    return;
  }

  livestate = state;
  const auto xOffset1 = static_cast<int16_t>(0 * 200);
  const auto xOffset2 = static_cast<int16_t>(1 * 200);
  const int32_t first_row_offset = 28;

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (livestate == LiveState::kShowGnss) {
    // Set Downrange text of FC 1 to white
    auto downrange_m = static_cast<int32_t>(std::roundf(navigation->getDistance()));
    display.setCursor(static_cast<int16_t>(xOffset1 + first_row_offset), 120);
    if (config.config.unitSystem == UnitSystem::kMetric) {
      display.print(old_downrange[0]);
      display.print(" m");
    } else {
      display.print(Utils::MetersToFeet(old_downrange[0]));
      display.print(" ft");
    }
    old_downrange[0] = downrange_m;

    float bearing = navigation->computeBearing();

    display.setCursor(static_cast<int16_t>(xOffset1 + first_row_offset), 145);
    display.print(old_bearing[0]);
    display.print(" deg");
    old_bearing[0] = bearing;

    // Set Downrange text of FC 2 to white
    downrange_m = static_cast<int32_t>(std::roundf(navigation->getDistance()));
    display.setCursor(static_cast<int16_t>(xOffset2 + first_row_offset), 120);
    if (config.config.unitSystem == UnitSystem::kMetric) {
      display.print(old_downrange[1]);
      display.print(" m");
    } else {
      display.print(Utils::MetersToFeet(old_downrange[1]));
      display.print(" ft");
    }
    old_downrange[1] = downrange_m;

    bearing = navigation->computeBearing();

    display.setCursor(static_cast<int16_t>(xOffset2 + first_row_offset), 145);
    display.print(old_bearing[1]);
    display.print(" deg");
    old_bearing[1] = bearing;

    // Set DownRange to white
    display.drawBitmap(3, 100, down_range, 24, 24, WHITE);
    display.drawBitmap(3, 125, compass, 24, 24, WHITE);
    display.drawBitmap(203, 100, down_range, 24, 24, WHITE);
    display.drawBitmap(203, 125, compass, 24, 24, WHITE);
    display.drawBitmap(158, 100, left_arrow, 24, 24, WHITE);
    display.drawBitmap(158, 125, left_arrow, 24, 24, WHITE);
    display.drawBitmap(358, 100, left_arrow, 24, 24, WHITE);
    display.drawBitmap(358, 125, left_arrow, 24, 24, WHITE);

    // Set GNSS to black
    display.drawBitmap(3, 100, live_lat, 24, 24, BLACK);
    display.drawBitmap(3, 125, live_lon, 24, 24, BLACK);
    display.drawBitmap(203, 100, live_lat, 24, 24, BLACK);
    display.drawBitmap(203, 125, live_lon, 24, 24, BLACK);
    display.drawBitmap(158, 100, right_arrow, 24, 24, BLACK);
    display.drawBitmap(158, 125, right_arrow, 24, 24, BLACK);
    display.drawBitmap(358, 100, right_arrow, 24, 24, BLACK);
    display.drawBitmap(358, 125, right_arrow, 24, 24, BLACK);
  } else {
    // Set GNSS text to white
    display.setCursor(static_cast<int16_t>(xOffset1 + first_row_offset), 120);
    display.print(data1->lat(), 4);
    display.print(" N");

    display.setCursor(static_cast<int16_t>(xOffset1 + first_row_offset), 145);
    display.print(data1->lon(), 4);
    display.print(" E");

    display.setCursor(static_cast<int16_t>(xOffset2 + first_row_offset), 120);
    display.print(data2->lat(), 4);
    display.print(" N");

    display.setCursor(static_cast<int16_t>(xOffset2 + first_row_offset), 145);
    display.print(data2->lon(), 4);
    display.print(" E");

    // Set GNSS to white
    display.drawBitmap(3, 100, live_lat, 24, 24, WHITE);
    display.drawBitmap(3, 125, live_lon, 24, 24, WHITE);
    display.drawBitmap(203, 100, live_lat, 24, 24, WHITE);
    display.drawBitmap(203, 125, live_lon, 24, 24, WHITE);
    display.drawBitmap(158, 100, right_arrow, 24, 24, WHITE);
    display.drawBitmap(158, 125, right_arrow, 24, 24, WHITE);
    display.drawBitmap(358, 100, right_arrow, 24, 24, WHITE);
    display.drawBitmap(358, 125, right_arrow, 24, 24, WHITE);

    // Set DownRange to black
    display.drawBitmap(3, 100, down_range, 24, 24, BLACK);
    display.drawBitmap(3, 125, compass, 24, 24, BLACK);
    display.drawBitmap(203, 100, down_range, 24, 24, BLACK);
    display.drawBitmap(203, 125, compass, 24, 24, BLACK);
    display.drawBitmap(158, 100, left_arrow, 24, 24, BLACK);
    display.drawBitmap(158, 125, left_arrow, 24, 24, BLACK);
    display.drawBitmap(358, 100, left_arrow, 24, 24, BLACK);
    display.drawBitmap(358, 125, left_arrow, 24, 24, BLACK);
  }

  surface.present();
}

void Window::updateLive(TelemetryInfo *info, int16_t index) {
  if (index > 1) {
    return;
  }

  // clear update flag
  info->clear();

  updateLiveInfo(&infoData[index], index, BLACK);

  memcpy(&infoData[index], info, sizeof(infoData[0]));
  dataAge[index] = static_cast<uint32_t>(clock.nowMs()) - lastTeleData[index];
  updateLiveInfo(&infoData[index], index, WHITE);
}

void Window::updateLive(TelemetryData *data, Navigation *navigation, TelemetryInfo *info, int16_t index) {
  if (index > 1) {
    return;
  }

  lastTeleData[index] = static_cast<uint32_t>(clock.nowMs());

  // Clear update flag
  data->clear();
  info->clear();

  updateLiveData(&teleData[index], navigation, index, WHITE);
  updateLiveInfo(&infoData[index], index, BLACK);

  // display.fillRect(10,19,190,200, WHITE);

  memcpy(&teleData[index], data, sizeof(teleData[0]));
  memcpy(&infoData[index], info, sizeof(infoData[0]));

  dataAge[index] = 0;

  updateLiveData(&teleData[index], navigation, index, BLACK);
  updateLiveInfo(&infoData[index], index, WHITE);
}

void Window::updateLive(TelemetryData *data, Navigation *navigation, int16_t index) {
  if (index > 1) {
    return;
  }

  lastTeleData[index] = static_cast<uint32_t>(clock.nowMs());

  // Clear update flag
  data->clear();

  updateLiveData(&teleData[index], navigation, index, WHITE);

  // display.fillRect(10,19,190,200, WHITE);

  memcpy(&teleData[index], data, sizeof(teleData[0]));

  dataAge[index] = 0;

  updateLiveData(&teleData[index], navigation, index, BLACK);
}

const char *const stateName[] = {"INVALID", "CALIB", "READY", "THRUST", "COAST", "DROGUE", "MAIN", "DOWN"};

const char *const errorName[] = {"No Config",   "Log Full",         "Filter Error",
                                 "Overheating", "Continuity Error", "Calibration Error"};

void Window::updateLiveData(TelemetryData *data, Navigation *navigation, int16_t index, uint16_t color) {
  const auto xOffset = static_cast<int16_t>(index * 200);

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(color);

  if (data->testingMode()) {
    drawCentreString("TESTING", static_cast<int16_t>(xOffset + 100), 42);
    display.fillRect(static_cast<int16_t>(xOffset + 1), 50, 198, 151, WHITE);
    display.setCursor(static_cast<int16_t>(xOffset + 20), 80);
    display.print("DO NOT FLY!");
    return;
  }

  drawCentreString(stateName[data->state()], static_cast<int16_t>(xOffset + 100), 42);

  const int32_t first_row_offset = 28;

  display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 70);
  const int32_t altitude_m = data->altitude();
  if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(altitude_m);
    display.print(" m");
  } else {
    display.print(Utils::MetersToFeet(altitude_m));
    display.print(" ft");
  }

  display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 95);

  const int16_t velocity_m_s = data->velocity();
  if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(velocity_m_s);
    display.print(" m/s");
  } else {
    display.print(Utils::MetersToFeet(velocity_m_s));
    display.print(" ft/s");
  }

  if (livestate == LiveState::kShowGnss) {
    display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 120);
    display.print(data->lat(), 4);
    display.print(" N");

    display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 145);
    display.print(data->lon(), 4);
    display.print(" E");
  } else {
    const auto downrange_m = static_cast<int32_t>(std::roundf(navigation->getDistance()));
    display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 120);
    if (config.config.unitSystem == UnitSystem::kMetric) {
      display.setTextColor(WHITE);
      display.print(old_downrange[index]);
      display.print(" m");
      display.setTextColor(color);
      display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 120);
      display.print(downrange_m);
      display.print(" m");
    } else {
      display.setTextColor(WHITE);
      display.print(Utils::MetersToFeet(old_downrange[index]));
      display.print(" ft");
      display.setTextColor(color);
      display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 120);
      display.print(Utils::MetersToFeet(downrange_m));
      display.print(" ft");
    }
    old_downrange[index] = downrange_m;

    const float bearing = navigation->computeBearing();

    display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 145);
    display.setTextColor(WHITE);
    display.print(old_bearing[index]);
    display.print(" deg");
    display.setTextColor(color);
    display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 145);
    display.print(bearing);
    display.print(" deg");
    old_bearing[index] = bearing;
  }

  display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 170);
  display.print(data->voltage());
  display.print(" V");

  if (static_cast<bool>(data->pyroContinuity() & 0x01U)) {
    display.drawBitmap(static_cast<int16_t>(xOffset + 142), 154, live_checkmark, 16, 16, color);
  } else {
    display.drawBitmap(static_cast<int16_t>(xOffset + 142), 154, live_cross, 16, 16, color);
  }

  if (static_cast<bool>(data->pyroContinuity() & 0x02U)) {
    display.drawBitmap(static_cast<int16_t>(xOffset + 180), 154, live_checkmark, 16, 16, color);
  } else {
    display.drawBitmap(static_cast<int16_t>(xOffset + 180), 154, live_cross, 16, 16, color);
  }

  display.setFont(&FreeSans9pt7b);
  display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 192);

  if (static_cast<bool>(data->errors() & 0x04U)) {
    display.print(errorName[2]);
  } else if (static_cast<bool>(data->errors() & 0x20U)) {
    display.print(errorName[5]);
  } else if (static_cast<bool>(data->errors() & 0x10U)) {
    display.print(errorName[4]);
  } else if (static_cast<bool>(data->errors() & 0x02U)) {
    display.print(errorName[1]);
  } else if (static_cast<bool>(data->errors() & 0x01U)) {
    display.print(errorName[0]);
  } else if (static_cast<bool>(data->errors() & 0x08U)) {
    display.print(errorName[3]);
  }

  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(GetNegativeColor(color));
}

void Window::updateLiveInfo(TelemetryInfo *info, int16_t index, uint16_t color) {
  const auto xOffset = static_cast<int16_t>(index * 200);

  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(color);

  if (dataAge[index] > 4900) {
    if (color == WHITE) {
      connected[index] = false;
      display.fillRect(xOffset, 202, 199, 240, BLACK);
      display.setCursor(static_cast<int16_t>(xOffset + 45), 227);
      display.print("Disconnected");
      info->lq();
    }
  } else {
    if (!connected[index]) {
      if (color == WHITE) {
        display.fillRect(xOffset, 202, 199, 240, BLACK);
        connected[index] = true;
        display.setCursor(static_cast<int16_t>(xOffset + 5), 217);
        display.print("AGE");
        display.setCursor(static_cast<int16_t>(xOffset + 100), 217);
        display.print("SNR");
        display.setCursor(static_cast<int16_t>(xOffset + 5), 237);
        display.print("LQ");
        display.setCursor(static_cast<int16_t>(xOffset + 100), 237);
        display.print("RSSI");
      }
    }
    display.setCursor(static_cast<int16_t>(xOffset + 50), 217);
    display.print(static_cast<float>(dataAge[index]) / 1000.0F, 1);
    display.setCursor(static_cast<int16_t>(xOffset + 145), 217);
    display.print(info->snr());
    display.setCursor(static_cast<int16_t>(xOffset + 50), 237);
    display.print(info->lq());
    display.setCursor(static_cast<int16_t>(xOffset + 145), 237);
    display.print(info->rssi());
  }
}

void Window::initRecovery(bool hasLastLocation) {
  clearMainScreen();

  display.drawCircle(300, 125, 80, BLACK);

  display.drawBitmap(5, 40, rocket_recovery, 32, 32, BLACK);

  display.drawBitmap(40, 30, live_lat, 24, 24, BLACK);
  display.drawBitmap(40, 55, live_lon, 24, 24, BLACK);

  display.drawBitmap(5, 100, house_recovery, 32, 32, BLACK);
  display.drawBitmap(40, 90, live_lat, 24, 24, BLACK);
  display.drawBitmap(40, 115, live_lon, 24, 24, BLACK);

  drawRecoveryHint(hasLastLocation);

  surface.present();
}

void Window::updateRecovery(Navigation *navigation, bool hasLastLocation) {
  const EarthPoint3D target = navigation->getPointB();
  updateRecoveryTarget(navigation, target, LocationQr::IsValid(target.lat, target.lon), -1, false, hasLastLocation);
}

void Window::updateRecoveryTarget(Navigation *navigation, const EarthPoint3D &target, bool targetValid,
                                  int8_t selectedLink, bool dualMode, bool hasLastLocation) {
  display.fillRect(60, 19, 400, 222, WHITE);

  float angle = navigation->getNorth();

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);

  display.setCursor(70, 50);
  float lon = target.lon;
  float lat = target.lat;
  if (!targetValid) {
    display.print(" --");
  } else {
    display.print(lat, 4);
  }

  display.setCursor(70, 75);
  if (!targetValid) {
    display.print(" --");
  } else {
    display.print(lon, 4);
  }

  lon = navigation->getPointA().lon;
  lat = navigation->getPointA().lat;

  display.setCursor(70, 110);
  if (lat == 0) {
    display.print(" -");
  } else {
    display.print(lat, 4);
  }

  display.setCursor(70, 135);
  if (lon == 0) {
    display.print(" -");
  } else {
    display.print(lon, 4);
  }

  const EarthPoint3D home = navigation->getPointA();
  float distance_m = 0.0F;
  float azimuth = 0.0F;
  const bool solutionValid = targetValid && LocationQr::IsValid(home.lat, home.lon);
  if (solutionValid) {
    constexpr float kEarthRadiusM = 6378100.0F;
    const float dy = (target.lat - home.lat) * (PI_F / 180.0F) * kEarthRadiusM;
    const float dx = (target.lon - home.lon) * (PI_F / 180.0F) * cos(home.lat * PI_F / 180.0F) * kEarthRadiusM;
    const float dz = target.alt - home.alt;
    distance_m = sqrt(dx * dx + dy * dy + dz * dz);
    azimuth = atan2(dx, dy);
  }

  display.setCursor(70, 170);
  if (!solutionValid) {
    display.print("--");
  } else if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(distance_m, 0);
    display.print(" m");
  } else {
    display.print(Utils::MetersToFeet(distance_m), 0);
    display.print(" ft");
  }

  if (dualMode) {
    display.fillRect(2, 145, 58, 28, BLACK);
    display.setFont(&FreeSans9pt7b);
    display.setTextColor(WHITE);
    display.setCursor(19, 165);
    display.print(selectedLink == 0 ? "L1" : "L2");
    display.fillTriangle(7, 157, 12, 151, 17, 157, WHITE);
    display.fillTriangle(7, 161, 12, 167, 17, 161, WHITE);
    display.setTextColor(BLACK);
  }

  display.setFont(&FreeSans9pt7b);

  const float radius = 90;
  const float correctionFactor = 0.06;

  auto x = static_cast<int16_t>(radius * cos(angle - PI_F / 2));
  auto y = static_cast<int16_t>(radius * sin(angle - PI_F / 2) + 125);
  drawCentreString("N", static_cast<int16_t>(x + 300),
                   static_cast<int16_t>(static_cast<float>(y) + correctionFactor * static_cast<float>(y)));

  x = static_cast<int16_t>(radius * cos(angle));
  y = static_cast<int16_t>(radius * sin(angle) + 125);
  drawCentreString("E", static_cast<int16_t>(x + 300),
                   static_cast<int16_t>(static_cast<float>(y) + correctionFactor * static_cast<float>(y)));

  x = static_cast<int16_t>(radius * cos(angle + PI_F / 2));
  y = static_cast<int16_t>(radius * sin(angle + PI_F / 2) + 125);
  drawCentreString("S", static_cast<int16_t>(x + 300),
                   static_cast<int16_t>(static_cast<float>(y) + correctionFactor * static_cast<float>(y)));

  x = static_cast<int16_t>(radius * cos(angle + PI_F));
  y = static_cast<int16_t>(radius * sin(angle + PI_F) + 125);
  drawCentreString("W", static_cast<int16_t>(x + 300),
                   static_cast<int16_t>(static_cast<float>(y) + correctionFactor * static_cast<float>(y)));

  angle = azimuth + angle - PI_F / 2;

  x = static_cast<int16_t>(70 * cos(angle) + 300);
  y = static_cast<int16_t>(70 * sin(angle) + 125);
  const auto x1 = static_cast<int16_t>(30 * cos(angle + 0.2F) + 300);
  const auto y1 = static_cast<int16_t>(30 * sin(angle + 0.2F) + 125);
  const auto x2 = static_cast<int16_t>(30 * cos(angle - 0.2F) + 300);
  const auto y2 = static_cast<int16_t>(30 * sin(angle - 0.2F) + 125);

  display.drawCircle(300, 125, 80, BLACK);

  if (solutionValid && distance_m > 20) {
    display.fillTriangle(x, y, x1, y1, x2, y2, BLACK);
  } else if (solutionValid) {
    display.drawCircle(300, 125, 6, BLACK);
  } else {
    display.setFont(&FreeSans9pt7b);
    display.setTextColor(BLACK);
    drawCentreString("No location", 300, 132);
  }

  drawRecoveryHint(hasLastLocation);

  surface.present();
}

void Window::drawRecoveryHint(bool showHint) {
  display.fillRect(0, 180, 200, 60, WHITE);
  if (!showHint) {
    return;
  }

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.drawRoundRect(2, 181, 170, 57, 5, BLACK);
  display.setCursor(8, 203);
  display.print("Press");
  display.fillTriangle(88, 195, 74, 187, 74, 203, BLACK);
  display.setCursor(92, 203);
  display.print("for");
  display.setCursor(8, 231);
  display.print("last locations");
}

void Window::initBox(const char *text) {
  display.fillRect(60, 60, 280, 120, WHITE);
  display.drawRect(60, 60, 280, 120, BLACK);

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  drawCentreString(text, 200, 110);

  display.setFont(&FreeSans9pt7b);

  display.setCursor(80, 160);
  display.print("Cancel (B)");

  display.setCursor(255, 160);
  display.print("OK (A)");

  surface.present();
}

void Window::initTestingBox(int16_t index [[maybe_unused]]) {
  display.fillRect(60, 60, 280, 120, WHITE);
  display.drawRect(60, 60, 280, 120, BLACK);

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  drawCentreString("Trigger Event?", 200, 110);

  display.setFont(&FreeSans9pt7b);

  display.setCursor(80, 160);
  display.print("Cancel (B)");

  display.setCursor(255, 160);
  display.print("OK (A)");

  surface.present();
}

void Window::initTesting() {
  clearMainScreen();

  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(10, 120);
  display.setTextSize(1);

  drawCentreString("Read this before continuing!", 200, 50);

  display.setFont(&FreeSans9pt7b);

  display.setCursor(6, 80);
  display.print("Testing mode enables manual triggering of");
  display.setCursor(6, 100);
  display.print("events and the corresponding actions associated");
  display.setCursor(6, 120);
  display.print("with them. This feature should only be used for");
  display.setCursor(6, 140);
  display.print("testing purposes and never during flight.");
  display.setCursor(6, 160);
  display.print("CATS GmbH is not responsible for any potential");
  display.setCursor(6, 180);
  display.print("injuries or material damage caused by operation");
  display.setCursor(6, 200);
  display.print("of the CATS flight computers.");

  display.setCursor(6, 225);
  display.print("Cancel (B)");

  display.setCursor(290, 225);
  display.print("Continue (A)");

  surface.present();
}

void Window::initTestingConfirmed(bool connected, bool testingEnabled) {
  clearMainScreen();
  display.setTextSize(1);
  if (connected) {
    if (testingEnabled) {
      display.drawRect(90, 95, 220, 50, BLACK);
      display.drawRect(91, 96, 218, 48, BLACK);
      display.drawRect(92, 97, 216, 46, BLACK);
      display.setFont(&FreeSansBold12pt7b);
      drawCentreString("ARM", 200, 127);

      display.setFont(&FreeSans9pt7b);
      display.setCursor(6, 225);
      display.print("Cancel (B)");

      display.setCursor(290, 225);
      display.print("Continue (A)");
    } else {
      display.setFont(&FreeSansBold9pt7b);
      drawCentreString("Not in Testing Mode.", 200, 100);
      display.setFont(&FreeSans9pt7b);
      display.setCursor(6, 130);
      display.print("Connect the flight computer to the Configurator,");
      display.setCursor(6, 150);
      display.print("enable 'Testing Mode' and set 'Testing Phrase'.");

      display.setFont(&FreeSans9pt7b);
      display.setCursor(6, 225);
      display.print("Cancel (B)");
    }
  } else {
    display.setFont(&FreeSansBold9pt7b);
    drawCentreString("No Connection", 200, 100);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 130);
    display.print("Make sure the 'Link Phrase 1' is set correctly on");
    display.setCursor(6, 150);
    display.print("both the flight computer and the ground station.");

    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 225);
    display.print("Cancel (B)");
  }

  surface.present();
}
void Window::initTestingFailed() {
  clearMainScreen();
  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Could not ARM System", 200, 100);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 130);
  display.print("Make sure 'Test Phrase' is set correctly on both");
  display.setCursor(6, 150);
  display.print("the flight computer and the ground station.");

  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 225);
  display.print("Cancel (B)");

  surface.present();
}

void Window::initTestingLost() {
  clearMainScreen();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Connection Lost", 200, 100);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 130);
  display.print("For safety reason the ground station disconnects");
  display.setCursor(6, 150);
  display.print("very quickly. Make sure you have a good");
  display.setCursor(6, 170);
  display.print("connection before continuing.");
  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 225);
  display.print("Cancel (B)");

  surface.present();
}

void Window::initTestingWait() {
  clearMainScreen();
  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Starting testing mode...", 200, 100);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 130);
  display.print("This may take a few seconds.");

  display.setFont(&FreeSans9pt7b);
  display.setCursor(6, 225);
  display.print("Cancel (B)");

  surface.present();
}

void Window::initTestingReady() {
  clearMainScreen();

  display.drawLine(199, 18, 199, 220, BLACK);
  display.drawLine(200, 18, 200, 220, BLACK);

  display.drawLine(0, 19, 399, 19, BLACK);

  display.drawLine(0, 68, 399, 68, BLACK);
  display.drawLine(0, 69, 399, 69, BLACK);

  display.drawLine(0, 118, 399, 118, BLACK);
  display.drawLine(0, 119, 399, 119, BLACK);

  display.drawLine(0, 168, 399, 168, BLACK);
  display.drawLine(0, 169, 399, 169, BLACK);

  display.drawLine(0, 218, 399, 218, BLACK);
  display.drawLine(0, 219, 399, 219, BLACK);

  display.setTextSize(1);
  display.setFont(&FreeSans12pt7b);

  drawCentreString(eventName[0], 100, 51);
  drawCentreString(eventName[1], 100, 101);
  drawCentreString(eventName[2], 100, 151);
  drawCentreString(eventName[3], 100, 201);

  drawCentreString(eventName[4], 300, 51);
  drawCentreString(eventName[5], 300, 101);
  drawCentreString(eventName[6], 300, 151);
  drawCentreString(eventName[7], 300, 201);
}

void Window::updateTesting(int16_t index) {
  auto xOffset = static_cast<int16_t>(201 * (oldTestingIndex / 4));
  auto yOffset = static_cast<int16_t>(50 * (oldTestingIndex % 4) + 20);

  display.fillRect(xOffset, yOffset, 199, 48, WHITE);

  display.setTextSize(1);
  display.setFont(&FreeSans12pt7b);
  display.setTextColor(BLACK);

  xOffset = static_cast<int16_t>(200 * (oldTestingIndex / 4) + 100);
  yOffset = static_cast<int16_t>(50 * (oldTestingIndex % 4) + 51);

  drawCentreString(eventName[oldTestingIndex], xOffset, yOffset);

  xOffset = static_cast<int16_t>(201 * (index / 4));
  yOffset = static_cast<int16_t>(50 * (index % 4) + 20);

  display.fillRect(xOffset, yOffset, 199, 48, BLACK);

  display.setTextColor(WHITE);
  xOffset = static_cast<int16_t>(200 * (index / 4) + 100);
  yOffset = static_cast<int16_t>(50 * (index % 4) + 51);
  drawCentreString(eventName[index], xOffset, yOffset);

  oldTestingIndex = index;
  surface.present();
}

void Window::initData(bool fileAvailable) {
  clearMainScreen();
  if (!fileAvailable) {
    display.setTextSize(1);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(BLACK);
    drawCentreString("No flight logs found!", 200, 100);
    surface.present();
  }
}

void Window::dataScrollIndicators(bool hasPrevious, bool hasNext, int16_t selectedRow) {
  if (hasPrevious) {
    display.fillTriangle(389, 25, 383, 33, 395, 33, selectedRow == 0 ? WHITE : BLACK);
  }
  if (hasNext) {
    display.fillTriangle(389, 233, 383, 225, 395, 225, selectedRow == 10 ? WHITE : BLACK);
  }
}

void Window::initDataOptions(const char *logName, bool active) {
  clearMainScreen();
  drawPageHeader(logName, false, false);
  display.fillRect(0, 59, 400, 36, BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(18, 85);
  display.print(active ? "Finalize Log" : "Delete Log");
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BLACK);
  display.setCursor(10, 230);
  display.print("Back (B)");
  display.setCursor(260, 230);
  display.print("Select (A)");
  drawVerticalNavigationTile(true);
  surface.present();
}

void Window::initDataMessage(const char *title, const char *message) {
  clearMainScreen();
  drawPageHeader(title, true, false);
  display.setFont(&FreeSans12pt7b);
  display.setTextColor(BLACK);
  drawCentreString(message, 200, 120);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(10, 230);
  display.print("Back (B)");
  surface.present();
}

void Window::initUsbStorage(bool active) {
  clearMainScreen();
  drawPageHeader("USB Drive", false, false);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  if (!active) {
    display.setFont(&FreeSansBold12pt7b);
    drawCentreString("Preparing USB drive...", 200, 120);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(10, 230);
    display.print("Cancel (B)");
    surface.present();
    return;
  }

  display.drawBitmap(136, 53, usb_logo, 128, 128, BLACK);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Logs available on PC", 200, 205);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(10, 230);
  display.print("Back (B)");
  display.setCursor(245, 230);
  display.print("Disconnect (A)");
  surface.present();
}

bool Window::showLocationQr(float latitude, float longitude, const char *label, bool hasPreviousPage,
                            bool hasNextPage) {
  if (!LocationQr::IsValid(latitude, longitude)) {
    return false;
  }

  clearMainScreen();
  drawPageHeader(label, hasPreviousPage, hasNextPage);

  if (!LocationQr::DrawGoogleMapsQr(display, latitude, longitude, 118, 50, BLACK, WHITE)) {
    return false;
  }
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  drawCentreString("Scan to see the last location on Google Maps", 200, 235);
  surface.present();
  return true;
}

void Window::drawPageHeader(const char *title, bool hasPreviousPage, bool hasNextPage) {
  display.fillRect(0, 19, 400, 30, BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  drawCentreString(title, 200, 42);
  if (hasPreviousPage) {
    display.fillTriangle(13, 33, 21, 25, 21, 41, WHITE);
  }
  if (hasNextPage) {
    display.fillTriangle(386, 33, 378, 25, 378, 41, WHITE);
  }
}

void Window::drawVerticalNavigationTile(bool pointsUp) {
  display.fillRect(370, 211, 30, 29, BLACK);
  if (pointsUp) {
    display.fillTriangle(385, 218, 377, 231, 393, 231, WHITE);
  } else {
    display.fillTriangle(377, 219, 393, 219, 385, 232, WHITE);
  }
}

void Window::initSensors() {
  clearMainScreen();

  display.drawLine(200, 19, 200, 250, BLACK);

  // display.drawLine(0, 125 + 19, 400, 125 + 19, BLACK);
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.fillRect(0, 19, 200, 30, BLACK);
  drawCentreString("IMU", 100, 42);

  display.fillRect(200, 19, 200, 30, BLACK);
  drawCentreString("GNSS", 300, 42);

  display.setTextColor(BLACK);
  display.setFont(&FreeSansBold9pt7b);

  /* Units */

  display.setCursor(40, 65);
  display.print("[G]");

  display.setCursor(120, 65);
  display.print("[deg/s]");

  display.setCursor(87, 170);
  display.print("[-]");

  display.setCursor(215, 200);
  display.print("Press A to calibrate");
  display.setCursor(220, 225);
  display.print("the compass");

  surface.present();
}

void Window::updateSensors(Navigation *navigation) {
  int16_t xinitOffset = 10;
  int16_t xOffset = 30;
  int16_t yinitOffset = 90;
  const int16_t yOffset = 30;

  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  display.fillRect(20, 75, 75, 80, WHITE);

  /* Ax, Ay, Az */

  display.setCursor(xinitOffset, yinitOffset);
  display.print("Ax: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), yinitOffset);
  display.print(navigation->getAX(), 2);

  display.setCursor(xinitOffset, static_cast<int16_t>(yinitOffset + yOffset));
  display.print("Ay: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), static_cast<int16_t>(yinitOffset + yOffset));
  display.print(navigation->getAY(), 2);

  display.setCursor(xinitOffset, static_cast<int16_t>(yinitOffset + 2 * yOffset));
  display.print("Az: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), static_cast<int16_t>(yinitOffset + 2 * yOffset));
  display.print(navigation->getAZ(), 2);

  /* Gx, Gy, Gz */

  xinitOffset = 110;

  display.fillRect(120, 75, 80, 80, WHITE);

  display.setCursor(xinitOffset, yinitOffset);
  display.print("Gx: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), yinitOffset);
  display.print(navigation->getGX(), 2);

  display.setCursor(xinitOffset, static_cast<int16_t>(yinitOffset + yOffset));
  display.print("Gy: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), static_cast<int16_t>(yinitOffset + yOffset));
  display.print(navigation->getGY(), 2);

  display.setCursor(xinitOffset, static_cast<int16_t>(yinitOffset + 2 * yOffset));
  display.print("Gz: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), static_cast<int16_t>(yinitOffset + 2 * yOffset));
  display.print(navigation->getGZ(), 2);

  /* GNSS */

  yinitOffset = 80;

  display.fillRect(202, 50, 200, 95, WHITE);

  xinitOffset = 230;
  xOffset = 50;

  display.setCursor(xinitOffset, yinitOffset);
  display.print("Lon: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), yinitOffset);
  const float lon = navigation->getPointA().lon;
  if (lon == 0) {
    display.print("-");
  } else {
    display.print(lon, 5);
  }

  display.setCursor(xinitOffset, static_cast<int16_t>(yinitOffset + yOffset));
  display.print("Lat: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), static_cast<int16_t>(yinitOffset + yOffset));
  const float lat = navigation->getPointA().lat;
  if (lat == 0) {
    display.print("-");
  } else {
    display.print(lat, 5);
  }

  /* MAG */

  display.fillRect(20, 180, 180, 55, WHITE);

  xinitOffset = 10;
  xOffset = 30;

  yinitOffset = 200;

  display.setCursor(xinitOffset, yinitOffset);
  display.print("Mx: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), yinitOffset);
  display.print(navigation->getMX() / 1000, 2);

  xinitOffset = 110;

  display.setCursor(xinitOffset, yinitOffset);
  display.print("My: ");

  display.setCursor(static_cast<int16_t>(xinitOffset + xOffset), yinitOffset);
  display.print(navigation->getMY() / 1000, 2);

  display.setCursor(70, static_cast<int16_t>(yinitOffset + yOffset));
  display.print("Mz: ");

  display.setCursor(static_cast<int16_t>(70 + xOffset), static_cast<int16_t>(yinitOffset + yOffset));
  display.print(navigation->getMZ() / 1000, 2);

  surface.present();
}

void Window::initSensorPrepareCalibrate() {
  display.fillRect(0, 19, 400, 222, WHITE);
  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Ready To Calibrate Compass", 200, 90);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(70, 130);
  display.print("Rotate the Ground Station slowly");
  display.setCursor(49, 150);
  display.print("in all directions, covering every angle.");
  display.setCursor(45, 180);
  display.print("When ready press A, to cancel press B.");

  surface.present();
}

void Window::initSensorCalibrate() {
  display.fillRect(0, 19, 400, 222, WHITE);
  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Calibrating Compass...", 200, 60);
  drawCentreString("Progress: 0.00%", 200, 160);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(82, 100);
  display.print("Rotate the Ground Station until");
  display.setCursor(95, 120);
  display.print("the progress shows 100%.");
  display.setCursor(130, 200);
  display.print("Press B to cancel.");

  surface.present();
}

void Window::updateSensorCalibrate(Navigation *navigation) {
  display.setFont(&FreeSansBold9pt7b);
  display.setTextSize(1);

  // Show Progress
  display.setTextColor(WHITE);
  String t = "Progress: " + String(oldCalibrationPercentage) + "%";
  drawCentreString(t, 200, 160);
  oldCalibrationPercentage = navigation->getCalibrationPercentage();
  display.setTextColor(BLACK);
  t = "Progress: " + String(oldCalibrationPercentage) + "%";
  drawCentreString(t, 200, 160);
}

void Window::initSensorCalibrateDone() {
  display.fillRect(0, 19, 400, 222, WHITE);
  display.setTextSize(1);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Compass Calibration Successful!", 200, 100);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(130, 160);
  display.print("Press A to continue.");

  surface.present();
}

void Window::initSettings(int16_t submenuIdx) {
  clearMainScreen();

  display.drawLine(0, 49, 400, 49, BLACK);

  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.fillRect(0, 19, 400, 30, BLACK);
  drawCentreString(settingPageName[submenuIdx], 200, 42);

  display.setTextColor(BLACK);
  for (int i = 0; i < settingsTableValueCount[submenuIdx]; i++) {
    addSettingEntry(i, &settingsTable[submenuIdx][i]);
  }

  drawSettingsTriangles(submenuIdx, WHITE);

  oldSettingsIndex = -1;
  subMenuSettingIndex = submenuIdx;

  display.drawLine(0, 177, 400, 177, BLACK);
  surface.present();
}

void Window::addSettingEntry(uint32_t settingIndex, const device_settings_t *setting, uint16_t color) {
  auto y = static_cast<int16_t>(75 + 30 * settingIndex);

  display.setTextColor(color);

  display.setCursor(10, y);
  display.print(setting->name);

  if (setting->type == TOGGLE) {
    const auto data = *static_cast<bool *>(setting->dataPtr);
    drawCentreString(lookup_tables[setting->config.lookup].values[static_cast<uint16_t>(data)], 305, y);

    y -= 23;
    if (!data) {
      display.fillTriangle(386, static_cast<int16_t>(y + 14), 378, static_cast<int16_t>(y + 6), 378,
                           static_cast<int16_t>(y + 22), color);
    } else {
      display.fillTriangle(224, static_cast<int16_t>(y + 14), 232, static_cast<int16_t>(y + 6), 232,
                           static_cast<int16_t>(y + 22), color);
    }
  } else if (setting->type == STRING) {
    display.setFont(&FreeMonoBold12pt7b);
    drawCentreString(static_cast<const char *>(setting->dataPtr), 285, y);
    display.setFont(&FreeSans12pt7b);
  } else if (setting->type == NUMBER) {
    char buffer[8];
    snprintf(buffer, 8, "%+d", *static_cast<int16_t *>(setting->dataPtr));
    drawCentreString(buffer, 305, y);

    y -= 23;
    if (setting->config.minmax.max == *static_cast<int16_t *>(setting->dataPtr)) {
      display.fillTriangle(386, static_cast<int16_t>(y + 14), 378, static_cast<int16_t>(y + 6), 378,
                           static_cast<int16_t>(y + 22), GetNegativeColor(color));
    } else {
      display.fillTriangle(386, static_cast<int16_t>(y + 14), 378, static_cast<int16_t>(y + 6), 378,
                           static_cast<int16_t>(y + 22), color);
    }

    if (setting->config.minmax.min == *static_cast<int16_t *>(setting->dataPtr)) {
      display.fillTriangle(224, static_cast<int16_t>(y + 14), 232, static_cast<int16_t>(y + 6), 232,
                           static_cast<int16_t>(y + 22), GetNegativeColor(color));
    } else {
      display.fillTriangle(224, static_cast<int16_t>(y + 14), 232, static_cast<int16_t>(y + 6), 232,
                           static_cast<int16_t>(y + 22), color);
    }
  }
}

void Window::updateSettings(int16_t index) {
  display.setTextSize(1);

  if (oldSettingsIndex >= 0) {
    if (oldSettingsIndex != index) {
      highlightSetting(oldSettingsIndex, BLACK, false);
    }
  } else {
    drawSettingsTriangles(subMenuSettingIndex, BLACK);
  }

  if (index >= 0) {
    highlightSetting(index, WHITE, true);
  } else {
    drawSettingsTriangles(subMenuSettingIndex, WHITE);
    display.fillRect(0, 178, 400, 62, WHITE);
  }

  oldSettingsIndex = index;
  surface.present();
}

void Window::drawSettingsTriangles(int16_t submenuIdx, int16_t color) {
  if (submenuIdx == 0) {  // first page
    display.fillTriangle(386, 33, 378, 25, 378, 41, color);
  } else if (submenuIdx == kSettingPages - 1) {  // last page
    display.fillTriangle(13, 33, 21, 25, 21, 41, color);
  } else {  // other pages
    display.fillTriangle(386, 33, 378, 25, 378, 41, color);
    display.fillTriangle(13, 33, 21, 25, 21, 41, color);
  }
}

void Window::highlightSetting(int16_t index, uint16_t color, bool updateDescription) {
  display.setFont(&FreeSans12pt7b);
  const auto yPos = static_cast<int16_t>(52 + 30 * index);
  display.fillRect(0, yPos, 400, 30, GetNegativeColor(color));
  addSettingEntry(index, &settingsTable[subMenuSettingIndex][index], color);

  if (!updateDescription) {
    return;
  }

  display.fillRect(0, 178, 400, 62, WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BLACK);
  display.setCursor(10, 195);
  display.print(settingsTable[subMenuSettingIndex][index].description1);
  display.setCursor(10, 215);
  display.print(settingsTable[subMenuSettingIndex][index].description2);
}

const uint8_t kNumKeyboardChars = 38;
const uint32_t kBackspaceCoordX = 330;
const uint32_t kBackspaceCoordY = 62;

const int16_t keybXY[kNumKeyboardChars][2] = {
    {20, 125},   //'1'
    {60, 125},   //'2'
    {100, 125},  //'3'
    {140, 125},  //'4'
    {180, 125},  //'5'
    {220, 125},  //'6'
    {260, 125},  //'7'
    {300, 125},  //'8'
    {340, 125},  //'9'
    {380, 125},  //'0' ---
    {20, 155},   //'Q'
    {60, 155},   //'W'
    {100, 155},  //'E'
    {140, 155},  //'R'
    {180, 155},  //'T'
    {220, 155},  //'Y'
    {260, 155},  //'U'
    {300, 155},  //'I'
    {340, 155},  //'O'
    {380, 155},  //'P' ---
    {40, 185},   //'A'
    {80, 185},   //'S'
    {120, 185},  //'D'
    {160, 185},  //'F'
    {200, 185},  //'G'
    {240, 185},  //'H'
    {280, 185},  //'J'
    {320, 185},  //'K'
    {360, 185},  //'L' ---
    {20, 215},   //'SHIFT'
    {60, 215},   //'Z'
    {100, 215},  //'X'
    {140, 215},  //'C'
    {180, 215},  //'V'
    {220, 215},  //'B'
    {260, 215},  //'N'
    {300, 215},  //'M'
    {340, 215},  //'_'
};

// clang-format off
const char keybChar[kNumKeyboardChars] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 
                                          'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P',
                                             'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', 
                                          ' ', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '_'};
// clang-format on

void Window::initKeyboard(char *text, uint32_t maxLength) {
  if (maxLength != 0) {
    keyboardTextMaxLength = maxLength;
  }
  clearMainScreen();

  display.setTextSize(1);
  updateKeyboardText(text, BLACK);

  display.setFont();
  display.setTextSize(2);

  for (int i = 0; i < kNumKeyboardChars; i++) {
    if (i == oldKey) {
      highlightKeyboardKey(i, BLACK);
    } else if (i != kShiftIdx) {
      if (!upperCase && i > 9 && i != kUnderscoreIdx) {
        display.drawChar(keybXY[i][0], keybXY[i][1], keybChar[i] + 32, BLACK, WHITE, 2);
      } else {
        display.drawChar(keybXY[i][0], keybXY[i][1], keybChar[i], BLACK, WHITE, 2);
      }
    }
  }

  if (oldKey != kShiftIdx) {
    display.drawBitmap(static_cast<int16_t>(keybXY[kShiftIdx][0] - 4), static_cast<int16_t>(keybXY[kShiftIdx][1] - 1),
                       shift_keyboard, 16, 16, BLACK);
  }
  if (oldKey != -1) {
    display.drawBitmap(kBackspaceCoordX, kBackspaceCoordY, backspace_keyboard, 24, 24, BLACK);
  } else {
    highlightKeyboardKey(-1, BLACK);
  }

  surface.present();
}

void Window::drawKeyboard(char *text, int32_t keyHighlight, bool uppercase, uint32_t maxLength) {
  oldKey = keyHighlight;
  upperCase = uppercase;
  initKeyboard(text, maxLength);
}

void Window::updateKeyboard(char *text, int32_t keyHighlight, bool keyPressed) {
  display.setFont(&FreeSans12pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);

  if (keyPressed) {
    if (keyHighlight == kShiftIdx) {  // SHIFT
      upperCase = !upperCase;
      initKeyboard(text);
    } else if (keyHighlight == -1) {  // BACKSPACE
      updateKeyboardText(text, WHITE);
      if (strlen(text) > 0) {
        text[strlen(text) - 1] = 0;
      }
      updateKeyboardText(text, BLACK);
    } else {  // KEY
      if (strlen(text) < keyboardTextMaxLength) {
        updateKeyboardText(text, WHITE);
        if (keyHighlight > 9 && keyHighlight != kUnderscoreIdx) {
          text[strlen(text)] = static_cast<char>(keybChar[keyHighlight] + static_cast<int>(!upperCase) * 32);
        } else {
          text[strlen(text)] = keybChar[keyHighlight];
        }
        updateKeyboardText(text, BLACK);
      }
    }
  }

  display.setFont();
  display.setTextSize(2);

  highlightKeyboardKey(oldKey, WHITE);

  highlightKeyboardKey(keyHighlight, BLACK);

  oldKey = keyHighlight;
  surface.present();
}

void Window::highlightKeyboardKey(int32_t key, uint16_t color) {
  if (key == -1) {
    display.fillCircle(kBackspaceCoordX + 12, kBackspaceCoordY + 11, 16, color);
    display.drawBitmap(kBackspaceCoordX, kBackspaceCoordY, backspace_keyboard, 24, 24, GetNegativeColor(color));
  } else {
    display.fillCircle(static_cast<int16_t>(keybXY[key][0] + 4), static_cast<int16_t>(keybXY[key][1] + 7), 16, color);
  }

  if (key == kShiftIdx) {
    display.drawBitmap(static_cast<int16_t>(keybXY[kShiftIdx][0] - 4), static_cast<int16_t>(keybXY[kShiftIdx][1] - 1),
                       shift_keyboard, 16, 16, GetNegativeColor(color));
  } else {
    if (!upperCase && key > 9 && key != kUnderscoreIdx) {
      display.drawChar(keybXY[key][0], keybXY[key][1], keybChar[key] + 32, GetNegativeColor(color), color, 2);
    } else {
      display.drawChar(keybXY[key][0], keybXY[key][1], keybChar[key], GetNegativeColor(color), color, 2);
    }
  }
}

void Window::updateKeyboardText(char *text, uint16_t color) {
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(color);
  display.setCursor(90, 80);
  display.print(text);
  display.setFont(&FreeSans12pt7b);
}

/**
 * Clears everything except the status bar.
 */
void Window::clearMainScreen() { display.fillRect(0, 19, 400, 222, WHITE); }

void Window::listFileName(const char *fileName, uint16_t index, uint16_t color) {
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(color);
  display.setCursor(10, static_cast<int16_t>(33 + 20 * index));
  display.print(fileName);
}

void Window::dataHighlight(const char *fileName, uint16_t index, bool highlight) {
  display.fillRect(0, static_cast<int16_t>(19 + 20 * index), 400, 20, highlight ? BLACK : WHITE);
  listFileName(fileName, index, highlight ? WHITE : BLACK);
}

void Window::dataShowFlightStatistics(FlightStatistics &stats1, FlightStatistics &stats2, const char *logName,
                                      bool hasNextPage) {
  clearMainScreen();
  display.setTextColor(BLACK);
  display.setTextSize(1);

  std::string visibleName = logName == nullptr ? "" : logName;
  // Reassigned below when the visible filename needs an ellipsis.
  // NOLINTNEXTLINE(misc-const-correctness)
  std::string title = visibleName;
  int16_t boundsX = 0;
  int16_t boundsY = 0;
  uint16_t boundsWidth = 0;
  uint16_t boundsHeight = 0;
  display.setFont(&FreeSans12pt7b);
  display.getTextBounds(title.c_str(), 0, 0, &boundsX, &boundsY, &boundsWidth, &boundsHeight);
  while (boundsWidth > 330U && !visibleName.empty()) {
    visibleName.pop_back();
    title = visibleName + "...";
    display.getTextBounds(title.c_str(), 0, 0, &boundsX, &boundsY, &boundsWidth, &boundsHeight);
  }

  drawPageHeader(title.c_str(), false, hasNextPage);
  display.setFont(&FreeSans12pt7b);
  display.setTextColor(BLACK);

  display.drawLine(199, 49, 199, 240, BLACK);
  display.drawLine(200, 49, 200, 240, BLACK);

  dataShowFlightStatisticsSide(stats1, 0);
  dataShowFlightStatisticsSide(stats2, 1);
  drawVerticalNavigationTile(false);

  surface.present();
}

void Window::dataShowFlightStatisticsSide(FlightStatistics &stats, uint16_t index) {
  const auto xOffset = static_cast<int16_t>(index * 200);

  // Max altitude
  display.drawBitmap(static_cast<int16_t>(xOffset + 9), 50, data_altitude_peak, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 69);
  const int32_t altitude_m = stats.getMaxAltitude();
  if (!stats.hasMaxAltitude()) {
    display.print("--");
  } else if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(altitude_m);
    display.print(" m");
  } else {
    display.print(Utils::MetersToFeet(altitude_m));
    display.print(" ft");
  }

  // Time to apogee
  display.drawBitmap(static_cast<int16_t>(xOffset + 9), 73, data_altitude_time, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 92);
  if (stats.hasTimeToApogee()) {
    display.print(stats.getTimeToApogee(), 1);
    display.print(" s");
  } else {
    display.print("--");
  }

  // Max speed
  display.drawBitmap(static_cast<int16_t>(xOffset + 5), 96, data_speed, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 115);
  const int32_t velocity_ms = stats.getMaxVelocity();
  if (!stats.hasMaxVelocity()) {
    display.print("--");
  } else if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(velocity_ms);
    display.print(" m/s");
  } else {
    display.print(Utils::MetersToFeet(velocity_ms));
    display.print(" ft/s");
  }

  // Drogue descent rate
  display.drawBitmap(static_cast<int16_t>(xOffset + 8), 119, data_drogue_speed, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 138);
  const float drogue_velocity_ms = stats.getDrogueDescentRate();
  if (!stats.hasDrogueDescentRate()) {
    display.print("--");
  } else if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(drogue_velocity_ms, 1);
    display.print(" m/s");
  } else {
    display.print(Utils::MetersToFeet(drogue_velocity_ms), 1);
    display.print(" ft/s");
  }

  // Main descent rate
  display.drawBitmap(static_cast<int16_t>(xOffset + 5), 142, data_main_speed, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 161);
  const float main_velocity_ms = stats.getMainDescentRate();
  if (!stats.hasMainDescentRate()) {
    display.print("--");
  } else if (config.config.unitSystem == UnitSystem::kMetric) {
    display.print(main_velocity_ms, 1);
    display.print(" m/s");
  } else {
    display.print(Utils::MetersToFeet(main_velocity_ms), 1);
    display.print(" ft/s");
  }

  // Latitude
  display.drawBitmap(static_cast<int16_t>(xOffset + 5), 165, live_lat, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 184);
  if (stats.hasLastLocation()) {
    display.print(stats.getLastLatitude(), 4);
    display.print(" N");
  } else {
    display.print("--");
  }

  // Longitude
  display.drawBitmap(static_cast<int16_t>(xOffset + 5), 188, live_lon, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 207);
  if (stats.hasLastLocation()) {
    display.print(stats.getLastLongitude(), 4);
    display.print(" E");
  } else {
    display.print("--");
  }

  // Flight Time
  display.drawBitmap(static_cast<int16_t>(xOffset + 5), 211, data_flight_time, 24, 24, BLACK);
  display.setCursor(static_cast<int16_t>(xOffset + 45), 230);
  if (stats.hasFlightTime()) {
    display.print(stats.getFlightTime(), 1);
    display.print(" s");
  } else {
    display.print("--");
  }
}
