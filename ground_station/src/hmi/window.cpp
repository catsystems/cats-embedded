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
#include <cstdio>
#include <cstring>

uint16_t GetNegativeColor(uint16_t color) {
  if (color == BLACK) {
    return WHITE;
  }
  return BLACK;
}

void Window::drawCoordinate(float value, char positiveHemisphere, char negativeHemisphere, int16_t x, int16_t baseline,
                            uint16_t color) {
  display.setTextColor(color);
  display.setCursor(x, baseline);
  display.print(std::fabs(value), 4);
  display.print(' ');
  display.print(value < 0.0F ? negativeHemisphere : positiveHemisphere);
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
  livestate = LiveState::kShowGnss;
  std::memset(liveNavigationText, 0, sizeof(liveNavigationText));
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
  constexpr int16_t kScaleNumerator = 6;
  constexpr int16_t kScaleDenominator = 5;
  const auto transform = [centerX, centerY](int16_t forward, int16_t sideways) -> Point {
    forward = static_cast<int16_t>((forward * kScaleNumerator) / kScaleDenominator);
    sideways = static_cast<int16_t>((sideways * kScaleNumerator) / kScaleDenominator);
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
  const Point flameBaseLeft = transform(-15, -4);
  const Point flameBaseRight = transform(-15, 4);
  const Point flameTip = transform(-38, flameWobble);
  const Point flameShoulder = transform(-22, 0);
  const Point flameForkLeft = transform(-29, static_cast<int16_t>(-7 - flameWobble));
  const Point flameForkRight = transform(-28, static_cast<int16_t>(7 - flameWobble));
  display.fillTriangle(flameBaseLeft.x, flameBaseLeft.y, flameBaseRight.x, flameBaseRight.y, flameTip.x, flameTip.y,
                       BLACK);
  display.fillTriangle(flameBaseLeft.x, flameBaseLeft.y, flameShoulder.x, flameShoulder.y, flameForkLeft.x,
                       flameForkLeft.y, BLACK);
  display.fillTriangle(flameBaseRight.x, flameBaseRight.y, flameShoulder.x, flameShoulder.y, flameForkRight.x,
                       flameForkRight.y, BLACK);

  const Point innerFlameLeft = transform(-17, -2);
  const Point innerFlameRight = transform(-17, 2);
  const Point innerFlameTip = transform(-29, flameWobble / 2);
  display.fillTriangle(innerFlameLeft.x, innerFlameLeft.y, innerFlameRight.x, innerFlameRight.y, innerFlameTip.x,
                       innerFlameTip.y, WHITE);
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
  // Full-screen modes draw over this area. Rebuild both its background and
  // cached values when returning to the normal status bar.
  display.fillRect(0, 0, 400, 19, WHITE);
  barNeedsRedraw = true;
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
  if (barNeedsRedraw || logging != oldBarLoggingStatus || recorderFault != oldBarRecorderFault) {
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
  if (barNeedsRedraw || free_memory != oldBarFreeMemory) {
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

  if ((barNeedsRedraw || clock.minute() != oldBarMinute || clock.hour() != oldBarHour) && time) {
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
  if (barNeedsRedraw || usb != oldBarUsbStatus) {
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
  barNeedsRedraw = false;

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
  livestate = LiveState::kShowGnss;
  std::memset(liveNavigationText, 0, sizeof(liveNavigationText));

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
  if (livestate == state) return;
  const bool showGnss = state == LiveState::kShowGnss;
  TelemetryData data[] = {*data1, *data2};  // Presentation getters only consume these copies.
  for (int16_t index = 0; index < 2; ++index) {
    const int16_t x = index * 200;
    display.fillRect(x + 28, 99, 130, 50, WHITE);
    display.fillRect(x + 3, 100, 24, 49, WHITE);
    display.fillRect(x + 158, 100, 24, 49, WHITE);
    display.drawBitmap(x + 3, 100, showGnss ? live_lat : down_range, 24, 24, BLACK);
    display.drawBitmap(x + 3, 125, showGnss ? live_lon : compass, 24, 24, BLACK);
    display.drawBitmap(x + 158, 100, showGnss ? right_arrow : left_arrow, 24, 24, BLACK);
    display.drawBitmap(x + 158, 125, showGnss ? right_arrow : left_arrow, 24, 24, BLACK);
    if (showGnss && !data[index].testingMode()) {
      display.setFont(&FreeSans12pt7b);
      drawCoordinate(data[index].lat(), 'N', 'S', x + 28, 120, BLACK);
      drawCoordinate(data[index].lon(), 'E', 'W', x + 28, 145, BLACK);
    }
  }
  livestate = state;
  std::memset(liveNavigationText, 0, sizeof(liveNavigationText));
  updateLiveNavigation(*data1, *data2, navigation);
  surface.present();
}

bool Window::updateLiveNavigation(const TelemetryData &data1, const TelemetryData &data2, Navigation *navigation) {
  if (livestate != LiveState::kShowDownRange) return false;
  // Copy before using presentation getters, preserving incoming packet flags.
  TelemetryData data[] = {data1, data2};
  const EarthPoint3D targets[] = {EarthPoint3D(data[0].lat(), data[0].lon()),
                                  EarthPoint3D(data[1].lat(), data[1].lon())};
  const bool validTarget[] = {LocationQr::IsValid(targets[0].lat, targets[0].lon),
                              LocationQr::IsValid(targets[1].lat, targets[1].lon)};
  const auto home = navigation->getPointA();
  const bool validHome = LocationQr::IsValid(home.lat, home.lon);
  const float north = navigation->getNorth();
  // Single mode tracks one rocket through the most recent receiver with GPS.
  size_t singleTarget = data1.getLastUpdateTime() > data2.getLastUpdateTime() ? 0 : 1;
  if (!validTarget[singleTarget]) singleTarget = 1 - singleTarget;
  bool changed = false;
  for (size_t index = 0; index < 2; ++index) {
    const size_t targetIndex = config.config.receiverMode == DUAL ? index : singleTarget;
    char values[2][24]{};
    if (validHome && validTarget[targetIndex] && !data[index].testingMode()) {
      const auto target = targets[targetIndex];
      constexpr float radians = PI_F / 180.0F;
      constexpr float earthRadiusM = 6378100.0F;
      const float dy = (target.lat - home.lat) * radians * earthRadiusM;
      const float dx =
          std::remainder(target.lon - home.lon, 360.0F) * radians * std::cos(home.lat * radians) * earthRadiusM;
      const auto distance = static_cast<int32_t>(std::round(std::hypot(dx, dy)));
      const bool metric = config.config.unitSystem == UnitSystem::kMetric;
      std::snprintf(values[0], sizeof(values[0]), "%ld %s",
                    static_cast<long>(metric ? distance : Utils::MetersToFeet(distance)), metric ? "m" : "ft");
      // Screen drawing uses a -90 degree offset; a turn angle starts straight ahead at zero.
      if ((dx != 0 || dy != 0) && std::isfinite(north)) {
        float bearing = std::round(std::remainder((std::atan2(dx, dy) + north) / radians, 360.0F) * 10.0F) / 10.0F;
        if (bearing <= -180.0F) bearing = 180.0F;
        if (bearing == 0) bearing = 0.0F;  // Avoid displaying negative zero after rounding.
        std::snprintf(values[1], sizeof(values[1]), "%.1f", static_cast<double>(bearing));
      }
    }
    for (size_t row = 0; row < 2; ++row) {
      if (std::strcmp(values[row], liveNavigationText[index][row]) == 0) continue;
      const auto x = static_cast<int16_t>(index * 200 + 28);
      const auto y = static_cast<int16_t>(99 + row * 25);
      display.fillRect(x, y, 130, 25, WHITE);
      display.setFont(&FreeSans12pt7b);
      display.setTextSize(1);
      display.setTextColor(BLACK);
      display.setCursor(x, y + 21);
      display.print(values[row]);
      if (row == 1 && values[row][0] != '\0') {
        // Keep the larger degree symbol inside the cleared row.
        display.drawCircle(static_cast<int16_t>(display.getCursorX() + 4), y + 6, 3, BLACK);
      }
      std::strcpy(liveNavigationText[index][row], values[row]);
      changed = true;
    }
  }
  return changed;
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

  updateLiveData(&teleData[index], index, WHITE);
  updateLiveInfo(&infoData[index], index, BLACK);

  // display.fillRect(10,19,190,200, WHITE);

  memcpy(&teleData[index], data, sizeof(teleData[0]));
  memcpy(&infoData[index], info, sizeof(infoData[0]));

  dataAge[index] = 0;

  updateLiveData(&teleData[index], index, BLACK);
  updateLiveNavigation(teleData[0], teleData[1], navigation);
  updateLiveInfo(&infoData[index], index, WHITE);
}

void Window::updateLive(TelemetryData *data, Navigation *navigation, int16_t index) {
  if (index > 1) {
    return;
  }

  lastTeleData[index] = static_cast<uint32_t>(clock.nowMs());

  // Clear update flag
  data->clear();

  updateLiveData(&teleData[index], index, WHITE);

  // display.fillRect(10,19,190,200, WHITE);

  memcpy(&teleData[index], data, sizeof(teleData[0]));

  dataAge[index] = 0;

  updateLiveData(&teleData[index], index, BLACK);
  updateLiveNavigation(teleData[0], teleData[1], navigation);
}

const char *const stateName[] = {"INVALID", "CALIB", "READY", "THRUST", "COAST", "DROGUE", "MAIN", "DOWN"};

const char *const errorName[] = {"No Config",   "Log Full",         "Filter Error",
                                 "Overheating", "Continuity Error", "Calibration Error"};

void Window::updateLiveData(TelemetryData *data, int16_t index, uint16_t color) {
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
    drawCoordinate(data->lat(), 'N', 'S', static_cast<int16_t>(xOffset + first_row_offset), 120, color);
    drawCoordinate(data->lon(), 'E', 'W', static_cast<int16_t>(xOffset + first_row_offset), 145, color);
  }

  display.setCursor(static_cast<int16_t>(xOffset + first_row_offset), 170);
  display.print(data->voltage(), 1);
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

void Window::initFirmwareUpdate(int16_t index) {
  clearMainScreen();
  drawPageHeader("Update Firmware", false, false);

  const char *names[] = {"Ground Station", "Radio Receivers"};
  for (int16_t row = 0; row < 2; ++row) {
    const int16_t y = static_cast<int16_t>(52 + 30 * row);
    display.fillRect(0, y, 400, 30, row == index ? BLACK : WHITE);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(row == index ? WHITE : BLACK);
    display.setCursor(10, y + 23);
    display.print(names[row]);
  }

  display.drawLine(0, 112, 400, 112, BLACK);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BLACK);
  if (index == 0) {
    display.setCursor(10, 143);
    display.print("Restart into the USB updater");
    display.setCursor(10, 163);
    display.print("Connect to a computer before continuing");
  } else {
    display.setCursor(10, 143);
    display.print("Install a telemetry .bin on both receivers");
    display.setCursor(10, 163);
    display.print("Copy it to /telemetry_firmware and eject USB");
  }
  display.setCursor(10, 230);
  display.print("Back (B)");
  display.setCursor(260, 230);
  display.print("Select (A)");
  surface.present();
}

void Window::initRadioUpdateList() {
  clearMainScreen();
  drawPageHeader("Update GS Radios", false, false);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(BLACK);
  display.setCursor(10, 230);
  display.print("Back (B)");
  display.setCursor(260, 230);
  display.print("Select (A)");
}

void Window::radioUpdateFileName(const char *filename, uint8_t row, bool selected) {
  const int16_t y = static_cast<int16_t>(50 + 20 * row);
  display.fillRect(0, y, 400, 20, selected ? BLACK : WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(selected ? WHITE : BLACK);
  display.setCursor(10, y + 15);
  display.print(filename);
}

void Window::initRadioUpdateConfirm(const char *filename, uint32_t size, uint32_t crc) {
  clearMainScreen();
  drawPageHeader("Confirm Radio Update", false, false);
  char visibleName[43]{};
  strncpy(visibleName, filename, sizeof(visibleName) - 1);
  if (strlen(filename) >= sizeof(visibleName)) {
    memcpy(visibleName + sizeof(visibleName) - 5, "...", 4);
  }
  display.setTextColor(BLACK);
  display.setFont(&FreeSans9pt7b);
  drawCentreString(visibleName, 200, 70);
  char identity[64]{};
  snprintf(identity, sizeof(identity), "%lu bytes  CRC32 %08lX", static_cast<unsigned long>(size),
           static_cast<unsigned long>(crc));
  drawCentreString(identity, 200, 91);
  drawCentreString("Only use production telemetry firmware", 200, 117);
  drawCentreString("intended for this Ground Station.", 200, 137);
  drawCentreString("Wrong firmware may require ST-Link recovery.", 200, 157);
  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("Do not disconnect power during this update.", 200, 184);
  display.setFont(&FreeSans9pt7b);
  drawCentreString("An interruption may require ST-Link recovery.", 200, 204);
  display.setCursor(10, 230);
  display.print("Back (B)");
  display.setCursor(263, 230);
  display.print("Install (A)");
  surface.present();
}

void Window::radioUpdateProgress(const char *phase, uint8_t link, uint8_t percent) {
  clearMainScreen();
  drawPageHeader("Updating GS Radios", false, false);
  display.setTextColor(BLACK);
  display.setFont(&FreeSansBold12pt7b);
  char label[64]{};
  if (link > 0) {
    snprintf(label, sizeof(label), "Link %u: %s", link, phase);
  } else {
    strncpy(label, phase, sizeof(label) - 1);
  }
  drawCentreString(label, 200, 105);
  display.drawRect(38, 132, 324, 26, BLACK);
  display.fillRect(42, 136, static_cast<int16_t>(316U * percent / 100U), 18, BLACK);
  display.setFont(&FreeSans9pt7b);
  snprintf(label, sizeof(label), "%u%%", percent);
  drawCentreString(label, 200, 184);
  drawCentreString("Do not disconnect power", 200, 224);
  surface.present();
}

void Window::radioUpdateResult(bool success, const char *message, const char *version1, const char *version2) {
  clearMainScreen();
  drawPageHeader(success ? "Radio Update Complete" : "Radio Update Stopped", false, false);
  display.setTextColor(BLACK);
  display.setFont(&FreeSansBold12pt7b);
  drawCentreString(success ? "Both radios verified" : "See recovery status below", 200, 88);
  display.setFont(&FreeSans9pt7b);
  drawCentreString(message, 200, 118);
  char value[80]{};
  snprintf(value, sizeof(value), "Link 1: %s", version1[0] != '\0' ? version1 : "not updated");
  drawCentreString(value, 200, 146);
  snprintf(value, sizeof(value), "Link 2: %s", version2[0] != '\0' ? version2 : "not updated");
  drawCentreString(value, 200, 168);
  display.setCursor(10, 230);
  display.print("Back (B)");
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
  display.setCursor(12, 225);
  display.print("Right: compass");

  surface.present();
}

void Window::initSensorOrientation() {
  clearMainScreen();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setFont(&FreeSansBold12pt7b);
  display.fillRect(0, 19, 400, 30, BLACK);
  drawCentreString("Compass / 3D Orientation", 200, 42);
  display.setTextColor(BLACK);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(8, 233);
  display.print("Left: sensors");
  display.setCursor(285, 233);
  display.print("A: calibrate");

  surface.present();
}

void Window::updateSensorOrientation(Navigation *navigation) {
  constexpr int16_t centerX = 105;
  constexpr int16_t centerY = 132;
  constexpr int16_t radius = 60;
  float northAngle = navigation->getNorth();
  while (northAngle < 0.0F) {
    northAngle += 2.0F * PI_F;
  }
  while (northAngle >= 2.0F * PI_F) {
    northAngle -= 2.0F * PI_F;
  }
  // The north pointer rotates opposite to the clockwise heading of the device.
  const float headingDegrees = (northAngle > 0.0F ? 2.0F * PI_F - northAngle : 0.0F) * 180.0F / PI_F;
  static constexpr const char *directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  const uint8_t direction = static_cast<uint8_t>((headingDegrees + 22.5F) / 45.0F) & 7U;

  display.fillRect(0, 49, 400, 163, WHITE);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.drawCircle(centerX, centerY, radius, BLACK);

  for (uint8_t index = 0; index < 8; ++index) {
    const float angle = static_cast<float>(index) * PI_F / 4.0F;
    const float tickDepth = index % 2U == 0U ? 5.0F : 3.0F;
    const int16_t outerX = static_cast<int16_t>(centerX + sinf(angle) * radius);
    const int16_t outerY = static_cast<int16_t>(centerY - cosf(angle) * radius);
    const int16_t innerX = static_cast<int16_t>(centerX + sinf(angle) * (radius - tickDepth));
    const int16_t innerY = static_cast<int16_t>(centerY - cosf(angle) * (radius - tickDepth));
    display.drawLine(innerX, innerY, outerX, outerY, BLACK);
  }

  display.setFont(&FreeSansBold9pt7b);
  drawCentreString("N", centerX, 68);
  drawCentreString("S", centerX, 208);
  drawCentreString("W", 31, 139);
  drawCentreString("E", 179, 139);

  const float sine = sinf(northAngle);
  const float cosine = cosf(northAngle);
  const int16_t northTipX = static_cast<int16_t>(centerX + sine * 50.0F);
  const int16_t northTipY = static_cast<int16_t>(centerY - cosine * 50.0F);
  const int16_t northLeftX = static_cast<int16_t>(centerX - cosine * 6.0F);
  const int16_t northLeftY = static_cast<int16_t>(centerY - sine * 6.0F);
  const int16_t northRightX = static_cast<int16_t>(centerX + cosine * 6.0F);
  const int16_t northRightY = static_cast<int16_t>(centerY + sine * 6.0F);
  const int16_t southTipX = static_cast<int16_t>(centerX - sine * 30.0F);
  const int16_t southTipY = static_cast<int16_t>(centerY + cosine * 30.0F);
  display.drawLine(centerX, centerY, southTipX, southTipY, BLACK);
  display.fillTriangle(northTipX, northTipY, northLeftX, northLeftY, northRightX, northRightY, BLACK);
  display.fillCircle(centerX, centerY, 4, WHITE);
  display.drawCircle(centerX, centerY, 4, BLACK);
  display.fillCircle(centerX, centerY, 1, BLACK);

  display.drawFastVLine(195, 62, 137, BLACK);

  display.setFont(&FreeSans9pt7b);
  display.setCursor(215, 78);
  display.print("Mag heading");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(215, 105);
  display.print(headingDegrees, 1);
  display.print(" deg ");
  display.print(directions[direction]);

  display.setFont(&FreeSans9pt7b);
  display.setCursor(215, 137);
  display.print("Pitch: ");
  display.print(navigation->getPitch() * 180.0F / PI_F, 1);
  display.print(" deg");
  display.setCursor(215, 164);
  display.print("Roll:  ");
  display.print(navigation->getRoll() * 180.0F / PI_F, 1);
  display.print(" deg");
  const float magneticMagnitude =
      sqrtf(navigation->getMX() * navigation->getMX() + navigation->getMY() * navigation->getMY() +
            navigation->getMZ() * navigation->getMZ()) /
      1000.0F;
  display.setCursor(215, 191);
  display.print("|M|:   ");
  display.print(magneticMagnitude, 2);

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

  display.fillRect(0, 140, 400, 35, WHITE);
  display.setTextColor(BLACK);
  String t = "Progress: " + String(navigation->getCalibrationPercentage()) + "%";
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
    if (settingsTable[subMenuSettingIndex][index].dataPtr == &systemConfig.config.receiverMode) {
      for (int16_t row = 0; row < settingsTableValueCount[subMenuSettingIndex]; ++row) {
        if (settingsTable[subMenuSettingIndex][row].dataPtr == systemConfig.config.linkPhrase2) {
          highlightSetting(row, BLACK, false);
        }
      }
    }
    highlightSetting(index, WHITE, true);
  } else {
    drawSettingsTriangles(subMenuSettingIndex, WHITE);
    display.fillRect(0, 178, 400, 62, WHITE);
  }

  oldSettingsIndex = index;
  // The HMI fills in cached versions before presenting this selection.
  if (index >= 0 && settingsTable[subMenuSettingIndex][index].type == BUTTON &&
      settingsTable[subMenuSettingIndex][index].config.buttonAction == BUTTON_ACTION_VERSION)
    return;
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

  const auto &setting = settingsTable[subMenuSettingIndex][index];
  if (setting.type == BUTTON && setting.config.buttonAction == BUTTON_ACTION_VERSION) {
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

void Window::settingsVersions(const char *telemetry1, const char *telemetry2) {
  display.fillRect(0, 178, 400, 62, WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  const char *labels[] = {"Ground Station:", "Telemetry 1:", "Telemetry 2:"};
  const char *versions[] = {FIRMWARE_VERSION, telemetry1, telemetry2};
  for (int16_t row = 0; row < 3; ++row) {
    const auto y = static_cast<int16_t>(193 + row * 20);
    display.setCursor(10, y);
    display.print(labels[row]);
    display.setCursor(165, y);
    display.print(versions[row]);
  }
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
