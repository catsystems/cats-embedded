/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "console.hpp"
#include "hmi/hmi.hpp"
#include "logging/recorder.hpp"
#include "navigation.hpp"
#include "telemetry/telemetry.hpp"
#include "utils.hpp"

#include <Arduino.h>

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables,cppcoreguidelines-interfaces-global-init)
Utils utils;
Hmi hmi("/logs");

Telemetry link1(Serial, 8, 9);
Telemetry link2(Serial1, 11, 12);

Navigation navigation;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables,cppcoreguidelines-interfaces-global-init)

void setup() {
  pinMode(21, INPUT);

  console.begin();
#ifdef CATS_DEBUG
  console.setLevel(Console::ConsoleLevel::LEVEL_OK);
#else
  console.setLevel(Console::ConsoleLevel::LEVEL_LOG);
#endif

  if (!utils.begin(0, "DRIVE")) {
    console.warning.println("[MAIN] Could not initialize utilities");
  }

  systemConfig.load();

  link1.begin();
  link2.begin();

  navigation.setPointA(0, 0);
  navigation.setPointB(0, 0);

  hmi.begin();
}

void loop() {
  static bool ini{false};
  static uint16_t link1LastTs{0};
  static uint16_t link2LastTs{0};

  if (!ini && millis() > 5000) {
    ini = true;
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

    navigation.begin();
  }

  // Update the home location
  if (link2.location.isUpdated()) {
    navigation.setPointA(link2.location.lat(), link2.location.lon());
  }

  // In single mode, both antennas track the same rocket
  if (systemConfig.config.receiverMode == SINGLE) {
    const bool link1GpsValid = (link1.data.lat() != 0) && (link1.data.lon() != 0);
    const bool link2GpsValid = (link2.data.lat() != 0) && (link2.data.lon() != 0);
    // Check if data from link 1 is newer than link 2
    if (link1.data.getLastUpdateTime() > link2.data.getLastUpdateTime()) {
      // Stream data from Link 1
      if (link1LastTs != link1.data.ts()) {
        Utils::streamUsb(&link1, 1);
      }

      // Take data from link 1 with higher priority
      if (link1GpsValid) {
        navigation.setPointB(link1.data.lat(), link1.data.lon());
      } else if (link2GpsValid) {
        navigation.setPointB(link2.data.lat(), link2.data.lon());
      }
    } else {
      // Stream data from Link 2
      if (link2LastTs != link2.data.ts()) {
        Utils::streamUsb(&link2, 2);
      }

      // Take data from link 2 with higher priority
      if (link2GpsValid) {
        navigation.setPointB(link2.data.lat(), link2.data.lon());
      } else if (link1GpsValid) {
        navigation.setPointB(link1.data.lat(), link1.data.lon());
      }
    }
  } else {
    // Stream both Links as they track different FC's
    if (link1LastTs != link1.data.ts()) {
      Utils::streamUsb(&link1, 1);
    }
    if (link2LastTs != link2.data.ts()) {
      Utils::streamUsb(&link2, 2);
    }

    if (link1.data.lat() != 0 && link1.data.lon() != 0) {
      navigation.setPointB(link1.data.lat(), link1.data.lon());
    }
  }

  // Update last timestamp from the link
  link1LastTs = link1.data.ts();
  link2LastTs = link2.data.ts();

  delay(100);
}
