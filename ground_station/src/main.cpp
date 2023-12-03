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

  if (!utils.begin()) {
    console.error.println("[MAIN] Could not initialize utilities");
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
  if (millis() > 5000 && !ini) {
    ini = true;
    if (systemConfig.config.receiverMode == ReceiverTelemetryMode::kSingle) {
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
  if (systemConfig.config.receiverMode == ReceiverTelemetryMode::kSingle) {
    const bool link1DataValid = (link1.data.lat() != 0) && (link1.data.lon() != 0);
    const bool link2DataValid = (link2.data.lat() != 0) && (link2.data.lon() != 0);
    // Check if data from link 1 is newer than link 2
    if (link1.data.getLastUpdateTime() > link2.data.getLastUpdateTime()) {
      // Take data from link 1 with higher priority
      if (link1DataValid) {
        navigation.setPointB(link1.data.lat(), link1.data.lon());
      } else if (link2DataValid) {
        navigation.setPointB(link2.data.lat(), link2.data.lon());
      }
    } else {
      // Take data from link 2 with higher priority
      if (link2DataValid) {
        navigation.setPointB(link2.data.lat(), link2.data.lon());
      } else if (link1DataValid) {
        navigation.setPointB(link1.data.lat(), link1.data.lon());
      }
    }
  } else {
    if (link1.data.lat() != 0 && link1.data.lon() != 0) {
      navigation.setPointB(link1.data.lat(), link1.data.lon());
    }
  }

  delay(100);
}
