/******************************************************************************
 * file    main.cpp
 *******************************************************************************
 * brief   Main Program
 *******************************************************************************
 * author  Florian Baumgartner
 * version 1.0
 * date    2022-08-02
 *******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Crelin - Florian Baumgartner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

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

  if (!utils.begin(0, "DRIVE")) {
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
