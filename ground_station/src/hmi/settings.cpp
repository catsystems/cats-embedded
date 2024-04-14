/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "hmi/settings.hpp"
#include "config.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
const device_settings_t settingsTable[][4] = {
    {
        {"Stop Logging",
         "Down: Stop the log at touchdown",
         "Never: Never stop logging after liftoff",
         TOGGLE,
         {.lookup = TABLE_LOGGING},
         &systemConfig.config.neverStopLogging},

        {
            "Version",
            "Firmware Version: " FIRMWARE_VERSION,
            "",
            BUTTON,
            {.fun_ptr = nullptr},
            nullptr,
        },
        {
            "Start Bootloader",
            "Press A to start the bootloader",
            "Make sure you are connected to a computer",
            BUTTON,
            {.fun_ptr = Utils::startBootloader},
            nullptr,
        },
    },
    {
        {"Mode",
         "Single: Use both receiver to track one rocket",
         "Dual: Use both receivers individually",
         TOGGLE,
         {.lookup = TABLE_MODE},
         &systemConfig.config.receiverMode},
        {"Link Phrase 1",
         "Single Mode: Set phrase for both receivers",
         "Dual Mode: Set phrase for the left receiver",
         STRING,
         {.stringLength = kMaxPhraseLen},
         systemConfig.config.linkPhrase1},
        {"Link Phrase 2",
         "Single Mode: No functionality",
         "Dual Mode: Set phrase for the right receiver",
         STRING,
         {.stringLength = kMaxPhraseLen},
         systemConfig.config.linkPhrase2},
        {"Testing Phrase",
         "Set the phrase for the testing mode",
         "",
         STRING,
         {.stringLength = kMaxPhraseLen},
         systemConfig.config.testingPhrase},
    },
    {
        {"Time Zone",
         "Time offset relative to UTC",
         "",
         NUMBER,
         {.minmax = {.min = -12, .max = 12}},
         &systemConfig.config.timeZoneOffset},
        {"Units", "Metric: meters", "Imperial: feet", TOGGLE, {.lookup = TABLE_UNIT}, &systemConfig.config.unitSystem},
    }};
