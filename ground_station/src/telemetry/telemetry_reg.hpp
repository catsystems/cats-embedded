/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

enum transmission_direction_e {
  TX_DIR = 0,
  RX_DIR = 1,
};

enum transmission_mode_e {
  UNIDIRECTIONAL = 0,
  BIDIRECTIONAL = 1,
};

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define CMD_DIRECTION   0x10
#define CMD_PA_GAIN     0x11
#define CMD_POWER_LEVEL 0x12
#define CMD_MODE        0x13
#define CMD_MODE_INDEX  0x14

#define CMD_LINK_PHRASE 0x15

#define CMD_ENABLE  0x20
#define CMD_DISBALE 0x21

#define CMD_TX   0x30
#define CMD_RX   0x31
#define CMD_INFO 0x32

#define CMD_GNSS_LOC  0x40
#define CMD_GNSS_TIME 0x41
#define CMD_GNSS_INFO 0x42
// NOLINTEND(cppcoreguidelines-macro-usage)
