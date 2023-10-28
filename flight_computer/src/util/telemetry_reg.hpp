/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

enum transmission_direction_e {
  TX = 0,
  RX = 1,
};

enum transmission_mode_e {
  UNIDIRECTIONAL = 0,
  BIDIRECTIONAL = 1,
};

inline constexpr uint8_t CMD_DIRECTION{0x10};
inline constexpr uint8_t CMD_PA_GAIN{0x11};
inline constexpr uint8_t CMD_POWER_LEVEL{0x12};
inline constexpr uint8_t CMD_MODE{0x13};
inline constexpr uint8_t CMD_MODE_INDEX{0x14};

inline constexpr uint8_t CMD_LINK_PHRASE{0x15};

inline constexpr uint8_t CMD_ENABLE{0x20};
inline constexpr uint8_t CMD_DISABLE{0x21};

inline constexpr uint8_t CMD_TX{0x30};
inline constexpr uint8_t CMD_RX{0x31};
inline constexpr uint8_t CMD_INFO{0x32};

inline constexpr uint8_t CMD_GNSS_LOC{0x40};
inline constexpr uint8_t CMD_GNSS_TIME{0x41};
inline constexpr uint8_t CMD_GNSS_INFO{0x42};

inline constexpr uint8_t CMD_TEMP_INFO{0x50};

inline constexpr uint8_t CMD_VERSION_INFO{0x60};
