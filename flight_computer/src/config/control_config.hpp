/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#define USE_MEDIAN_FILTER
inline constexpr uint8_t MEDIAN_FILTER_SIZE = 9;

inline constexpr float P_INITIAL = 101250.0F;                   // hPa
inline constexpr float GRAVITY = 9.81F;                         // m/s^2
inline constexpr float TEMPERATURE_0 = 15.0F;                   // °C
inline constexpr float BARO_LIFTOFF_MOV_AVG_SIZE = 500.0F;      // samples -> 5 seconds
inline constexpr float BARO_LIFTOFF_FAST_MOV_AVG_SIZE = 10.0F;  // samples -> 5 seconds
