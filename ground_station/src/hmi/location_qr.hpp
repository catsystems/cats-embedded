/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Adafruit_GFX.h>

#include <cstddef>
#include <cstdint>

namespace LocationQr {

inline constexpr size_t kGoogleMapsUrlSize = 80;

bool IsValid(float latitude, float longitude);

bool BuildGoogleMapsUrl(float latitude, float longitude, char *url, size_t urlSize);

bool DrawGoogleMapsQr(Adafruit_GFX &display, float latitude, float longitude, int16_t x, int16_t y, uint16_t black,
                      uint16_t white);

}  // namespace LocationQr
