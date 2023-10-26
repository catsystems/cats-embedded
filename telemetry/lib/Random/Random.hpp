/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

// the max value returned by rng
constexpr uint16_t kRngMax = 0x7FFF;

uint16_t rng();

void rngSeed(uint32_t newSeed);
// 0..255 returned
uint8_t rng8Bit();
// 0..31 returned
uint8_t rng5Bit();

// returns 0 <= x < max where n < 256
uint8_t rngN(uint8_t max);
