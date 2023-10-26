/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "Random.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static uint32_t seed = 0;

// returns values between 0 and 0x7FFF
// NB rngN depends on this output range, so if we change the
// behaviour rngN will need updating
uint16_t rng() {
  const uint32_t m = 2147483648;
  const uint32_t a = 214013;
  const uint32_t c = 2531011;
  seed = (a * seed + c) % m;
  return seed >> 16U;
}

void rngSeed(const uint32_t newSeed) { seed = newSeed; }

// returns 0 <= x < max where max < 256
uint8_t rngN(const uint8_t max) { return rng() % max; }

// 0..255 returned
uint8_t rng8Bit() { return rng() & 0xFFU; }

// 0..31 returned
uint8_t rng5Bit() { return rng() & 0x1FU; }
