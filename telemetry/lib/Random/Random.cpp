/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
