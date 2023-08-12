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
