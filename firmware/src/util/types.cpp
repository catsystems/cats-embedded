/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "util/types.h"

/* as suggested by
 * https://stackoverflow.com/questions/23699719/inline-vs-static-inline-in-header-file
 */
extern inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low);
extern inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low);
