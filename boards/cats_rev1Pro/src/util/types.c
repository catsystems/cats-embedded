/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

const char *flight_fsm_map[14] = {"INVALID",   "MOVING",      "READY",       "THRUSTING_1", "THRUSTING_2",
                                  "COASTING",  "TRANSONIC_1", "SUPERSONIC", "TRANSONIC_2", "APOGEE",
                                  "PARACHUTE", "BALLISTIC",   "TOUCHDOWN",  "HEHE"};

const char *drop_test_fsm_map[7] = {"DT_INVALID", "DT_READY",      "DT_WAITING", "DT_DROGUE",
                                    "DT_MAIN",    "DT_TOUCHDOWN", "HEHE"};
