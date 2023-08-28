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

#pragma once

#include "config/cats_config.hpp"

extern const char *const fsm_map[8];
extern const char *const event_map[9];
extern const char *const action_map[8];
extern const char *const on_off_map[2];
extern const char *const battery_map[3];

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern char *recorder_speed_map[NUM_REC_SPEEDS];

void init_recorder_speed_map();
