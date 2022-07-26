/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

extern const char *const boot_state_map[6];
extern const char *const fsm_map[14];
extern const char *const event_map[9];
extern const char *const action_map[17];

#ifdef CATS_VEGA
extern const char *const direction_map[2];
#endif

extern char *recorder_speed_map[NUM_REC_SPEEDS];

void init_recorder_speed_map();
