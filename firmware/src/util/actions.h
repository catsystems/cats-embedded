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

#pragma once

#include <cstdint>

#define NUM_ACTION_FUNCTIONS 17

using peripheral_act_fp = bool (*)(int16_t);

extern const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS];

enum action_function_e {
  ACT_NO_OP = 0,
  ACT_OS_DELAY,
  ACT_HIGH_CURRENT_ONE,
  ACT_HIGH_CURRENT_TWO,
  ACT_HIGH_CURRENT_THREE,
  ACT_HIGH_CURRENT_FOUR,
  ACT_HIGH_CURRENT_FIVE,
  ACT_HIGH_CURRENT_SIX,
  ACT_LOW_LEVEL_ONE,
  ACT_LOW_LEVEL_TWO,
  ACT_LOW_LEVEL_THREE,
  ACT_LOW_LEVEL_FOUR,
  ACT_SERVO_ONE,
  ACT_SERVO_TWO,
  ACT_SERVO_THREE,
  ACT_SERVO_FOUR,
  ACT_SET_RECORDER_STATE,
};

/* TODO - don't export this anymore after the flash is working */
bool set_recorder_state(int16_t state);
