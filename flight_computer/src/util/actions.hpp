/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

inline constexpr uint8_t NUM_ACTION_FUNCTIONS = 8;

using peripheral_act_fp = bool (*)(int16_t);

extern const peripheral_act_fp action_table[NUM_ACTION_FUNCTIONS];

enum action_function_e {
  ACT_NO_OP = 0,
  ACT_OS_DELAY,
  ACT_HIGH_CURRENT_ONE,
  ACT_HIGH_CURRENT_TWO,
  ACT_LOW_LEVEL_ONE,
  ACT_SERVO_ONE,
  ACT_SERVO_TWO,
  ACT_SET_RECORDER_STATE,
};

/* TODO - don't export this anymore after the flash is working */
bool set_recorder_state(int16_t state);
