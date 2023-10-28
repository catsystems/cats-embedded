/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "arm_math.h"

#include <cstdint>

float32_t median(float32_t input_array[]);
float32_t calculate_height(float32_t pressure);
float32_t approx_moving_average(float32_t data, bool is_transparent);
