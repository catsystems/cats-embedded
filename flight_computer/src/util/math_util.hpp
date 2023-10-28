/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "arm_math.h"

void quaternion_skew(const float* input, float* output);
void quaternion_mat(const arm_matrix_instance_f32* input1, const arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output);
void extendR3(const float32_t* input, float32_t* output);
void normalize_q(float32_t* input);
void conjugate_q(const float32_t* input, float32_t* output);
