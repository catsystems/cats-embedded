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

#include "arm_math.h"

void quaternion_skew(const float* input, float* output);
void quaternion_mat(const arm_matrix_instance_f32* input1, const arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output);
void extendR3(const float32_t* input, float32_t* output);
void normalize_q(float32_t* input);
void conjugate_q(const float32_t* input, float32_t* output);
