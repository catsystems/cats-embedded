//
// Created by jonas on 8/30/2021.
//

#pragma once

#include "arm_math.h"
#include "util/types.h"

void quaternion_skew(const float* input, float* output);
void quaternion_mat(const arm_matrix_instance_f32* input1, const arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output);
void extendR3(const float32_t* input, float32_t* output);
void normalize_q(float32_t* input);
void conjugate_q(const float32_t* input, float32_t* output);
