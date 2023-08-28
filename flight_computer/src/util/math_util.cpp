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

#include "math_util.hpp"

/* skews the quaternion */
void quaternion_skew(const float* input, float* output) {
  /* Matrix -> mat[9] = [0, 1, 2; 3, 4 , 5; 6, 7, 8];	 */
  output[0] = 0;
  output[1] = -input[3];
  output[2] = input[2];
  output[3] = input[3];
  output[4] = 0;
  output[5] = -input[1];
  output[6] = -input[2];
  output[7] = input[1];
  output[8] = 0;
}

/* ALL INPUTS AND OUTPUTS NEED TO BE 4,1 MATRICES */
void quaternion_mat(const arm_matrix_instance_f32* input1, const arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output) {
  float32_t LQM[16] = {input1->pData[0], -input1->pData[1], -input1->pData[2], -input1->pData[3],
                       input1->pData[1], input1->pData[0],  -input1->pData[3], input1->pData[2],
                       input1->pData[2], input1->pData[3],  input1->pData[0],  -input1->pData[1],
                       input1->pData[3], -input1->pData[2], input1->pData[1],  input1->pData[0]};
  arm_matrix_instance_f32 LQM_mat;
  arm_mat_init_f32(&LQM_mat, 4, 4, LQM);

  arm_mat_mult_f32(&LQM_mat, input2, output);
}

/* Extends a vector to R4 */
void extendR3(const float32_t* input, float32_t* output) {
  const float32_t abs_value = 1.0F - input[0] * input[0] - input[1] * input[1] - input[2] * input[2];
  float32_t q0 = 0;
  arm_sqrt_f32(abs_value, &q0);
  output[0] = q0;
  output[1] = input[0];
  output[2] = input[1];
  output[3] = input[2];
}

/* Normalized the quaternion such that abs(q) = 1 */
void normalize_q(float32_t* input) {
  const float32_t abs_value_sq = input[0] * input[0] + input[1] * input[1] + input[2] * input[2] + input[3] * input[3];
  float32_t abs_value = 0;
  arm_sqrt_f32(abs_value_sq, &abs_value);
  input[0] /= abs_value;
  input[1] /= abs_value;
  input[2] /= abs_value;
  input[3] /= abs_value;
}

/* conjugates the quaternion */
void conjugate_q(const float32_t* input, float32_t* output) {
  output[0] = input[0];
  output[1] = -input[1];
  output[2] = -input[2];
  output[3] = -input[3];
}
