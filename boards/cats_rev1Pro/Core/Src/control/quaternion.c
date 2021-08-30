//
// Created by jonas on 8/30/2021.
//

#include "control/quaternion.h"

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
void quaternion_mat(arm_matrix_instance_f32* input1, arm_matrix_instance_f32* input2, arm_matrix_instance_f32* output) {
  float32_t LQM[16] = {input1->pData[0], -input1->pData[1], -input1->pData[2], -input1->pData[3],
                       input1->pData[1], input1->pData[0],  -input1->pData[3], input1->pData[2],
                       input1->pData[2], input1->pData[3],  input1->pData[0],  -input1->pData[1],
                       input1->pData[3], -input1->pData[2], input1->pData[1],  input1->pData[0]};
  arm_matrix_instance_f32 LQM_mat;
  arm_mat_init_f32(&LQM_mat, 4, 4, LQM);

  arm_mat_mult_f32(&LQM_mat, input2, output);
}

void extendR3(const float32_t* input, float32_t* output) {
  float32_t abs_value = 1.0f - input[0] * input[0] - input[1] * input[1] - input[2] * input[2];
  float32_t q0 = 0;
  arm_sqrt_f32(abs_value, &q0);
  output[0] = q0;
  output[1] = input[0];
  output[2] = input[1];
  output[3] = input[2];
}

void normalize_q(float32_t* input) {
  float32_t abs_value_sq = input[0] * input[0] + input[1] * input[1] + input[2] * input[2] + input[3] * input[3];
  float32_t abs_value = 0;
  arm_sqrt_f32(abs_value_sq, &abs_value);
  input[0] /= abs_value;
  input[1] /= abs_value;
  input[2] /= abs_value;
  input[3] /= abs_value;
}

void conjugate_q(float32_t* input, float32_t* output) {
  output[0] = input[0];
  output[1] = -input[1];
  output[2] = -input[2];
  output[3] = -input[3];
}
