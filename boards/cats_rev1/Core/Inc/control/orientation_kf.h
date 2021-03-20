/*
 * orientation_kf.h
 *
 *  Created on: Mar 20, 2021
 *      Author: jonas
 */

#ifndef INC_CONTROL_ORIENTATION_KF_H_
#define INC_CONTROL_ORIENTATION_KF_H_

#endif /* INC_CONTROL_ORIENTATION_KF_H_ */

#include "../DSP/Inc/arm_math.h"
#include "util/types.h"

void quaternion_skew(float* input, float* output);

void quaternion_mat(arm_matrix_instance_f32* input1,
                    arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output);

typedef struct {
  float32_t F_data[36];
  float32_t F_T_data[36];
  float32_t G_data[36];
  float32_t G_T_data[36];
  float32_t Q_data[36];
  float32_t H_data[9];
  float32_t H_T_data[9];
  float32_t R_data[9];
  float32_t K_data[9];
  float32_t x_bar_data[4];
  float32_t delta_x_bar_data[6];
  float32_t bias_data[4];
  float32_t velocity_data[4];
  float32_t P_bar_data[36];
  float32_t x_hat_data[4];
  float32_t delta_x_hat_data[6];
  float32_t P_hat_data[36];
  float32_t Identity_data[36];
  arm_matrix_instance_f32 F;
  arm_matrix_instance_f32 F_T;
  arm_matrix_instance_f32 G_T;
  arm_matrix_instance_f32 G;
  arm_matrix_instance_f32 Q;
  arm_matrix_instance_f32 H;
  arm_matrix_instance_f32 H_T;
  arm_matrix_instance_f32 R;
  arm_matrix_instance_f32 K;
  arm_matrix_instance_f32 x_bar;
  arm_matrix_instance_f32 delta_x_bar;
  arm_matrix_instance_f32 bias;
  arm_matrix_instance_f32 velocity;
  arm_matrix_instance_f32 P_bar;
  arm_matrix_instance_f32 x_hat;
  arm_matrix_instance_f32 delta_x_hat;
  arm_matrix_instance_f32 P_hat;
  arm_matrix_instance_f32 Identity;
  float t_sampl;
} orientation_filter_t;
