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

#define INIT_COV   0.1f
#define NOISE_VEL  0.1f
#define NOISE_POS  0.01f
#define NOISE_BIAS 0.01f

typedef struct {
  float32_t F_data[36];
  float32_t F_T_data[36];
  float32_t G_data[36];
  float32_t G_T_data[36];
  float32_t Q_data[36];
  float32_t H_data[18];
  float32_t H_T_data[18];
  float32_t R_data[9];
  float32_t K_data[18];
  float32_t x_bar_data[4];
  float32_t delta_x_bar_data[6];
  float32_t bias_data[3];
  float32_t velocity_data[4];
  float32_t P_bar_data[36];
  float32_t x_hat_data[4];
  float32_t z_data[3];
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
  arm_matrix_instance_f32 z;
  arm_matrix_instance_f32 delta_x_hat;
  arm_matrix_instance_f32 P_hat;
  arm_matrix_instance_f32 Identity;
  float t_sampl;
} orientation_filter_t;

void init_orientation_filter_struct(orientation_filter_t* filter);
void initialize_orientation_matrices(orientation_filter_t* filter);
void orientation_prediction_step(orientation_filter_t* filter,
                                 imu_data_t* data);
void orientation_update_step(orientation_filter_t* filter, imu_data_t* data);
