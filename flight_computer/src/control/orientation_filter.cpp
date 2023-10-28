/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/orientation_filter.hpp"
#include "config/globals.hpp"
#include "util/math_util.hpp"

/* Orientation Filter */
void init_orientation_filter(orientation_filter_t* filter) {
  arm_mat_init_f32(&filter->gyro, 4, 1, filter->gyro_data);
  arm_mat_init_f32(&filter->estimate, 4, 1, filter->estimate_data);
  filter->t_sampl = static_cast<float32_t>(1.0F / CONTROL_SAMPLING_FREQ);
}

void reset_orientation_filter(orientation_filter_t* filter) {
  filter->estimate_data[0] = 1.0F;
  filter->estimate_data[1] = 0.0F;
  filter->estimate_data[2] = 0.0F;
  filter->estimate_data[3] = 0.0F;
}

void quaternion_kinematics(orientation_filter_t* filter, vf32_t angular_vel) {
  filter->gyro_data[0] = 0.0F;
  filter->gyro_data[1] = angular_vel.x / 180.0F * PI;  // Convert to rad/s
  filter->gyro_data[2] = angular_vel.y / 180.0F * PI;  // Convert to rad/s
  filter->gyro_data[3] = angular_vel.z / 180.0F * PI;  // Convert to rad/s

  /* x_hat = x_bar + 1/2*Ts(quat_mult(velocity, x_bar)) */
  float32_t holder_data[4] = {};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  float32_t holder2_data[4] = {};
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 4, 1, holder2_data);
  quaternion_mat(&filter->estimate, &filter->gyro, &holder_mat);

  arm_mat_scale_f32(&holder_mat, static_cast<float32_t>(0.5F * filter->t_sampl), &holder2_mat);

  arm_mat_add_f32(&holder2_mat, &filter->estimate, &filter->estimate);

  /* Normalize Prediction */
  normalize_q(filter->estimate_data);
}