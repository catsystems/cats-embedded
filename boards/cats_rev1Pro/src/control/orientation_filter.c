/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "control/orientation_filter.h"
#include "control/quaternion.h"
#include "config/globals.h"

/* Orientation Filter */
void init_orientation_filter(orientation_filter_t* filter) {
  arm_mat_init_f32(&filter->gyro, 4, 1, filter->gyro_data);
  arm_mat_init_f32(&filter->estimate, 4, 1, filter->estimate_data);
  filter->t_sampl = (float32_t)CONTROL_SAMPLING_FREQ;
}

void reset_orientation_filter(orientation_filter_t* filter) {
  filter->estimate_data[0] = 1.0f;
  filter->estimate_data[1] = 0.0f;
  filter->estimate_data[2] = 0.0f;
  filter->estimate_data[3] = 0.0f;
}

void quaternion_kinematics(orientation_filter_t* filter, const vf32_t* angular_vel) {

  filter->gyro_data[0] = 0.0f;
  filter->gyro_data[1] = angular_vel->x;
  filter->gyro_data[2] = angular_vel->y;
  filter->gyro_data[3] = angular_vel->z;

  /* x_hat = x_bar + 1/2*Ts(quat_mult(velocity, x_bar)) */
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  float32_t holder2_data[4] = {0};
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 4, 1, holder2_data);
  quaternion_mat(&filter->gyro, &filter->estimate, &holder_mat);

  arm_mat_scale_f32(&holder_mat, (float32_t)(0.5f * filter->t_sampl), &holder2_mat);

  arm_mat_add_f32(&holder2_mat, &filter->estimate, &filter->estimate);

  /* Normalize Prediction */
  normalize_q(filter->estimate_data);
}