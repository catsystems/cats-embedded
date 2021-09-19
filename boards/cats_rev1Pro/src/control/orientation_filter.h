/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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
#include "util/types.h"

/* Abstracted Filter functions */
void init_orientation_filter(void* filter);
void reset_orientation_filter(void* filter);
void read_sensor_data(magneto_data_t* mag_data, imu_data_t* imu_data, void* filter);
void orientation_filter_step(void* filter);

/* Orientation Filter */
/* Source: https://www.mdpi.com/1424-8220/15/8/19302 */
#ifdef USE_ORIENTATION_FILTER
typedef struct {
  float32_t gyro_data[4];
  float32_t acceleration_data[4];
  float32_t magneto_data[4];
  float32_t propagation_estimate_data[4];
  float32_t propagation_estimate_conj_data[4];
  float32_t gravity_estimate_data[4];
  float32_t gravity_error_data[4];
  float32_t acceleration_error_data[4];
  float32_t acceleration_estimate_data[4];
  float32_t acceleration_estimate_conj_data[4];
  float32_t rotated_magneto_data[4];
  float32_t delta_magneto_data[4];
  float32_t magneto_correction_data[4];
  float32_t estimate_data[4];
  float32_t accel_gain;
  float32_t magneto_gain;
  float32_t interpolation_threshold;
  float32_t t_sampl;
  arm_matrix_instance_f32 gyro;
  arm_matrix_instance_f32 acceleration;
  arm_matrix_instance_f32 magneto;
  arm_matrix_instance_f32 propagation_estimate;
  arm_matrix_instance_f32 propagation_estimate_conj;
  arm_matrix_instance_f32 gravity_estimate;
  arm_matrix_instance_f32 gravity_error;
  arm_matrix_instance_f32 acceleration_error;
  arm_matrix_instance_f32 acceleration_estimate;
  arm_matrix_instance_f32 acceleration_estimate_conj;
  arm_matrix_instance_f32 rotated_magneto;
  arm_matrix_instance_f32 delta_magneto;
  arm_matrix_instance_f32 magneto_correction;
  arm_matrix_instance_f32 estimate;
} orientation_filter_t;
#endif
/* Orientation Kalman Filter */

#ifdef USE_ORIENTATION_KF

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
  float32_t accel_data[4];
  float32_t gyro_data[4];
  float32_t magneto_data[4];
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
  float32_t raw_computed_orientation[4];
} orientation_kf_t;
#endif
