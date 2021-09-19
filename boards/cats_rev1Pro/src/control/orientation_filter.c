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

#include "control/orientation_filter.h"
#include "control/quaternion.h"
#include <math.h>

/* Orientation Filter */
#ifdef USE_ORIENTATION_FILTER
static void init_filter_mat(orientation_filter_t* filter) {
  arm_mat_init_f32(&filter->gyro, 4, 1, filter->gyro_data);
  arm_mat_init_f32(&filter->acceleration, 4, 1, filter->acceleration_data);
  arm_mat_init_f32(&filter->magneto, 4, 1, filter->magneto_data);
  arm_mat_init_f32(&filter->propagation_estimate, 4, 1, filter->propagation_estimate_data);
  arm_mat_init_f32(&filter->propagation_estimate_conj, 4, 1, filter->propagation_estimate_conj_data);
  arm_mat_init_f32(&filter->gravity_estimate, 4, 1, filter->gravity_estimate_data);
  arm_mat_init_f32(&filter->gravity_error, 4, 1, filter->gravity_error_data);
  arm_mat_init_f32(&filter->acceleration_error, 4, 1, filter->acceleration_error_data);
  arm_mat_init_f32(&filter->acceleration_estimate, 4, 1, filter->acceleration_estimate_data);
  arm_mat_init_f32(&filter->acceleration_estimate_conj, 4, 1, filter->acceleration_estimate_conj_data);
  arm_mat_init_f32(&filter->rotated_magneto, 4, 1, filter->rotated_magneto_data);
  arm_mat_init_f32(&filter->delta_magneto, 4, 1, filter->delta_magneto_data);
  arm_mat_init_f32(&filter->magneto_correction, 4, 1, filter->magneto_correction_data);
  arm_mat_init_f32(&filter->estimate, 4, 1, filter->estimate_data);
}

static void reset_filter(orientation_filter_t* filter) {
  filter->accel_gain = 0.2f;
  filter->magneto_gain = 0.2f;
  filter->interpolation_threshold = 0.9f;
  filter->estimate_data[0] = 1.0f;
  filter->estimate_data[1] = 0.0f;
  filter->estimate_data[2] = 0.0f;
  filter->estimate_data[3] = 0.0f;
}

static inline float32_t gain_factor(float32_t error) {
  if (error < 0.1f) {
    return 1.0f;
  } else if (error < 0.2f) {
    return (2.0f - 10.0f * error);
  } else {
    return 0.0f;
  }
}

static void inject_data(magneto_data_t* magneto, imu_data_t* imu, orientation_filter_t* filter) {
  float32_t abs_value = sqrtf(magneto->magneto_x * magneto->magneto_x + magneto->magneto_y * magneto->magneto_y +
                              magneto->magneto_z * magneto->magneto_z);
  filter->magneto_data[0] = 0.0f;
  filter->magneto_data[1] = magneto->magneto_x / abs_value;
  filter->magneto_data[2] = magneto->magneto_y / abs_value;
  filter->magneto_data[3] = magneto->magneto_z / abs_value;

  filter->gyro_data[0] = 0.0f;
  filter->gyro_data[1] = ((float32_t)(imu->gyro_x) / 16.4f) * (PI / 180.0f);
  filter->gyro_data[2] = ((float32_t)(imu->gyro_y) / 16.4f) * (PI / 180.0f);
  filter->gyro_data[3] = ((float32_t)(imu->gyro_z) / 16.4f) * (PI / 180.0f);

  filter->acceleration_data[0] = 0.0f;
  filter->acceleration_data[1] = ((float32_t)(imu->acc_x) / 1024.0f);
  filter->acceleration_data[2] = ((float32_t)(imu->acc_y) / 1024.0f);
  filter->acceleration_data[3] = ((float32_t)(imu->acc_z) / 1024.0f);
}

static void quaternion_kinematics(orientation_filter_t* filter) {
  /* x_hat = x_bar - 1/2*Ts(quat_mult(velocity, x_bar)) */
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  float32_t holder2_data[4] = {0};
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 4, 1, holder2_data);
  quaternion_mat(&filter->gyro, &filter->estimate, &holder_mat);

  arm_mat_scale_f32(&holder_mat, (float32_t)(-0.5f * filter->t_sampl), &holder2_mat);

  arm_mat_add_f32(&holder2_mat, &filter->estimate, &filter->propagation_estimate);

  /* Normalize Prediction */
  normalize_q(filter->propagation_estimate_data);

  conjugate_q(filter->propagation_estimate_data, filter->propagation_estimate_conj_data);
}

static void compute_gravity_error(orientation_filter_t* filter) {
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  quaternion_mat(&filter->propagation_estimate_conj, &filter->acceleration, &holder_mat);
  quaternion_mat(&holder_mat, &filter->propagation_estimate, &filter->gravity_estimate);

  filter->gravity_error_data[0] = sqrtf((filter->gravity_estimate_data[3] + 1) / 2);
  filter->gravity_error_data[1] = -filter->gravity_estimate_data[2] / sqrtf(2 * (filter->gravity_estimate_data[3] + 1));
  filter->gravity_error_data[2] = filter->gravity_estimate_data[1] / sqrtf(2 * (filter->gravity_estimate_data[3] + 1));
  filter->gravity_error_data[3] = 0;
}

static void interpolation(orientation_filter_t* filter, bool ismagneto) {
  /* Do the Linear or circular interpolation */
  if (ismagneto) {
    if (filter->delta_magneto_data[0] > filter->interpolation_threshold) {
      /* LERP */
      filter->magneto_correction_data[0] =
          (1.0f - filter->magneto_gain) + filter->magneto_gain * filter->delta_magneto_data[0];
      filter->magneto_correction_data[1] = filter->magneto_gain * filter->delta_magneto_data[1];
      filter->magneto_correction_data[2] = filter->magneto_gain * filter->delta_magneto_data[2];
      filter->magneto_correction_data[3] = filter->magneto_gain * filter->delta_magneto_data[3];

      normalize_q(filter->magneto_correction_data);
    } else {
      /* SLERP */
      float32_t omega = acosf(filter->delta_magneto_data[0]);
      float32_t factor1 = sinf((1.0f - filter->magneto_gain) * omega) / sinf(omega);
      float32_t factor2 = sinf(filter->magneto_gain * omega) / sinf(omega);
      filter->magneto_correction_data[0] = factor1 + factor2 * filter->delta_magneto_data[0];
      filter->magneto_correction_data[1] = factor2 * filter->delta_magneto_data[1];
      filter->magneto_correction_data[2] = factor2 * filter->delta_magneto_data[2];
      filter->magneto_correction_data[3] = factor2 * filter->delta_magneto_data[3];
    }
  } else {
#ifdef USE_ADAPTIVE_GAIN
    float32_t error = fabsf(sqrtf(filter->acceleration_data[1] * filter->acceleration_data[1] +
                                  filter->acceleration_data[2] * filter->acceleration_data[2] +
                                  filter->acceleration_data[3] * filter->acceleration_data[3]) -
                            1.0f);
    float32_t acc_gain = filter->accel_gain * gain_factor(error);
#elif
    float32_t acc_gain = filter->accel_gain;
#endif
    if (filter->gravity_error_data[0] > filter->interpolation_threshold) {
      /* LERP */
      filter->acceleration_error_data[0] = (1.0f - acc_gain) + acc_gain * filter->gravity_error_data[0];
      filter->acceleration_error_data[1] = acc_gain * filter->gravity_error_data[1];
      filter->acceleration_error_data[2] = acc_gain * filter->gravity_error_data[2];
      filter->acceleration_error_data[3] = acc_gain * filter->gravity_error_data[3];

      normalize_q(filter->acceleration_error_data);
    } else {
      /* SLERP */
      float32_t omega = acosf(filter->gravity_error_data[0]);
      float32_t factor1 = sinf((1.0f - acc_gain) * omega) / sinf(omega);
      float32_t factor2 = sinf(acc_gain * omega) / sinf(omega);
      filter->acceleration_error_data[0] = factor1 + factor2 * filter->gravity_error_data[0];
      filter->acceleration_error_data[1] = factor2 * filter->gravity_error_data[1];
      filter->acceleration_error_data[2] = factor2 * filter->gravity_error_data[2];
      filter->acceleration_error_data[3] = factor2 * filter->gravity_error_data[3];
    }
  }
}

static void accel_correction(orientation_filter_t* filter) {
  compute_gravity_error(filter);

  interpolation(filter, false);

  /* Include Accel Information into the estimate */
  quaternion_mat(&filter->propagation_estimate, &filter->acceleration_error, &filter->acceleration_estimate);
  conjugate_q(filter->acceleration_estimate_data, filter->acceleration_estimate_conj_data);
}

static void magneto_correction(orientation_filter_t* filter) {
  /* First rotate magneto data in the right coordinate frame */
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  quaternion_mat(&filter->acceleration_estimate_conj, &filter->magneto, &holder_mat);
  quaternion_mat(&holder_mat, &filter->acceleration_estimate, &filter->rotated_magneto);

  /* Compute Delta Quaternion */
  float32_t tau = filter->rotated_magneto_data[1] * filter->rotated_magneto_data[1] +
                  filter->rotated_magneto_data[2] * filter->rotated_magneto_data[2];
  float32_t sqrttau = sqrtf(tau);
  filter->delta_magneto_data[0] = sqrtf(tau + filter->rotated_magneto_data[1] * sqrttau) / (sqrtf(2) * sqrttau);
  filter->delta_magneto_data[1] = 0;
  filter->delta_magneto_data[2] = 0;
  filter->delta_magneto_data[3] =
      filter->rotated_magneto_data[2] / (sqrtf(2 * (tau + filter->rotated_magneto_data[1] * sqrttau)));

  interpolation(filter, true);

  /* Include magneto Information into the estimate */
  quaternion_mat(&filter->acceleration_estimate, &filter->magneto_correction, &filter->estimate);
}

static void orientation_filter(orientation_filter_t* filter) {
  /* First Propagate the Kinematics */
  quaternion_kinematics(filter);
  /* Correct Estimate with accel */
  accel_correction(filter);
  /* Correct Estimate with magneto */
  magneto_correction(filter);
}
#endif
/* Implement Orientation Kalman Filter */

#ifdef USE_ORIENTATION_KF
static void inject_kf_sensor_data(magneto_data_t* magneto, imu_data_t* imu, orientation_kf_t* filter) {
  float32_t abs_value = sqrtf(magneto->magneto_x * magneto->magneto_x + magneto->magneto_y * magneto->magneto_y +
                              magneto->magneto_z * magneto->magneto_z);
  filter->magneto_data[0] = 0.0f;
  filter->magneto_data[1] = magneto->magneto_x / abs_value;
  filter->magneto_data[2] = magneto->magneto_y / abs_value;
  filter->magneto_data[3] = magneto->magneto_z / abs_value;

  filter->gyro_data[0] = 0.0f;
  filter->gyro_data[1] = ((float32_t)(imu->gyro_x) / 16.4f) * (PI / 180.0f);
  filter->gyro_data[2] = ((float32_t)(imu->gyro_y) / 16.4f) * (PI / 180.0f);
  filter->gyro_data[3] = ((float32_t)(imu->gyro_z) / 16.4f) * (PI / 180.0f);

  filter->accel_data[0] = 0.0f;
  filter->accel_data[1] = ((float32_t)(imu->acc_x) / 1024.0f);
  filter->accel_data[2] = ((float32_t)(imu->acc_y) / 1024.0f);
  filter->accel_data[3] = ((float32_t)(imu->acc_z) / 1024.0f);
}

static void init_orientation_kf_struct(orientation_kf_t* const filter) {
  arm_mat_init_f32(&filter->F, 6, 6, filter->F_data);
  arm_mat_init_f32(&filter->F_T, 6, 6, filter->F_T_data);
  arm_mat_init_f32(&filter->G, 6, 6, filter->G_data);
  arm_mat_init_f32(&filter->G_T, 6, 6, filter->G_T_data);
  arm_mat_init_f32(&filter->Q, 6, 6, filter->Q_data);
  arm_mat_init_f32(&filter->H, 3, 6, filter->H_data);
  arm_mat_init_f32(&filter->H_T, 6, 3, filter->H_T_data);
  arm_mat_init_f32(&filter->R, 3, 3, filter->R_data);
  arm_mat_init_f32(&filter->K, 6, 3, filter->K_data);
  arm_mat_init_f32(&filter->x_bar, 4, 1, filter->x_bar_data);
  arm_mat_init_f32(&filter->delta_x_bar, 6, 1, filter->delta_x_bar_data);
  arm_mat_init_f32(&filter->bias, 3, 1, filter->bias_data);
  arm_mat_init_f32(&filter->velocity, 4, 1, filter->velocity_data);
  arm_mat_init_f32(&filter->P_bar, 6, 6, filter->P_bar_data);
  arm_mat_init_f32(&filter->x_hat, 4, 1, filter->x_hat_data);
  arm_mat_init_f32(&filter->z, 3, 1, filter->z_data);
  arm_mat_init_f32(&filter->delta_x_hat, 6, 1, filter->delta_x_hat_data);
  arm_mat_init_f32(&filter->P_hat, 6, 6, filter->P_hat_data);
  arm_mat_init_f32(&filter->Identity, 6, 6, filter->Identity_data);
}

static void initialize_orientation_matrices(orientation_kf_t* const filter) {
  /* Matrix -> mat[9] = [0, 1, 2; 3, 4 , 5; 6, 7, 8];*/

  /* Initialize static values */
  float32_t F[36] = {0};
  float32_t F_T[36] = {0};
  float32_t G[36] = {0};
  float32_t G_T[36] = {0};
  float32_t Q[36] = {0};
  float32_t H[18] = {0};
  float32_t H_T[18] = {0};
  float32_t R[9] = {0};
  float32_t K[18] = {0};
  float32_t x_bar[4] = {0};
  float32_t delta_x_bar[6] = {0};
  float32_t bias[3] = {0};
  float32_t velocity[4] = {0};
  float32_t P_bar[36] = {0};
  float32_t x_hat[4] = {0};
  float32_t z[3] = {0};
  float32_t delta_x_hat[6] = {0};
  float32_t P_hat[36] = {0};
  float32_t Identity[36] = {0};

  Q[0] = NOISE_VEL;
  Q[7] = NOISE_VEL;
  Q[14] = NOISE_VEL;
  Q[21] = NOISE_BIAS;
  Q[28] = NOISE_BIAS;
  Q[35] = NOISE_BIAS;

  R[0] = NOISE_POS;
  R[4] = NOISE_POS;
  R[8] = NOISE_POS;

  P_bar[0] = INIT_COV;
  P_bar[7] = INIT_COV;
  P_bar[14] = INIT_COV;
  P_bar[21] = INIT_COV;
  P_bar[28] = INIT_COV;
  P_bar[35] = INIT_COV;

  P_hat[0] = INIT_COV;
  P_hat[7] = INIT_COV;
  P_hat[14] = INIT_COV;
  P_hat[21] = INIT_COV;
  P_hat[28] = INIT_COV;
  P_hat[35] = INIT_COV;

  x_bar[0] = 1.0f;
  x_hat[0] = 1.0f;

  Identity[0] = 1.0f;
  Identity[7] = 1.0f;
  Identity[14] = 1.0f;
  Identity[21] = 1.0f;
  Identity[28] = 1.0f;
  Identity[35] = 1.0f;

  H[0] = 1.0f;
  H[7] = 1.0f;
  H[14] = 1.0f;

  H_T[0] = 1.0f;
  H_T[4] = 1.0f;
  H_T[8] = 1.0f;

  memcpy(filter->F_data, F, sizeof(F));
  memcpy(filter->F_T_data, F_T, sizeof(F_T));
  memcpy(filter->G_data, G, sizeof(G));
  memcpy(filter->G_T_data, G_T, sizeof(G_T));
  memcpy(filter->H_data, H, sizeof(H));
  memcpy(filter->H_T_data, H_T, sizeof(H_T));
  memcpy(filter->R_data, R, sizeof(R));
  memcpy(filter->Q_data, Q, sizeof(Q));
  memcpy(filter->K_data, K, sizeof(K));
  memcpy(filter->x_bar_data, x_bar, sizeof(x_bar));
  memcpy(filter->delta_x_bar_data, delta_x_bar, sizeof(delta_x_bar));
  memcpy(filter->bias_data, bias, sizeof(bias));
  memcpy(filter->velocity_data, velocity, sizeof(velocity));
  memcpy(filter->P_bar_data, P_bar, sizeof(P_bar));
  memcpy(filter->x_hat_data, x_hat, sizeof(x_hat));
  memcpy(filter->z_data, z, sizeof(z));
  memcpy(filter->delta_x_hat_data, delta_x_hat, sizeof(delta_x_hat));
  memcpy(filter->P_hat_data, P_hat, sizeof(P_hat));
  memcpy(filter->Identity_data, Identity, sizeof(Identity));
}

static void compute_F(orientation_kf_t* filter) {
  /* Get the skew of the velocity */
  float32_t vel_skew[9] = {0};
  quaternion_skew(filter->velocity_data, vel_skew);

  /* Compute F
   * F = [-q_skew(omega_hat), -0.5*eye(3);...
zeros(3), zeros(3)];
   */
  filter->F_data[0] = vel_skew[0];
  filter->F_data[1] = vel_skew[1];
  filter->F_data[2] = vel_skew[2];
  filter->F_data[6] = vel_skew[3];
  filter->F_data[7] = vel_skew[4];
  filter->F_data[8] = vel_skew[5];
  filter->F_data[12] = vel_skew[6];
  filter->F_data[13] = vel_skew[7];
  filter->F_data[14] = vel_skew[8];
  filter->F_data[0] = vel_skew[0];
  filter->F_data[3] = -0.5f;
  filter->F_data[10] = -0.5f;
  filter->F_data[17] = -0.5f;

  arm_mat_trans_f32(&filter->F, &filter->F_T);
}

static void compute_G(orientation_kf_t* filter) {
  /*
   * G = [-0.5*eye(3), zeros(3);...
zeros(3), eye(3)];
   */
  filter->G_data[0] = -0.5f;
  filter->G_data[7] = -0.5f;
  filter->G_data[14] = -0.5f;
  filter->G_data[21] = 1.0f;
  filter->G_data[28] = 1.0f;
  filter->G_data[35] = 1.0f;
  arm_mat_trans_f32(&filter->G, &filter->G_T);
}

static void compute_angle(orientation_kf_t* filter) {
  float32_t amplitude =
      sqrtf(filter->accel_data[1] * filter->accel_data[1] + filter->accel_data[2] * filter->accel_data[2] +
            filter->accel_data[3] * filter->accel_data[3]);
  filter->accel_data[1] /= amplitude;
  filter->accel_data[2] /= amplitude;
  filter->accel_data[3] /= amplitude;
  /* Compute Quaternion from Accelerometer Data */
  float32_t quat_meas_acc[4] = {0};
  if (filter->accel_data[3] < 0) {
    quat_meas_acc[0] = -filter->accel_data[2] / sqrtf(2.0f * (1.0f - filter->accel_data[3]));
    quat_meas_acc[1] = sqrtf((1.0f - filter->accel_data[3]) / 2.0f);
    quat_meas_acc[3] = filter->accel_data[1] / sqrtf(2.0f * (1.0f - filter->accel_data[3]));
  } else {
    quat_meas_acc[0] = sqrtf((filter->accel_data[2] + 1.0f) / 2.0f);
    quat_meas_acc[1] = -filter->accel_data[2] / sqrtf(2.0f * (filter->accel_data[3] + 1.0f));
    quat_meas_acc[2] = filter->accel_data[1] / sqrtf(2.0f * (filter->accel_data[3] + 1.0f));
  }

  /* Create Quaternion Structs for IMU */
  arm_matrix_instance_f32 quat_imu_mat;
  arm_mat_init_f32(&quat_imu_mat, 4, 1, quat_meas_acc);
  /* Conjugate to turn the magnetometer in the right direction */
  arm_matrix_instance_f32 quat_imu_mat_conj;
  float32_t quat_meas_acc_conj[4] = {quat_meas_acc[0], -quat_meas_acc[1], -quat_meas_acc[2], -quat_meas_acc[3]};
  arm_mat_init_f32(&quat_imu_mat_conj, 4, 1, quat_meas_acc_conj);

  /* Compute Quaternion from Magnetometer Data */

  /* Normalize and rotate quaternion */
  float32_t quat_meas_magneto[4] = {0};
  float32_t magneto_scaled[4] = {0, filter->magneto_data[1], filter->magneto_data[2], filter->magneto_data[3]};
  arm_matrix_instance_f32 scaled_magneto_mat;
  arm_mat_init_f32(&scaled_magneto_mat, 4, 1, magneto_scaled);

  float32_t holder[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder);
  float32_t rotated_magneto[4] = {0};
  arm_matrix_instance_f32 rotated_magneto_mat;
  arm_mat_init_f32(&rotated_magneto_mat, 4, 1, rotated_magneto);
  /* l = q_star*m*q */
  quaternion_mat(&quat_imu_mat_conj, &scaled_magneto_mat, &holder_mat);
  quaternion_mat(&holder_mat, &quat_imu_mat, &rotated_magneto_mat);
  /*
    log_trace("%ld; %ld; %ld; %ld", (int32_t)((float)rotated_magneto[0] * 1000),
              (int32_t)((float)rotated_magneto[1] * 1000), (int32_t)((float)rotated_magneto[2] * 1000),
              (int32_t)((float)rotated_magneto[3] * 1000));
  */
  float32_t tau = rotated_magneto[1] * rotated_magneto[1] + rotated_magneto[2] * rotated_magneto[2];
  float32_t sqrt_tau = sqrtf(tau);
  if (rotated_magneto[1] < 0) {
    quat_meas_magneto[0] = rotated_magneto[2] / (sqrtf(2.0f) * sqrtf(tau - rotated_magneto[1] * sqrt_tau));
    quat_meas_magneto[3] = sqrtf(tau - rotated_magneto[1] * sqrt_tau) / (sqrtf(2.0f) * sqrt_tau);
  } else {
    quat_meas_magneto[0] = sqrtf(tau + rotated_magneto[1] * sqrt_tau) / (sqrtf(2.0f) * sqrt_tau);
    quat_meas_magneto[3] = rotated_magneto[2] / (sqrtf(2.0f) * sqrtf(tau + rotated_magneto[1] * sqrt_tau));
  }

  arm_matrix_instance_f32 quat_magneto_mat;
  arm_mat_init_f32(&quat_magneto_mat, 4, 1, quat_meas_magneto);

  /* Compute Quaternion */
  float32_t quat_meas[4] = {0};
  arm_matrix_instance_f32 quat_meas_mat;
  arm_mat_init_f32(&quat_meas_mat, 4, 1, quat_meas);
  quaternion_mat(&quat_imu_mat, &quat_magneto_mat, &quat_meas_mat);

  /* For Logging */
  memcpy(filter->raw_computed_orientation, quat_meas, sizeof(quat_meas));

  /* Take out the current guess and conjugate it*/
  float32_t x_hat[4] = {filter->x_hat_data[0], -filter->x_hat_data[1], -filter->x_hat_data[2], -filter->x_hat_data[3]};
  arm_matrix_instance_f32 x_hat_mat;
  arm_mat_init_f32(&x_hat_mat, 4, 1, x_hat);

  /* Compute the quaternion error */
  float32_t z_1[4] = {0};
  arm_matrix_instance_f32 z_1_mat;
  arm_mat_init_f32(&z_1_mat, 4, 1, z_1);
  quaternion_mat(&x_hat_mat, &quat_meas_mat, &z_1_mat);

  /* reduce error quaternion and insert into measurement variable */
  filter->z_data[0] = z_1[1];
  filter->z_data[1] = z_1[2];
  filter->z_data[2] = z_1[3];
}

static void orientation_prediction_step(orientation_kf_t* filter) {
  /* remove bias from measurement */
  /* SCALE GYRO CORRECTLY */
  filter->velocity_data[1] = filter->gyro_data[1] - filter->bias_data[0];
  filter->velocity_data[2] = filter->gyro_data[2] - filter->bias_data[1];
  filter->velocity_data[3] = filter->gyro_data[3] - filter->bias_data[2];

  /* Prediction step */
  /* x_hat = x_bar + 1/2*Ts(quat_mult(x_bar, velocity)) */
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);

  float32_t holder2_data[4] = {0};
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 4, 1, holder2_data);
  quaternion_mat(&filter->x_bar, &filter->velocity, &holder_mat);

  arm_mat_scale_f32(&holder_mat, (float32_t)(0.5f * filter->t_sampl), &holder2_mat);

  arm_mat_add_f32(&holder2_mat, &filter->x_bar, &filter->x_hat);

  /* Normalize Prediction */
  normalize_q(filter->x_hat_data);

  /* Compute F and G matrix */
  compute_F(filter);
  compute_G(filter);

  /* Propagate Covariance */
  /* P_hat = P_bar + Ts*(F*P+P*F' + G*Q*G')*/
  float32_t holder3_data[16] = {0};
  arm_matrix_instance_f32 holder3_mat;
  arm_mat_init_f32(&holder3_mat, 4, 4, holder3_data);

  float32_t holder4_data[16] = {0};
  arm_matrix_instance_f32 holder4_mat;
  arm_mat_init_f32(&holder4_mat, 4, 4, holder4_data);

  float32_t holder5_data[16] = {0};
  arm_matrix_instance_f32 holder5_mat;
  arm_mat_init_f32(&holder5_mat, 4, 4, holder5_data);

  float32_t holder6_data[16] = {0};
  arm_matrix_instance_f32 holder6_mat;
  arm_mat_init_f32(&holder6_mat, 4, 4, holder6_data);

  /* G*Q*G' */
  arm_mat_mult_f32(&filter->G, &filter->Q, &holder3_mat);
  arm_mat_mult_f32(&holder3_mat, &filter->G_T, &holder4_mat);

  /*F*P+P*F'*/
  arm_mat_mult_f32(&filter->F, &filter->P_bar, &holder3_mat);
  arm_mat_mult_f32(&filter->P_bar, &filter->F_T, &holder5_mat);
  arm_mat_add_f32(&holder3_mat, &holder5_mat, &holder6_mat);

  /*Ts*(F*P+P*F' + G*Q*G')*/
  arm_mat_add_f32(&holder3_mat, &holder6_mat, &holder4_mat);
  arm_mat_scale_f32(&holder4_mat, (float32_t)(0.5f), &holder3_mat);

  arm_mat_add_f32(&holder3_mat, &filter->P_bar, &filter->P_hat);
}

static void orientation_update_step(orientation_kf_t* filter) {
  /* Compute Kalman Gain */
  /* K = P_hat*H'/(H*P_hat*H' + R) */
  float32_t holder_6x3_data[18] = {0};
  arm_matrix_instance_f32 holder_6x3_mat;
  arm_mat_init_f32(&holder_6x3_mat, 6, 3, holder_6x3_data);

  float32_t holder_3x6_data[18] = {0};
  arm_matrix_instance_f32 holder_3x6_mat;
  arm_mat_init_f32(&holder_3x6_mat, 3, 6, holder_3x6_data);

  float32_t holder_3x3_data[16] = {0};
  arm_matrix_instance_f32 holder_3x3_mat;
  arm_mat_init_f32(&holder_3x3_mat, 3, 3, holder_3x3_data);

  float32_t holder_3x3_data_1[16] = {0};
  arm_matrix_instance_f32 holder_3x3_mat_1;
  arm_mat_init_f32(&holder_3x3_mat_1, 3, 3, holder_3x3_data_1);

  /* H*P_hat*H' */
  arm_mat_mult_f32(&filter->H, &filter->P_hat, &holder_3x6_mat);
  arm_mat_mult_f32(&holder_3x6_mat, &filter->H_T, &holder_3x3_mat);

  /* 1/(H*P_hat*H' + R) */
  arm_mat_add_f32(&holder_3x3_mat, &filter->R, &holder_3x3_mat_1);
  arm_mat_inverse_f32(&holder_3x3_mat_1, &holder_3x3_mat);

  /* P_hat*H' */
  arm_mat_mult_f32(&filter->P_hat, &filter->H_T, &holder_6x3_mat);

  /* K = P_hat*H'/(H*P_hat*H' + R) */
  arm_mat_mult_f32(&holder_6x3_mat, &holder_3x3_mat, &filter->K);

  /* Compute Error */
  /* error = K*reduce(quat_error) = K*reduce(x_hat^* * meas) = K*z */
  compute_angle(filter);
  arm_mat_mult_f32(&filter->K, &filter->z, &filter->delta_x_hat);

  /* Extract and Normalize attitude Error */
  float32_t extended_delta_x_hat_data[4] = {0};
  arm_matrix_instance_f32 extended_delta_x_hat_mat;
  arm_mat_init_f32(&extended_delta_x_hat_mat, 4, 1, extended_delta_x_hat_data);

  extendR3(filter->delta_x_hat_data, extended_delta_x_hat_data);

  /* Extract Bias Error */
  float32_t bias_error_data[3] = {filter->delta_x_hat_data[3], filter->delta_x_hat_data[4],
                                  filter->delta_x_hat_data[5]};
  arm_matrix_instance_f32 bias_error_mat;
  arm_mat_init_f32(&bias_error_mat, 3, 1, bias_error_data);

  /* Compute measurement update */
  quaternion_mat(&filter->x_hat, &extended_delta_x_hat_mat, &filter->x_bar);

  /* Update Bias */
  float32_t holder_data_vec[3] = {0};
  arm_matrix_instance_f32 holder_data_mat;
  arm_mat_init_f32(&holder_data_mat, 3, 1, holder_data_vec);
  arm_mat_add_f32(&bias_error_mat, &filter->bias, &holder_data_mat);

  filter->bias_data[0] = holder_data_vec[0];
  filter->bias_data[1] = holder_data_vec[1];
  filter->bias_data[2] = holder_data_vec[2];

  /* Update Covariance */
  /* P_bar = (1-KH)P_hat */
  float32_t holder_6x6_data[36] = {0};
  arm_matrix_instance_f32 holder_6x6_mat;
  arm_mat_init_f32(&holder_6x6_mat, 6, 6, holder_6x6_data);
  float32_t holder_6x6_data_1[36] = {0};
  arm_matrix_instance_f32 holder_6x6_mat_1;
  arm_mat_init_f32(&holder_6x6_mat_1, 6, 6, holder_6x6_data_1);

  arm_mat_mult_f32(&filter->K, &filter->H, &holder_6x6_mat);
  arm_mat_add_f32(&holder_6x6_mat, &filter->Identity, &holder_6x6_mat_1);
  arm_mat_mult_f32(&holder_6x6_mat_1, &filter->P_hat, &filter->P_bar);
}

static void orientation_kf(orientation_kf_t* filter) {
  /* Prediction Step */
  orientation_prediction_step(filter);
  /* Update Step */
  orientation_update_step(filter);
}
#endif

/* Abstraction Functions */
void init_orientation_filter(void* filter) {
#ifdef USE_ORIENTATION_FILTER
  init_filter_mat((orientation_filter_t*)filter);
#endif
#ifdef USE_ORIENTATION_KF
  init_orientation_kf_struct((orientation_kf_t*)filter);
#endif
}

void reset_orientation_filter(void* filter) {
#ifdef USE_ORIENTATION_FILTER
  reset_filter((orientation_filter_t*)filter);
#endif
#ifdef USE_ORIENTATION_KF
  initialize_orientation_matrices((orientation_kf_t*)filter);
#endif
}

void orientation_filter_step(void* filter) {
#ifdef USE_ORIENTATION_FILTER
  orientation_filter((orientation_filter_t*)filter);
#endif
#ifdef USE_ORIENTATION_KF
  orientation_kf((orientation_kf_t*)filter);
#endif
}

void read_sensor_data(magneto_data_t* mag_data, imu_data_t* imu_data, void* filter) {
#ifdef USE_ORIENTATION_FILTER
  inject_data(mag_data, imu_data, (orientation_filter_t*)filter);
#endif
#ifdef USE_ORIENTATION_KF
  inject_kf_sensor_data(mag_data, imu_data, (orientation_kf_t*)filter);
#endif
}