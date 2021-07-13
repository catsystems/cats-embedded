/*
 * orientation_kf.c
 *
 *  Created on: Mar 20, 2021
 *      Author: jonas
 */

#include "control/orientation_kf.h"

void init_orientation_filter_struct(orientation_filter_t* const filter) {
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

void initialize_orientation_matrices(orientation_filter_t* const filter) {
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

/* ALL INPUTS AND OUTPUTS NEED TO BE 4,1 MATRIXES */
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
  float32_t abs_value = 1.0f - input[0] * input[0] + input[1] * input[1] + input[2] * input[2];
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

void compute_angle(imu_data_t* data, orientation_filter_t* filter) {
  /* Scale acceleration and rotate into inertial frame */
  float32_t quat_meas[4] = {(float32_t)(data->acc_z) / 1024.0f, -(float32_t)(data->acc_y) / 1024.0f,
                            (float32_t)(data->acc_x) / 1024.0f, 0};
  float32_t abs_sq = quat_meas[0] * quat_meas[0] + quat_meas[1] * quat_meas[1] + quat_meas[2] * quat_meas[2];
  float32_t abs = 0;
  arm_sqrt_f32(abs_sq, &abs);

  quat_meas[0] /= abs;
  quat_meas[1] /= abs;
  quat_meas[2] /= abs;

  arm_matrix_instance_f32 quat_meas_mat;
  arm_mat_init_f32(&quat_meas_mat, 4, 1, quat_meas);

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

void compute_F(orientation_filter_t* filter) {
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

void compute_G(orientation_filter_t* filter) {
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

void orientation_prediction_step(orientation_filter_t* filter, imu_data_t* data) {
  /* remove bias from measurement */
  /* SCALE GYRO CORRECTLY */
  filter->velocity_data[1] = (float32_t)(data->gyro_x) / 16.4f - filter->bias_data[0];
  filter->velocity_data[2] = (float32_t)(data->gyro_y) / 16.4f - filter->bias_data[1];
  filter->velocity_data[3] = (float32_t)(data->gyro_z) / 16.4f - filter->bias_data[2];

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

void orientation_update_step(orientation_filter_t* filter, imu_data_t* data) {
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
  compute_angle(data, filter);
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
