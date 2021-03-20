/*
 * orientation_kf.c
 *
 *  Created on: Mar 20, 2021
 *      Author: jonas
 */

#include "control/orientation_kf.h"
void quaternion_skew(float* input, float* output) {
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
void quaternion_mat(arm_matrix_instance_f32* input1,
                    arm_matrix_instance_f32* input2,
                    arm_matrix_instance_f32* output) {
  float32_t LQM[16] = {
      input1->pData[0], -input1->pData[1], -input1->pData[2], -input1->pData[3],
      input1->pData[1], input1->pData[0],  -input1->pData[3], input1->pData[2],
      input1->pData[2], input1->pData[3],  input1->pData[0],  -input1->pData[1],
      input1->pData[3], -input1->pData[2], input1->pData[1],  input1->pData[0]};
  arm_matrix_instance_f32 LQM_mat;
  arm_mat_init_f32(&LQM_mat, 3, 3, LQM);

  arm_mat_mult_f32(&LQM_mat, input2, output);
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

void orientation_prediction_step(orientation_filter_t* filter,
                                 imu_data_t* data) {
  /* remove bias from measurement */
  /* SCALE GYRO CORRECTLY */
  filter->velocity_data[1] = data->gyro_x - filter->bias_data[1];
  filter->velocity_data[2] = data->gyro_y - filter->bias_data[2];
  filter->velocity_data[3] = data->gyro_z - filter->bias_data[3];

  /* Prediction step */
  /* x_hat = x_bar + 1/2*Ts(quat_mult(x_bar, velocity)) */
  float32_t holder_data[4] = {0};
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 4, 1, holder_data);
  float32_t holder2_data[4] = {0};
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 4, 1, holder2_data);
  quaternion_mat(&filter->x_bar, &filter->velocity, &holder_mat);

  arm_mat_scale_f32(&holder_mat, (float32_t)(0.5f * filter->t_sampl),
                    &holder2_mat);

  arm_mat_add_f32(&holder2_mat, &filter->x_bar, &filter->x_hat);

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
