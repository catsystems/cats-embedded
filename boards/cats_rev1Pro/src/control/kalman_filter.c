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

#include "control/kalman_filter.h"
#include "cmsis_os.h"
#include <string.h>

void init_filter_struct(kalman_filter_t *const filter) {
  arm_mat_init_f32(&filter->Ad, 3, 3, filter->Ad_data);
  arm_mat_init_f32(&filter->Ad_T, 3, 3, filter->Ad_T_data);
  arm_mat_init_f32(&filter->Gd, 3, 2, filter->Gd_data);
  arm_mat_init_f32(&filter->Bd, 3, 1, filter->Bd_data);
  arm_mat_init_f32(&filter->GdQGd_T, 3, 3, filter->GdQGd_T_data);
  arm_mat_init_f32(&filter->Q, 2, 2, filter->Q_data);
  arm_mat_init_f32(&filter->H_full, 3, 3, filter->H_full_data);
  arm_mat_init_f32(&filter->H_full_T, 3, 3, filter->H_full_T_data);
  arm_mat_init_f32(&filter->H_eliminated, 2, 3, filter->H_eliminated_data);
  arm_mat_init_f32(&filter->H_eliminated_T, 3, 2, filter->H_eliminated_T_data);
  arm_mat_init_f32(&filter->H_2_eliminated, 1, 3, filter->H_2_eliminated_data);
  arm_mat_init_f32(&filter->H_2_eliminated_T, 1, 2, filter->H_2_eliminated_T_data);
  arm_mat_init_f32(&filter->R_full, 3, 3, filter->R_full_data);
  arm_mat_init_f32(&filter->R_eliminated, 2, 2, filter->R_eliminated_data);
  arm_mat_init_f32(&filter->R_2_eliminated, 1, 1, filter->R_2_eliminated_data);
  arm_mat_init_f32(&filter->K_full, 3, 3, filter->K_full_data);
  arm_mat_init_f32(&filter->K_eliminated, 3, 2, filter->K_eliminated_data);
  arm_mat_init_f32(&filter->K_2_eliminated, 3, 1, filter->K_2_eliminated_data);
  arm_mat_init_f32(&filter->P_hat, 3, 3, filter->P_hat_data);
  arm_mat_init_f32(&filter->P_bar, 3, 3, filter->P_bar_data);
  arm_mat_init_f32(&filter->x_hat, 3, 1, filter->x_hat_data);
  arm_mat_init_f32(&filter->x_bar, 3, 1, filter->x_bar_data);
}

void initialize_matrices(kalman_filter_t *const filter) {
  /* Matrix -> mat[9] = [0, 1, 2; 3, 4 , 5; 6, 7, 8];*/

  /* Initialize static values */
  float32_t Ad[9] = {1, filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 0, 1, filter->t_sampl, 0, 0, 1};
  arm_matrix_instance_f32 Ad_mat;
  arm_mat_init_f32(&Ad_mat, 3, 3, Ad);

  float32_t Ad_T[9] = {1, filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 0, 1, filter->t_sampl, 0, 0, 1};
  arm_matrix_instance_f32 Ad_T_mat;
  arm_mat_init_f32(&Ad_T_mat, 3, 3, Ad_T);
  arm_mat_trans_f32(&Ad_mat, &Ad_T_mat);

  float32_t Gd[6] = {filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 1, filter->t_sampl, 0, 1};
  // float32_t Gd[6] = {filter->t_sampl, 0, 1, 0, 0, 0};
  arm_matrix_instance_f32 Gd_mat;
  arm_mat_init_f32(&Gd_mat, 3, 2, Gd);

  float32_t Gd_T[6];
  arm_matrix_instance_f32 Gd_T_mat;
  arm_mat_init_f32(&Gd_T_mat, 2, 3, Gd_T);
  arm_mat_trans_f32(&Gd_mat, &Gd_T_mat);

  float32_t Bd[3] = {filter->t_sampl * filter->t_sampl / 2, filter->t_sampl, 0};
  arm_matrix_instance_f32 Bd_mat;
  arm_mat_init_f32(&Bd_mat, 3, 1, Bd);

  float32_t P_hat[9] = {10.0f, 0, 0, 0, 10.0f, 0, 0, 0, 0.1f};
  arm_matrix_instance_f32 P_hat_mat;
  arm_mat_init_f32(&P_hat_mat, 3, 3, P_hat);

  float32_t P_bar[9] = {10.0f, 0, 0, 0, 10.0f, 0, 0, 0, 0.1f};
  arm_matrix_instance_f32 P_bar_mat;
  arm_mat_init_f32(&P_bar_mat, 3, 3, P_bar);

  float32_t H_full[9] = {1, 0, 0, 1, 0, 0, 1, 0, 0};
  arm_matrix_instance_f32 H_full_mat;
  arm_mat_init_f32(&H_full_mat, 3, 3, H_full);

  float32_t H_eliminated[6] = {1, 0, 0, 1, 0, 0};
  arm_matrix_instance_f32 H_eliminated_mat;
  arm_mat_init_f32(&H_eliminated_mat, 2, 3, H_eliminated);

  float32_t H_2_eliminated[3] = {1, 0, 0};
  arm_matrix_instance_f32 H_2_eliminated_mat;
  arm_mat_init_f32(&H_2_eliminated_mat, 1, 3, H_2_eliminated);

  float32_t Q[4] = {STD_NOISE_IMU, 0, 0, STD_NOISE_OFFSET};
  arm_matrix_instance_f32 Q_mat;
  arm_mat_init_f32(&Q_mat, 2, 2, Q);

  float32_t R_full[9] = {STD_NOISE_BARO, 0, 0, 0, STD_NOISE_BARO, 0, 0, 0, STD_NOISE_BARO};
  arm_matrix_instance_f32 R_full_mat;
  arm_mat_init_f32(&R_full_mat, 3, 3, R_full);

  float32_t R_eliminated[4] = {STD_NOISE_BARO, 0, 0, STD_NOISE_BARO};
  arm_matrix_instance_f32 R_eliminated_mat;
  arm_mat_init_f32(&R_eliminated_mat, 2, 2, R_eliminated);

  float32_t R_2_eliminated[1] = {STD_NOISE_BARO};
  arm_matrix_instance_f32 R_2_eliminated_mat;
  arm_mat_init_f32(&R_2_eliminated_mat, 1, 1, R_2_eliminated);

  float32_t H_full_T[9];
  arm_matrix_instance_f32 H_full_T_mat;
  arm_mat_init_f32(&H_full_T_mat, 3, 3, H_full_T);
  arm_mat_trans_f32(&H_full_mat, &H_full_T_mat);

  float32_t H_eliminated_T[6];
  arm_matrix_instance_f32 H_eliminated_T_mat;
  arm_mat_init_f32(&H_eliminated_T_mat, 3, 2, H_eliminated_T);
  arm_mat_trans_f32(&H_eliminated_mat, &H_eliminated_T_mat);

  float32_t H_2_eliminated_T[3];
  arm_matrix_instance_f32 H_2_eliminated_T_mat;
  arm_mat_init_f32(&H_2_eliminated_T_mat, 3, 1, H_2_eliminated_T);
  arm_mat_trans_f32(&H_2_eliminated_mat, &H_2_eliminated_T_mat);

  float32_t GdQGd_T[9];
  arm_matrix_instance_f32 GdQGd_T_mat;
  arm_mat_init_f32(&GdQGd_T_mat, 3, 3, GdQGd_T);

  float32_t holder[6];
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 3, 2, holder);

  arm_mat_mult_f32(&Gd_mat, &Q_mat, &holder_mat);
  arm_mat_mult_f32(&holder_mat, &Gd_T_mat, &GdQGd_T_mat);

  float32_t K_full[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  arm_matrix_instance_f32 K_full_mat;
  arm_mat_init_f32(&K_full_mat, 3, 3, K_full);

  float32_t K_eliminated[6] = {0, 0, 0, 0, 0, 0};
  arm_matrix_instance_f32 K_eliminated_mat;
  arm_mat_init_f32(&K_eliminated_mat, 3, 2, K_eliminated);

  float32_t K_2_eliminated[3] = {0, 0, 0};
  arm_matrix_instance_f32 K_2_eliminated_mat;
  arm_mat_init_f32(&K_2_eliminated_mat, 3, 1, K_2_eliminated);

  float32_t x_bar[3] = {0, 0, 0};
  arm_matrix_instance_f32 x_bar_mat;
  arm_mat_init_f32(&x_bar_mat, 3, 1, x_bar);

  float32_t x_hat[3] = {0, 0, 0};
  arm_matrix_instance_f32 x_hat_mat;
  arm_mat_init_f32(&x_hat_mat, 3, 1, x_hat);

  memcpy(filter->Ad_data, Ad, sizeof(Ad));
  memcpy(filter->Ad_T_data, Ad_T, sizeof(Ad_T));
  memcpy(filter->Gd_data, Gd, sizeof(Gd));
  memcpy(filter->GdQGd_T_data, GdQGd_T, sizeof(GdQGd_T));
  memcpy(filter->Bd_data, Bd, sizeof(Bd));
  memcpy(filter->P_hat_data, P_hat, sizeof(P_hat));
  memcpy(filter->P_bar_data, P_bar, sizeof(P_bar));
  memcpy(filter->Q_data, Q, sizeof(Q));
  memcpy(filter->H_full_data, H_full, sizeof(H_full));
  memcpy(filter->H_full_T_data, H_full_T, sizeof(H_full_T));
  memcpy(filter->H_eliminated_data, H_eliminated, sizeof(H_eliminated));
  memcpy(filter->H_eliminated_T_data, H_eliminated_T, sizeof(H_eliminated_T));
  memcpy(filter->R_full_data, R_full, sizeof(R_full));
  memcpy(filter->R_eliminated_data, R_eliminated, sizeof(R_eliminated));
  memcpy(filter->R_2_eliminated_data, R_2_eliminated, sizeof(R_2_eliminated));
  memcpy(filter->K_eliminated_data, K_eliminated, sizeof(K_eliminated));
  memcpy(filter->K_full_data, K_full, sizeof(K_full));
  memcpy(filter->x_bar_data, x_bar, sizeof(x_bar));
  memcpy(filter->x_hat_data, x_hat, sizeof(x_hat));
}

void reset_kalman(kalman_filter_t *filter, float initial_pressure) {
  log_debug("Resetting Kalman Filter...");
  float32_t x_dash[3] = {0, 10.0f, 0};
  float32_t P_dash[9] = {10.0f, 0, 0, 0, 10.0f, 0, 0, 0, 0.1f};

  filter->pressure_0 = initial_pressure;
  memcpy(filter->P_bar_data, P_dash, sizeof(P_dash));
  memcpy(filter->P_hat_data, P_dash, sizeof(P_dash));
  memcpy(filter->x_bar_data, x_dash, sizeof(x_dash));
  memcpy(filter->x_bar_data, x_dash, sizeof(x_dash));
}

/* This Function Implements the kalman Prediction as long as more than 0 IMU
 * work */
void kalman_prediction(kalman_filter_t *filter, state_estimation_data_t *data, sensor_elimination_t *elimination,
                       flight_fsm_e fsm_state) {
  float u = 0;
  float32_t holder[9];
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 3, 3, holder);

  float32_t holder2[9];
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 3, 3, holder2);

  float32_t holder_data[3];
  arm_matrix_instance_f32 holder_vec;
  arm_mat_init_f32(&holder_vec, 3, 1, holder_data);

  float32_t holder2_data[3];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 3, 1, holder2_data);

  /* Prediction Step */

  /* Average Acceleration */
  int counter_acc = 0;
  /* check if we are in high acceleration mode */
  if (elimination->high_acc) {
    u = data->acceleration[HIGH_G_ACC_INDEX];
    counter_acc++;
  } else {
    /* Check if we have ruled out an accelerometer */
    /* if we only have one accelerometer left, use it */
    if (elimination->num_faulty_accel == (NUM_ACC - 1)) {
      for (int i = 0; i < NUM_ACC; i++) {
        if (elimination->faulty_accel[i] == 0) {
          u += data->acceleration[i];
          counter_acc++;
        }
      }
    }
    /* if we have at least two left, only use low acc imus */
    else {
      for (int i = 0; i < NUM_ACC; i++) {
        if ((elimination->faulty_accel[i] == 0) && (i != HIGH_G_ACC_INDEX)) {
          u += data->acceleration[i];
          counter_acc++;
        }
      }
    }
  }
  if(counter_acc == 0){
      return;
  }

  if (fsm_state > APOGEE) {
    u = 0;
  } else {
    u /= (float)(counter_acc);
  }

  /* Calculate Prediction of the state: x_hat = A*x_bar + B*u */
  arm_mat_mult_f32(&filter->Ad, &filter->x_bar, &holder_vec);
  arm_mat_scale_f32(&filter->Bd, (float32_t)(u), &holder2_vec);
  arm_mat_add_f32(&holder_vec, &holder2_vec, &filter->x_hat);

  /* Update the Variance of the state P_hat = A*P_bar*A' + GQG' */
  arm_mat_mult_f32(&filter->Ad, &filter->P_bar, &holder_mat);
  arm_mat_mult_f32(&holder_mat, &filter->Ad_T, &holder2_mat);
  arm_mat_add_f32(&holder2_mat, &filter->GdQGd_T, &filter->P_hat);

  /* Prediction Step finished */
}

/* This function implements the Kalman update when no Barometer is faulty */
cats_error_e kalman_update_full(kalman_filter_t *filter, state_estimation_data_t *data) {
  float32_t holder[9];
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 3, 3, holder);

  float32_t holder2[9];
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 3, 3, holder2);

  float32_t holder3[9];
  arm_matrix_instance_f32 holder3_mat;
  arm_mat_init_f32(&holder3_mat, 3, 3, holder3);

  float32_t holder_data[3];
  arm_matrix_instance_f32 holder_vec;
  arm_mat_init_f32(&holder_vec, 3, 1, holder_data);

  float32_t holder2_data[3];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 3, 1, holder2_data);
  cats_error_e status = CATS_ERR_OK;

  /* Update Step */

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  arm_mat_mult_f32(&filter->H_full, &filter->P_hat, &holder_mat);
  arm_mat_mult_f32(&holder_mat, &filter->H_full_T, &holder2_mat);
  arm_mat_add_f32(&holder2_mat, &filter->R_full, &holder_mat);
  arm_status inv_status = arm_mat_inverse_f32(&holder_mat, &holder2_mat);

  arm_mat_mult_f32(&filter->P_hat, &filter->H_full_T, &holder_mat);
  arm_mat_mult_f32(&holder_mat, &holder2_mat, &filter->K_full);

  /* if the matrix is singular, return an error */
  if (inv_status == ARM_MATH_SINGULAR) {
    status = CATS_ERR_FILTER;
  }

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */

  float32_t z[3] = {(float32_t)data->calculated_AGL[0], (float32_t)data->calculated_AGL[1],
                    (float32_t)data->calculated_AGL[2]};
  arm_matrix_instance_f32 z_vec;
  arm_mat_init_f32(&z_vec, 3, 1, z);

  arm_mat_mult_f32(&filter->H_full, &filter->x_hat, &holder_vec);
  arm_mat_sub_f32(&z_vec, &holder_vec, &holder2_vec);
  arm_mat_mult_f32(&filter->K_full, &holder2_vec, &holder_vec);
  arm_mat_add_f32(&holder_vec, &filter->x_hat, &filter->x_bar);

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */
  float32_t eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  arm_matrix_instance_f32 eye_mat;
  arm_mat_init_f32(&eye_mat, 3, 3, eye);

  arm_mat_mult_f32(&filter->K_full, &filter->H_full, &holder_mat);
  arm_mat_sub_f32(&eye_mat, &holder_mat, &holder2_mat);
  arm_mat_mult_f32(&holder2_mat, &filter->P_hat, &filter->P_bar);

  /* Finished Calculating P_bar */

  return status;
}

/* This function implements the Kalman update when one Barometer is faulty */
cats_error_e kalman_update_eliminated(kalman_filter_t *filter, state_estimation_data_t *data,
                                      sensor_elimination_t *elimination) {
  /* Placeholder Matrices */

  float32_t holder_0_3x3[9];
  arm_matrix_instance_f32 holder_0_3x3_mat;
  arm_mat_init_f32(&holder_0_3x3_mat, 3, 3, holder_0_3x3);

  float32_t holder_1_3x3[9];
  arm_matrix_instance_f32 holder_1_3x3_mat;
  arm_mat_init_f32(&holder_1_3x3_mat, 3, 3, holder_1_3x3);

  float32_t holder_0_2x2[4];
  arm_matrix_instance_f32 holder_0_2x2_mat;
  arm_mat_init_f32(&holder_0_2x2_mat, 2, 2, holder_0_2x2);

  float32_t holder_1_2x2[4];
  arm_matrix_instance_f32 holder_1_2x2_mat;
  arm_mat_init_f32(&holder_1_2x2_mat, 2, 2, holder_1_2x2);

  float32_t holder_0_2x3[6];
  arm_matrix_instance_f32 holder_0_2x3_mat;
  arm_mat_init_f32(&holder_0_2x3_mat, 2, 3, holder_0_2x3);

  float32_t holder_1_2x3[6];
  arm_matrix_instance_f32 holder_1_2x3_mat;
  arm_mat_init_f32(&holder_1_2x3_mat, 2, 3, holder_1_2x3);

  float32_t holder_0_3x2[6];
  arm_matrix_instance_f32 holder_0_3x2_mat;
  arm_mat_init_f32(&holder_0_3x2_mat, 3, 2, holder_0_3x2);

  float32_t holder_data[2];
  arm_matrix_instance_f32 holder_vec;
  arm_mat_init_f32(&holder_vec, 2, 1, holder_data);

  float32_t holder2_data[2];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 2, 1, holder2_data);
  float32_t holder3_data[3];
  arm_matrix_instance_f32 holder3_vec;
  arm_mat_init_f32(&holder3_vec, 3, 1, holder3_data);
  cats_error_e status = CATS_ERR_OK;

  /* Update Step */

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  arm_mat_mult_f32(&filter->H_eliminated, &filter->P_hat, &holder_0_2x3_mat);
  arm_mat_mult_f32(&holder_0_2x3_mat, &filter->H_eliminated_T, &holder_0_2x2_mat);
  arm_mat_add_f32(&holder_0_2x2_mat, &filter->R_eliminated, &holder_1_2x2_mat);
  arm_status inv_status = arm_mat_inverse_f32(&holder_1_2x2_mat, &holder_0_2x2_mat);

  arm_mat_mult_f32(&filter->P_hat, &filter->H_eliminated_T, &holder_0_3x2_mat);
  arm_mat_mult_f32(&holder_0_3x2_mat, &holder_0_2x2_mat, &filter->K_eliminated);

  /* if the matrix is singular, return an error */
  if (inv_status == ARM_MATH_SINGULAR) {
    status = CATS_ERR_FILTER;
  }

  /* Finished Calculating K */

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */
  float32_t z[2];
  uint8_t counter = 0;
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_baro[i] == 0) {
      z[counter] = (float32_t)data->calculated_AGL[i];
      counter++;
    }
  }

  arm_matrix_instance_f32 z_vec;
  arm_mat_init_f32(&z_vec, 2, 1, z);

  arm_mat_mult_f32(&filter->H_eliminated, &filter->x_hat, &holder_vec);
  arm_mat_sub_f32(&z_vec, &holder_vec, &holder2_vec);
  arm_mat_mult_f32(&filter->K_eliminated, &holder2_vec, &holder3_vec);
  arm_mat_add_f32(&holder3_vec, &filter->x_hat, &filter->x_bar);

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */
  float32_t eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  arm_matrix_instance_f32 eye_mat;
  arm_mat_init_f32(&eye_mat, 3, 3, eye);

  arm_mat_mult_f32(&filter->K_eliminated, &filter->H_eliminated, &holder_0_3x3_mat);
  arm_mat_sub_f32(&eye_mat, &holder_0_3x3_mat, &holder_1_3x3_mat);
  arm_mat_mult_f32(&holder_1_3x3_mat, &filter->P_hat, &filter->P_bar);

  return status;
}

/* This function implements the Kalman update when one Barometer is faulty */
cats_error_e kalman_update_2_eliminated(kalman_filter_t *filter, state_estimation_data_t *data,
                                        sensor_elimination_t *elimination) {
  /* Placeholder Matrices */

  float32_t holder_0_3x3[9];
  arm_matrix_instance_f32 holder_0_3x3_mat;
  arm_mat_init_f32(&holder_0_3x3_mat, 3, 3, holder_0_3x3);

  float32_t holder_1_3x3[9];
  arm_matrix_instance_f32 holder_1_3x3_mat;
  arm_mat_init_f32(&holder_1_3x3_mat, 3, 3, holder_1_3x3);

  float32_t holder_0_1x1[1];
  arm_matrix_instance_f32 holder_0_1x1_mat;
  arm_mat_init_f32(&holder_0_1x1_mat, 1, 1, holder_0_1x1);

  float32_t holder_1_1x1[1];
  arm_matrix_instance_f32 holder_1_1x1_mat;
  arm_mat_init_f32(&holder_1_1x1_mat, 1, 1, holder_1_1x1);

  float32_t holder_0_1x3[3];
  arm_matrix_instance_f32 holder_0_1x3_mat;
  arm_mat_init_f32(&holder_0_1x3_mat, 1, 3, holder_0_1x3);

  float32_t holder_1_1x3[3];
  arm_matrix_instance_f32 holder_1_1x3_mat;
  arm_mat_init_f32(&holder_1_1x3_mat, 1, 3, holder_1_1x3);

  float32_t holder_0_3x1[3];
  arm_matrix_instance_f32 holder_0_3x1_mat;
  arm_mat_init_f32(&holder_0_3x1_mat, 3, 1, holder_0_3x1);

  float32_t holder_data[1];
  arm_matrix_instance_f32 holder_vec;
  arm_mat_init_f32(&holder_vec, 1, 1, holder_data);

  float32_t holder2_data[1];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 1, 1, holder2_data);
  float32_t holder3_data[3] = {0, 0, 0};
  arm_matrix_instance_f32 holder3_vec;
  arm_mat_init_f32(&holder3_vec, 3, 1, holder3_data);
  cats_error_e status = CATS_ERR_OK;

  /* Update Step */

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  arm_mat_mult_f32(&filter->H_2_eliminated, &filter->P_hat, &holder_0_1x3_mat);
  arm_mat_mult_f32(&holder_0_1x3_mat, &filter->H_2_eliminated_T, &holder_0_1x1_mat);
  arm_mat_add_f32(&holder_0_1x1_mat, &filter->R_2_eliminated, &holder_1_1x1_mat);
  arm_status inv_status = arm_mat_inverse_f32(&holder_1_1x1_mat, &holder_0_1x1_mat);

  arm_mat_mult_f32(&filter->P_hat, &filter->H_2_eliminated_T, &holder_0_3x1_mat);
  arm_mat_mult_f32(&holder_0_3x1_mat, &holder_0_1x1_mat, &filter->K_2_eliminated);

  /* if the matrix is singular, return an error */
  if (inv_status == ARM_MATH_SINGULAR) {
    status = CATS_ERR_FILTER;
  }

  /* Finished Calculating K */

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */
  float32_t z[1];
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_baro[i] == 0) {
      z[0] = (float32_t)data->calculated_AGL[i];
    }
  }

  arm_matrix_instance_f32 z_vec;
  arm_mat_init_f32(&z_vec, 1, 1, z);

  arm_mat_mult_f32(&filter->H_2_eliminated, &filter->x_hat, &holder_vec);
  arm_mat_sub_f32(&z_vec, &holder_vec, &holder2_vec);
  arm_mat_mult_f32(&filter->K_2_eliminated, &holder2_vec, &holder3_vec);
  arm_mat_add_f32(&holder3_vec, &filter->x_hat, &filter->x_bar);

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */
  float32_t eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  arm_matrix_instance_f32 eye_mat;
  arm_mat_init_f32(&eye_mat, 3, 3, eye);

  arm_mat_mult_f32(&filter->K_2_eliminated, &filter->H_2_eliminated, &holder_0_3x3_mat);
  arm_mat_sub_f32(&eye_mat, &holder_0_3x3_mat, &holder_1_3x3_mat);
  arm_mat_mult_f32(&holder_1_3x3_mat, &filter->P_hat, &filter->P_bar);

  return status;
}

cats_error_e kalman_step(kalman_filter_t *filter, state_estimation_data_t *data, sensor_elimination_t *elimination,
                         flight_fsm_e fsm_state) {
  cats_error_e status;

  kalman_prediction(filter, data, elimination, fsm_state);

  switch (NUM_PRESSURE - elimination->num_faulty_baros) {
    case 3:
      status = kalman_update_full(filter, data);
      break;
    case 2:
      status = kalman_update_eliminated(filter, data, elimination);
      break;
    case 1:
      status = kalman_update_2_eliminated(filter, data, elimination);
      break;
    case 0:
      status = CATS_ERR_FILTER;
      break;
    default:
      status = CATS_ERR_FILTER;
      break;
  }
  return status;
}
