/*
 * kalman_filter.c
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#include "control\kalman_filter.h"
#include "cmsis_os.h"
#include <string.h>

void initialize_matrices(kalman_filter_t *const filter) {
  /* Initialize static values */
  kalman_filter_t temp_filter = {
      .Ad = {{1, filter->t_sampl, filter->t_sampl * filter->t_sampl / 2},
             {0, 1, filter->t_sampl},
             {0, 0, 1}},
      .Gd = {filter->t_sampl * filter->t_sampl / 2, filter->t_sampl, 0},
      .Bd = {filter->t_sampl * filter->t_sampl / 2, filter->t_sampl, 0},
      .P_hat = {{10.0f, 0, 0}, {10.0f, 0}, {0, 0, 10.0f}},
      .P_bar = {{10.0f, 0, 0}, {10.0f, 0}, {0, 0, 10.0f}},
      .Q = 0.1f,
      .H_full = {{1, 0, 0}, {1, 0, 0}, {1, 0, 0}},
      .H_eliminated = {{1, 0, 0}, {1, 0, 0}},
      .R_full = {{1.0f, 0, 0}, {0, 1.0f, 0}, {0, 0, 1.0f}},
      .R_eliminated = {{1.0f, 0}, {0, 1.0f}},
      .pressure_0 = filter->pressure_0,
      .t_sampl = filter->t_sampl};

  /* Transpose matrices */
  transpose(3, 3, temp_filter.Ad, temp_filter.Ad_T);
  SAM **transpose(3, 3, temp_filter.H_full, temp_filter.H_full_T);
  transpose(2, 3, temp_filter.H_eliminated, temp_filter.H_eliminated_T);

  /* Fill up GdQGd_T */
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      temp_filter.GdQGd_T[i][j] =
          temp_filter.Q * temp_filter.Gd[i] * temp_filter.Gd[j];
    }
  }

  /* Copy over the entire temp struct to filter */
  memcpy(filter, &temp_filter, sizeof(kalman_filter_t));
}

void reset_kalman(kalman_filter_t *filter, int32_t initial_pressure) {
  log_debug("Resetting Kalman Filter...");
  float x_dash[3] = {0, 0, 0};
  float P_dash[3][3] = {{10.0f, 0, 0}, {0, 10.0f, 0}, {0, 0, 10.0f}};

  filter->pressure_0 = (float)initial_pressure;

  const size_t flt_3x3_size = 9 * sizeof(float);
  const size_t flt_3_size = 3 * sizeof(float);
  memcpy(filter->P_hat, P_dash, flt_3x3_size);
  memcpy(filter->P_bar, P_dash, flt_3x3_size);
  memcpy(filter->x_hat, x_dash, flt_3_size);
  memcpy(filter->x_bar, x_dash, flt_3_size);
}

/* This Function Implements the kalman Prediction as long as more than 0 IMU
 * work */
void kalman_prediction(kalman_filter_t *filter, state_estimation_data_t *data,
                       sensor_elimination_t *elimination) {
  float u = 0;
  float placeholder_mat[3][3] = {0};

  /* Prediction Step */

  memset(filter->x_hat, 0, sizeof(filter->x_hat));
  memset(filter->P_hat, 0, sizeof(filter->P_hat));

  /* Average Acceleration */
  /* Check if we have ruled out an accelerometer */
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_imu[i] == 0) {
      u += data->acceleration[i];
    }
  }

  u /= (float)(3 - elimination->num_faulty_imus);

  /* Calculate Prediction of the state: x_hat = A*x_bar + B*u */
  for (int i = 0; i < 3; i++) {
    filter->x_hat[i] = u * filter->Bd[i];

    for (int j = 0; j < 3; j++) {
      filter->x_hat[i] += filter->Ad[i][j] * filter->x_bar[j];
    }
  }

  /* Update the Variance of the state P_hat = A*P_bar*A' + GQG' */
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat[i][j] += filter->Ad[i][k] * filter->P_bar[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        filter->P_hat[i][j] += placeholder_mat[i][k] * filter->Ad_T[k][j];
      }
      filter->P_hat[i][j] += filter->GdQGd_T[i][j];
    }
  }

  /* Prediction Step finished */
}

/* This function implements the Kalman update when no Barometer is faulty */
cats_status_e kalman_update_full(kalman_filter_t *filter,
                                 state_estimation_data_t *data) {
  float placeholder_vec[3] = {0};
  float placeholder_mat[3][3] = {0};
  float placeholder_mat2[3][3] = {0};
  float placeholder_mat3[3][3] = {0};
  cats_status_e status = CATS_OK;

  /* Update Step */
  float old_K[3][3] = {0};

  memcpy(old_K, filter->K_full, sizeof(old_K));

  memset(filter->K_full, 0, sizeof(filter->K_full));

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat[i][j] += filter->H_full[i][k] * filter->P_hat[k][j];
        placeholder_mat3[i][j] += filter->P_hat[i][k] * filter->H_full_T[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat2[i][j] +=
            placeholder_mat[i][k] * filter->H_full_T[k][j];
      }
      placeholder_mat2[i][j] += filter->R_full[i][j];
    }
  }

  memset(placeholder_mat, 0, sizeof(placeholder_mat));

  status = inverse(3, placeholder_mat2, placeholder_mat, 0);

  /* if the matrix is singular, return an error and ignore this step */
  if (status != CATS_OK) {
    memcpy(filter->K_full, old_K, sizeof(old_K));
    return CATS_FILTER_ERROR;
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        filter->K_full[i][j] += placeholder_mat3[i][k] * placeholder_mat[k][j];
      }
    }
  }

  /* Finished Calculating K */

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */

  memset(filter->x_bar, 0, sizeof(filter->x_bar));

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      placeholder_vec[i] += filter->H_full[i][j] * filter->x_hat[j];
    }
    placeholder_vec[i] = data->calculated_AGL[i] - placeholder_vec[i];
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      filter->x_bar[i] += filter->K_full[i][j] * placeholder_vec[j];
    }
    filter->x_bar[i] += filter->x_hat[i];
  }

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */

  memset(filter->P_bar, 0, sizeof(filter->P_bar));
  memset(placeholder_mat, 0, sizeof(placeholder_mat));
  memset(placeholder_mat2, 0, sizeof(placeholder_mat2));

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat2[i][j] += filter->K_full[i][k] * filter->H_full[k][j];
      }
      if (i == j) {
        placeholder_mat[i][j] = 1 - placeholder_mat2[i][j];
      } else {
        placeholder_mat[i][j] = -placeholder_mat2[i][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        filter->P_bar[i][j] += placeholder_mat[i][k] * filter->P_hat[k][j];
      }
    }
  }
  /* Finished Calculating P_bar */

  return status;
}

/* This function implements the Kalman update when one Barometer is faulty */
cats_status_e kalman_update_eliminated(kalman_filter_t *filter,
                                       state_estimation_data_t *data,
                                       sensor_elimination_t *elimination) {
  float placeholder_vec[2] = {0};
  float placeholder_mat[2][3] = {0};
  float placeholder_mat2[2][2] = {0};
  float placeholder_mat3[3][2] = {0};
  float placeholder_mat4[2][2] = {0};
  float placeholder_mat5[3][3] = {0};
  float placeholder_mat6[3][3] = {0};
  cats_status_e status = CATS_OK;

  /* Update Step */
  float old_K[3][2] = {0};

  memcpy(old_K, filter->K_eliminated, sizeof(old_K));

  memset(filter->K_eliminated, 0, sizeof(filter->K_eliminated));

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat[i][j] +=
            filter->H_eliminated[i][k] * filter->P_hat[k][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat3[i][j] +=
            filter->P_hat[i][k] * filter->H_eliminated_T[k][j];
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 3; k++) {
        placeholder_mat2[i][j] +=
            placeholder_mat[i][k] * filter->H_eliminated_T[k][j];
      }
      placeholder_mat2[i][j] += filter->R_eliminated[i][j];
    }
  }

  memset(placeholder_mat, 0, sizeof(placeholder_mat));

  status = inverse(2, placeholder_mat2, placeholder_mat4, 0);

  /* if the matrix is singular, return an error and ignore this step */
  if (status != CATS_OK) {
    memcpy(filter->K_eliminated, old_K, sizeof(old_K));
    return CATS_FILTER_ERROR;
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        filter->K_eliminated[i][j] +=
            placeholder_mat3[i][k] * placeholder_mat4[k][j];
      }
    }
  }

  /* Finished Calculating K */

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */

  memset(filter->x_bar, 0, sizeof(filter->x_bar));

  /* prepare data vector with bad sensor reading */
  float data_vec[2];
  uint8_t counter = 0;
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_baro[i] == 0) {
      data_vec[counter] = data->calculated_AGL[i];
      counter++;
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      placeholder_vec[i] += filter->H_eliminated[i][j] * filter->x_hat[j];
    }
    placeholder_vec[i] = data_vec[i] - placeholder_vec[i];
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      filter->x_bar[i] += filter->K_eliminated[i][j] * placeholder_vec[j];
    }
    filter->x_bar[i] += filter->x_hat[i];
  }

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */

  memset(filter->P_bar, 0, sizeof(filter->P_bar));
  memset(placeholder_mat, 0, sizeof(placeholder_mat));
  memset(placeholder_mat2, 0, sizeof(placeholder_mat2));

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 2; k++) {
        placeholder_mat5[i][j] +=
            filter->K_eliminated[i][k] * filter->H_eliminated[k][j];
      }
      if (i == j) {
        placeholder_mat6[i][j] = 1 - placeholder_mat5[i][j];
      } else {
        placeholder_mat6[i][j] = -placeholder_mat5[i][j];
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        filter->P_bar[i][j] += placeholder_mat6[i][k] * filter->P_hat[k][j];
      }
    }
  }
  /* Finished Calculating P_bar */

  return status;
}

cats_status_e kalman_step(kalman_filter_t *filter,
                          state_estimation_data_t *data,
                          sensor_elimination_t *elimination) {
  cats_status_e status = CATS_OK;

  kalman_prediction(filter, data, elimination);

  switch (elimination->num_faulty_baros) {
    case 0:
      status = kalman_update_full(filter, data);
      break;
    case 1:
      status = kalman_update_eliminated(filter, data, elimination);
      break;
    case 2:
      status = CATS_FILTER_ERROR;
      break;
    case 3:
      status = CATS_FILTER_ERROR;
      break;
    default:
      status = CATS_FILTER_ERROR;
      break;
  }
  if (elimination->num_faulty_baros > 1) {
    log_error("Kalman step faulty baros: %d", elimination->num_faulty_baros);
  }
  if (elimination->num_faulty_imus > 1) {
    log_error("Faulty IMUs: %d", elimination->num_faulty_imus);
  }
  return status;
}
