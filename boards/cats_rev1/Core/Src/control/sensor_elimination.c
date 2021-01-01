/*
 * sensor_elimination.c
 *
 *  Created on: Dec 20, 2020
 *      Author: Jonas
 */

#include "control/sensor_elimination.h"
#include "util/log.h"
#include <stdlib.h>
#include <math.h>

cats_status_e check_sensors(state_estimation_data_t *data,
                            sensor_elimination_t *elimination) {
  cats_status_e status = CATS_OK;

  switch (elimination->num_faulty_imus) {
    case 0:
      status = check_imus_no_faults(data, elimination);
      break;
    case 1:
      status = check_imus_1_fault(data, elimination);
      break;
    case 2:
      break;
    default:
      break;
  }

  switch (elimination->num_faulty_baros) {
    case 0:
      status = check_baros_no_faults(data, elimination);
      break;
    case 1:
      status = check_baros_1_fault(data, elimination);
      break;
    case 2:
      break;
    default:
      break;
  }
  return status;
}

cats_status_e check_imus_no_faults(state_estimation_data_t *data,
                                   sensor_elimination_t *elimination) {
  /* Check bounds */
  for (int i = 0; i < 3; i++) {
    if ((data->acceleration[i] > UPPER_BOUND_ACC) ||
        (data->acceleration[i] < LOWER_BOUND_ACC)) {
      elimination->faulty_imu[i] = 1;
    }
  }

  /* Check freezing of Sensor */
  for (int i = 0; i < 3; i++) {
    /* Acceleration */
    if (data->acceleration[i] == elimination->last_value[i]) {
      elimination->num_freeze[i] += 1;
      if (elimination->num_freeze[i] > MAX_NUM_SAME_VALUE_IMU) {
        elimination->faulty_imu[i] = 1;
      }
    } else {
      elimination->last_value[i] = data->acceleration[i];
      elimination->num_freeze[i] = 0;
    }
  }

  /* Do Majority Voting */
  float error[3] = {0};

  /* Acceleration */
  error[0] = fabsf(data->acceleration[0] - data->acceleration[1]);
  error[1] = fabsf(data->acceleration[1] - data->acceleration[2]);
  error[2] = fabsf(data->acceleration[0] - data->acceleration[2]);
  if ((error[0] > MAJ_VOTE_IMU_ERROR) && (error[2] > MAJ_VOTE_IMU_ERROR)) {
    /* IMU 0 probably faulty */
    elimination->num_maj_vote[0] += 1;
    if (elimination->num_maj_vote[0] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_imu[0] = 1;
    }
  } else {
    elimination->num_maj_vote[0] = 0;
  }
  if ((error[0] > MAJ_VOTE_IMU_ERROR) && (error[1] > MAJ_VOTE_IMU_ERROR)) {
    /* IMU 1 probably faulty */
    elimination->num_maj_vote[1] += 1;
    if (elimination->num_maj_vote[1] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_imu[1] = 1;
    }
  } else {
    elimination->num_maj_vote[1] = 0;
  }
  if ((error[1] > MAJ_VOTE_IMU_ERROR) && (error[2] > MAJ_VOTE_IMU_ERROR)) {
    /* IMU 2 probably faulty */
    elimination->num_maj_vote[2] += 1;
    if (elimination->num_maj_vote[2] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_imu[2] = 1;
    }
  } else {
    elimination->num_maj_vote[2] = 0;
  }

  /* Add Up all the faulty IMUs */
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_imu[i] == 1) {
      elimination->num_faulty_imus += 1;
    }
  }

  switch (elimination->num_faulty_imus) {
    case 0:
      return CATS_OK;
    case 1:
      return CATS_IMU_ERROR;
    case 2:
      return CATS_FILTER_ERROR;
    default:
      return CATS_FILTER_ERROR;
  }
}

cats_status_e check_baros_no_faults(state_estimation_data_t *data,
                                    sensor_elimination_t *elimination) {
  /* Check bounds */
  for (int i = 0; i < 3; i++) {
    if ((data->pressure[i] > UPPER_BOUND_PRESSURE) ||
        (data->pressure[i] < LOWER_BOUND_PRESSURE) ||
        (data->temperature[i] > UPPER_BOUND_TEMPERATURE) ||
        (data->temperature[i] < LOWER_BOUND_TEMPERATURE)) {
      elimination->faulty_baro[i] = 1;
    }
  }

  /* Check freezing of Sensor */
  for (int i = 0; i < 3; i++) {
    /* Pressure */
    if (data->pressure[i] == elimination->last_value[i + 3]) {
      elimination->num_freeze[i + 3] += 1;
      if (elimination->num_freeze[i + 3] > MAX_NUM_SAME_VALUE_PRESSURE) {
        elimination->faulty_baro[i] = 1;
      }
    } else {
      elimination->last_value[i + 3] = data->pressure[i];
      elimination->num_freeze[i + 3] = 0;
    }
    /* Temperature */
    if (data->temperature[i] == elimination->last_value[i + 6]) {
      elimination->num_freeze[i + 6] += 1;
      if (elimination->num_freeze[i + 6] > MAX_NUM_SAME_VALUE_TEMPERATURE) {
        elimination->faulty_baro[i] = 1;
      }
    } else {
      elimination->last_value[i + 6] = data->temperature[i];
      elimination->num_freeze[i + 6] = 0;
    }
  }

  /* Do Majority Voting */
  float error[3] = {0};

  /* Barometer */
  error[0] = fabsf(data->pressure[0] - data->pressure[1]);
  error[1] = fabsf(data->pressure[1] - data->pressure[2]);
  error[2] = fabsf(data->pressure[0] - data->pressure[2]);
  if ((error[0] > MAJ_VOTE_PRESSURE_ERROR) &&
      (error[2] > MAJ_VOTE_PRESSURE_ERROR)) {
    /* Baro 0 probably faulty */
    elimination->num_maj_vote[3] += 1;
    if (elimination->num_maj_vote[3] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[0] = 1;
    }
  } else {
    elimination->num_maj_vote[3] = 0;
  }
  if ((error[0] > MAJ_VOTE_PRESSURE_ERROR) &&
      (error[1] > MAJ_VOTE_PRESSURE_ERROR)) {
    /* Baro 1 probably faulty */
    elimination->num_maj_vote[4] += 1;
    if (elimination->num_maj_vote[4] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[1] = 1;
    }
  } else {
    elimination->num_maj_vote[4] = 0;
  }
  if ((error[1] > MAJ_VOTE_PRESSURE_ERROR) &&
      (error[2] > MAJ_VOTE_PRESSURE_ERROR)) {
    /* Baro 2 probably faulty */
    elimination->num_maj_vote[5] += 1;
    if (elimination->num_maj_vote[5] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[2] = 1;
    }
  } else {
    elimination->num_maj_vote[5] = 0;
  }

  /* Temperature */
  error[0] = fabsf(data->temperature[0] - data->temperature[1]);
  error[1] = fabsf(data->temperature[1] - data->temperature[2]);
  error[2] = fabsf(data->temperature[0] - data->temperature[2]);
  if ((error[0] > MAJ_VOTE_TEMPERATURE_ERROR) &&
      (error[2] > MAJ_VOTE_TEMPERATURE_ERROR)) {
    /* Baro 0 probably faulty */
    elimination->num_maj_vote[6] += 1;
    if (elimination->num_maj_vote[6] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[0] = 1;
    }
  } else {
    elimination->num_maj_vote[6] = 0;
  }
  if ((error[0] > MAJ_VOTE_TEMPERATURE_ERROR) &&
      (error[1] > MAJ_VOTE_TEMPERATURE_ERROR)) {
    /* Baro 1 probably faulty */
    elimination->num_maj_vote[7] += 1;
    if (elimination->num_maj_vote[7] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[1] = 1;
    }
  } else {
    elimination->num_maj_vote[7] = 0;
  }
  if ((error[1] > MAJ_VOTE_TEMPERATURE_ERROR) &&
      (error[2] > MAJ_VOTE_TEMPERATURE_ERROR)) {
    /* Baro 2 probably faulty */
    elimination->num_maj_vote[8] += 1;
    if (elimination->num_maj_vote[8] > MAJ_VOTE_NUM_VALUES) {
      elimination->faulty_baro[2] = 1;
    }
  } else {
    elimination->num_maj_vote[8] = 0;
  }

  /* Add Up all the faulty Baros */
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_baro[i] == 1) {
      elimination->num_faulty_baros += 1;
    }
  }

  switch (elimination->num_faulty_baros) {
    case 0:
      return CATS_OK;
    case 1:
      return CATS_BARO_ERROR;
    case 2:
      return CATS_FILTER_ERROR;
    default:
      return CATS_FILTER_ERROR;
  }
}

cats_status_e check_imus_1_fault(state_estimation_data_t *data,
                                 sensor_elimination_t *elimination) {
  /* Check bounds */
  for (int i = 0; i < 3; i++) {
    if ((data->acceleration[i] > UPPER_BOUND_ACC) ||
        (data->acceleration[i] < LOWER_BOUND_ACC)) {
      elimination->faulty_imu[i] = 1;
    }
  }

  /* Check freezing of Sensor */
  for (int i = 0; i < 3; i++) {
    /* Acceleration */
    if (data->acceleration[i] == elimination->last_value[i]) {
      elimination->num_freeze[i] += 1;
      if (elimination->num_freeze[i] > MAX_NUM_SAME_VALUE_IMU) {
        elimination->faulty_imu[i] = 1;
      }
    } else {
      elimination->last_value[i] = data->acceleration[i];
      elimination->num_freeze[i] = 0;
    }
  }

  /* Add Up all the faulty IMUs */
  uint8_t counter = 0;
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_imu[i] == 1) {
      counter += 1;
    }
  }
  elimination->num_faulty_imus = counter;

  switch (elimination->num_faulty_imus) {
    case 0:
      return CATS_OK;
    case 1:
      return CATS_IMU_ERROR;
    case 2:
      return CATS_FILTER_ERROR;
    default:
      return CATS_FILTER_ERROR;
  }
}

cats_status_e check_baros_1_fault(state_estimation_data_t *data,
                                  sensor_elimination_t *elimination) {
  /* Check bounds */
  for (int i = 0; i < 3; i++) {
    if ((data->pressure[i] > UPPER_BOUND_PRESSURE) ||
        (data->pressure[i] < LOWER_BOUND_PRESSURE) ||
        (data->temperature[i] > UPPER_BOUND_TEMPERATURE) ||
        (data->temperature[i] < LOWER_BOUND_TEMPERATURE)) {
      elimination->faulty_baro[i] = 1;
    }
  }

  /* Check freezing of Sensor */
  for (int i = 0; i < 3; i++) {
    /* Pressure */
    if (data->pressure[i] == elimination->last_value[i + 3]) {
      elimination->num_freeze[i + 3] += 1;
      if (elimination->num_freeze[i + 3] > MAX_NUM_SAME_VALUE_PRESSURE) {
        elimination->faulty_baro[i] = 1;
      }
    } else {
      elimination->last_value[i + 3] = data->pressure[i];
      elimination->num_freeze[i + 3] = 0;
    }
    /* Temperature */
    if (data->temperature[i] == elimination->last_value[i + 6]) {
      elimination->num_freeze[i + 6] += 1;
      if (elimination->num_freeze[i + 6] > MAX_NUM_SAME_VALUE_TEMPERATURE) {
        elimination->faulty_baro[i] = 1;
      }
    } else {
      elimination->last_value[i + 6] = data->temperature[i];
      elimination->num_freeze[i + 6] = 0;
    }
  }

  /* Add Up all the faulty Baros */
  uint8_t counter = 0;
  for (int i = 0; i < 3; i++) {
    if (elimination->faulty_baro[i] == 1) {
      counter += 1;
    }
  }
  elimination->num_faulty_baros = counter;

  switch (elimination->num_faulty_baros) {
    case 0:
      return CATS_OK;
    case 1:
      return CATS_BARO_ERROR;
    case 2:
      return CATS_FILTER_ERROR;
    default:
      return CATS_FILTER_ERROR;
  }
}
