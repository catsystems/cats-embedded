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

#include "control/sensor_elimination.h"
#include "util/log.h"
#include <math.h>
#include "config/control_config.h"

static cats_error_e check_sensor_bounds(state_estimation_data_t *data, sensor_elimination_t *elimination, uint8_t index,
                                        bool is_pressure);
static cats_error_e check_sensor_freezing(state_estimation_data_t *data, sensor_elimination_t *elimination,
                                          uint8_t index, bool is_pressure);
static cats_error_e check_sensor_majority(state_estimation_data_t *data, sensor_elimination_t *elimination,
                                          bool is_pressure);
static cats_error_e get_error_code(uint8_t index, bool is_pressure);

cats_error_e check_sensors(state_estimation_data_t *data, sensor_elimination_t *elimination) {
  cats_error_e status = CATS_ERR_OK;
  sensor_elimination_t old_elimination = *elimination;

  /* Accelerometers */

  for (uint8_t i = 0; i < NUM_ACC; i++) {
    status |= check_sensor_bounds(data, elimination, i, false);
    // Only check freezing on non high G accels
    if(i != 2) status |= check_sensor_freezing(data, elimination, i, false);
    /* Accel is not faulty anymore */
    if (elimination->faulty_accel[i] == 1) {
      if (status == CATS_ERR_OK) {
        elimination->faulty_accel[i] = 0;
      }
    }
    status = CATS_ERR_OK;
  }

  /* Add Up all the faulty Accels */
  elimination->num_faulty_accel = 0;
  for (uint8_t i = 0; i < NUM_ACC; i++) {
    if (elimination->faulty_accel[i] == 1) {
      elimination->num_faulty_accel += 1;
    }
  }

  /* If we have 3 Working Accels, check Majority voting */
  if (elimination->num_faulty_accel == 0) {
    status |= check_sensor_majority(data, elimination, false);
    elimination->num_faulty_accel = 0;
    for (uint8_t i = 0; i < NUM_ACC; i++) {
      if (elimination->faulty_accel[i] == 1) {
        elimination->num_faulty_accel += 1;
      }
    }
    status = CATS_ERR_OK;
  }

  /* Barometers */
  for (uint8_t i = 0; i < NUM_BARO; i++) {
    status |= check_sensor_bounds(data, elimination, i, true);
    status |= check_sensor_freezing(data, elimination, i, true);
    /* Baro is not faulty anymore */
    if (elimination->faulty_baro[i] == 1) {
      if (status == CATS_ERR_OK) {
        elimination->faulty_baro[i] = 0;
      }
    }
    status = CATS_ERR_OK;
  }

  /* Add Up all the faulty Baros */
  elimination->num_faulty_baros = 0;
  for (uint8_t i = 0; i < NUM_BARO; i++) {
    if (elimination->faulty_baro[i] == 1) {
      elimination->num_faulty_baros += 1;
    }
  }

  /* If we have 3 Working Baros, check Majority voting */
  if (elimination->num_faulty_baros == 0) {
    status |= check_sensor_majority(data, elimination, true);
    elimination->num_faulty_baros = 0;
    for (uint8_t i = 0; i < NUM_BARO; i++) {
      if (elimination->faulty_baro[i] == 1) {
        elimination->num_faulty_baros += 1;
      }
    }
    status = CATS_ERR_OK;
  }

  /* Error Logging */
  cats_error_e log_status = CATS_ERR_OK;
  for (uint8_t i = 0; i < NUM_BARO; i++) {
    if (elimination->faulty_baro[i]) {
      if (elimination->faulty_baro[i] != old_elimination.faulty_baro[i]) {
        switch (i) {
          case 0:
            log_status |= CATS_ERR_BARO_0;
            break;
          case 1:
            log_status |= CATS_ERR_BARO_1;
            break;
          case 2:
            log_status |= CATS_ERR_BARO_2;
            break;
          default:
            break;
        }
      }
    }
  }
  for (uint8_t i = 0; i < NUM_ACC; i++) {
    if (elimination->faulty_accel[i]) {
      if (elimination->faulty_accel[i] != old_elimination.faulty_accel[i]) {
        switch (i) {
          case 0:
            log_status |= CATS_ERR_IMU_0;
            break;
          case 1:
            log_status |= CATS_ERR_IMU_1;
            break;
          case 2:
            log_status |= CATS_ERR_IMU_2;
            break;
          default:
            break;
        }
      }
    }
  }
  add_error(log_status);

  return status;
}

static cats_error_e check_sensor_bounds(state_estimation_data_t *data, sensor_elimination_t *elimination, uint8_t index,
                                        bool is_pressure) {
  cats_error_e status = CATS_ERR_OK;
  cats_error_e error_status = get_error_code(index, is_pressure);

  if (is_pressure) {
    /* Check Pressure Data */
    if ((data->pressure[index] > UPPER_BOUND_PRESSURE) || (data->pressure[index] < LOWER_BOUND_PRESSURE) ||
        (data->temperature[index] > UPPER_BOUND_TEMPERATURE) || (data->temperature[index] < LOWER_BOUND_TEMPERATURE)) {
      status = error_status;
      elimination->faulty_baro[index] = 1;
    }
  } else {
    /* Check Accelerometer Data */
    if (index == HIGH_G_ACC_INDEX) {
      /* check Bound of High G accel */
      if ((data->acceleration[index] > UPPER_BOUND_HIGH_G_ACC) ||
          (data->acceleration[index] < LOWER_BOUND_HIGH_G_ACC)) {
        elimination->faulty_accel[index] = 1;
        status = error_status;
      }
      /* check if we are in high acceleration mode */
      else if ((data->acceleration[index] > UPPER_BOUND_ACC) && (data->acceleration[index] < LOWER_BOUND_HIGH_G_ACC)) {
        elimination->high_acc = true;
      } else {
        elimination->high_acc = false;
      }
    } else {
      if ((data->acceleration[index] > UPPER_BOUND_ACC) || (data->acceleration[index] < LOWER_BOUND_ACC)) {
        status = error_status;
        elimination->faulty_accel[index] = 1;
      }
    }
  }

  return status;
}

static cats_error_e check_sensor_freezing(state_estimation_data_t *data, sensor_elimination_t *elimination,
                                          uint8_t index, bool is_pressure) {
  cats_error_e status = CATS_ERR_OK;
  cats_error_e error_status = get_error_code(index, is_pressure);

  if (is_pressure) {
    /* Pressure */
    if (data->pressure[index] == elimination->last_value[index + 3]) {
      elimination->num_freeze[index + 3] += 1;
      if (elimination->num_freeze[index + 3] > MAX_NUM_SAME_VALUE_PRESSURE) {
        status = error_status;
        elimination->faulty_baro[index] = 1;
      }
    } else {
      elimination->last_value[index + 3] = data->pressure[index];
      elimination->num_freeze[index + 3] = 0;
    }
    /* Temperature */
    if (data->temperature[index] == elimination->last_value[index + 6]) {
      elimination->num_freeze[index + 6] += 1;
      if (elimination->num_freeze[index + 6] > MAX_NUM_SAME_VALUE_TEMPERATURE) {
        status = error_status;
        elimination->faulty_baro[index] = 1;
      }
    } else {
      elimination->last_value[index + 6] = data->temperature[index];
      elimination->num_freeze[index + 6] = 0;
    }
  } else {
    /* Check imu */
    if (data->acceleration[index] == elimination->last_value[index]) {
      elimination->num_freeze[index] += 1;
      if (elimination->num_freeze[index] > MAX_NUM_SAME_VALUE_IMU) {
        status = error_status;
        elimination->faulty_accel[index] = 1;
      }
    } else {
      elimination->last_value[index] = data->acceleration[index];
      elimination->num_freeze[index] = 0;
    }
  }

  return status;
}

static cats_error_e check_sensor_majority(state_estimation_data_t *data, sensor_elimination_t *elimination,
                                          bool is_pressure) {
  cats_error_e status = CATS_ERR_OK;

  /* Do Majority Voting */
  float error[3] = {0};

  if (is_pressure) {
    /* Barometer */
    error[0] = fabsf(data->pressure[0] - data->pressure[1]);
    error[1] = fabsf(data->pressure[1] - data->pressure[2]);
    error[2] = fabsf(data->pressure[0] - data->pressure[2]);
    if ((error[0] > MAJ_VOTE_PRESSURE_ERROR) && (error[2] > MAJ_VOTE_PRESSURE_ERROR)) {
      /* Baro 0 probably faulty */
      elimination->num_maj_vote[3] += 1;
      if (elimination->num_maj_vote[3] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_0;
        elimination->faulty_baro[0] = 1;
      }
    } else {
      elimination->num_maj_vote[3] = 0;
    }
    if ((error[0] > MAJ_VOTE_PRESSURE_ERROR) && (error[1] > MAJ_VOTE_PRESSURE_ERROR)) {
      /* Baro 1 probably faulty */
      elimination->num_maj_vote[4] += 1;
      if (elimination->num_maj_vote[4] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_1;
        elimination->faulty_baro[1] = 1;
      }
    } else {
      elimination->num_maj_vote[4] = 0;
    }
    if ((error[1] > MAJ_VOTE_PRESSURE_ERROR) && (error[2] > MAJ_VOTE_PRESSURE_ERROR)) {
      /* Baro 2 probably faulty */
      elimination->num_maj_vote[5] += 1;
      if (elimination->num_maj_vote[5] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_2;
        elimination->faulty_baro[2] = 1;
      }
    } else {
      elimination->num_maj_vote[5] = 0;
    }

    /* Temperature */
    error[0] = fabsf(data->temperature[0] - data->temperature[1]);
    error[1] = fabsf(data->temperature[1] - data->temperature[2]);
    error[2] = fabsf(data->temperature[0] - data->temperature[2]);
    if ((error[0] > MAJ_VOTE_TEMPERATURE_ERROR) && (error[2] > MAJ_VOTE_TEMPERATURE_ERROR)) {
      /* Baro 0 probably faulty */
      elimination->num_maj_vote[6] += 1;
      if (elimination->num_maj_vote[6] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_0;
        elimination->faulty_baro[0] = 1;
      }
    } else {
      elimination->num_maj_vote[6] = 0;
    }
    if ((error[0] > MAJ_VOTE_TEMPERATURE_ERROR) && (error[1] > MAJ_VOTE_TEMPERATURE_ERROR)) {
      /* Baro 1 probably faulty */
      elimination->num_maj_vote[7] += 1;
      if (elimination->num_maj_vote[7] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_1;
        elimination->faulty_baro[1] = 1;
      }
    } else {
      elimination->num_maj_vote[7] = 0;
    }
    if ((error[1] > MAJ_VOTE_TEMPERATURE_ERROR) && (error[2] > MAJ_VOTE_TEMPERATURE_ERROR)) {
      /* Baro 2 probably faulty */
      elimination->num_maj_vote[8] += 1;
      if (elimination->num_maj_vote[8] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_BARO_2;
        elimination->faulty_baro[2] = 1;
      }
    } else {
      elimination->num_maj_vote[8] = 0;
    }
  } else {
    /* Acceleration */
    error[0] = fabsf(data->acceleration[0] - data->acceleration[1]);
    error[1] = fabsf(data->acceleration[1] - data->acceleration[2]);
    error[2] = fabsf(data->acceleration[0] - data->acceleration[2]);
    if ((error[0] > MAJ_VOTE_IMU_ERROR) && (error[2] > MAJ_VOTE_IMU_ERROR)) {
      /* IMU 0 probably faulty */
      elimination->num_maj_vote[0] += 1;
      if (elimination->num_maj_vote[0] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_IMU_0;
        elimination->faulty_accel[0] = 1;
      }
    } else {
      elimination->num_maj_vote[0] = 0;
    }
    if ((error[0] > MAJ_VOTE_IMU_ERROR) && (error[1] > MAJ_VOTE_IMU_ERROR)) {
      /* IMU 1 probably faulty */
      elimination->num_maj_vote[1] += 1;
      if (elimination->num_maj_vote[1] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_IMU_1;
        elimination->faulty_accel[1] = 1;
      }
    } else {
      elimination->num_maj_vote[1] = 0;
    }
    if ((error[1] > MAJ_VOTE_IMU_ERROR) && (error[2] > MAJ_VOTE_IMU_ERROR)) {
      /* IMU 2 probably faulty */
      elimination->num_maj_vote[2] += 1;
      if (elimination->num_maj_vote[2] > MAJ_VOTE_NUM_VALUES) {
        status = CATS_ERR_IMU_2;
        elimination->faulty_accel[2] = 1;
      }
    } else {
      elimination->num_maj_vote[2] = 0;
    }
  }

  return status;
}

static cats_error_e get_error_code(uint8_t index, bool is_pressure) {
  switch (index) {
    case 0:
      if (is_pressure) {
        return CATS_ERR_BARO_0;
      } else {
        return CATS_ERR_IMU_0;
      }
      break;
    case 1:
      if (is_pressure) {
        return CATS_ERR_BARO_1;
      } else {
        return CATS_ERR_IMU_1;
      }
      break;
    case 2:
      if (is_pressure) {
        return CATS_ERR_BARO_2;
      } else {
        return CATS_ERR_IMU_2;
      }
      break;
    default:
      return CATS_ERR_HARD_FAULT;
      break;
  }
}
