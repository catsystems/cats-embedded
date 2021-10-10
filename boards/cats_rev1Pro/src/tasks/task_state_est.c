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

#include "tasks/task_state_est.h"
#include "control/kalman_filter.h"
#include "control/orientation_filter.h"
#include "control/sensor_elimination.h"
#include "control/calibration.h"
#include "control/data_processing.h"

#include <math.h>

/** Private Constants **/

static const float P_INITIAL = 101250.f;
static const float GRAVITY = 9.81f;

/** Private Function Declarations **/

inline static float calculate_height(float pressure_initial, float pressure, float temperature);

static void transform_data(state_estimation_data_t *state_data, kalman_filter_t *filter,
                           calibration_data_t *calibration, flight_fsm_e *fsm_state);

static void average_data(imu_data_t *rolling_imu, uint8_t *imu_counter, int32_t *rolling_pressure,
                         uint8_t *pressure_counter, sensor_elimination_t *elimination, imu_data_t *average_imu,
                         float *average_pressure);

#ifdef USE_MEDIAN_FILTER
static void median_filter(median_filter_t *filter_data, state_estimation_data_t *state_data);
#endif

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
_Noreturn void task_state_est(__attribute__((unused)) void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;
  /* old flight phase */
  flight_fsm_e new_fsm_enum = MOVING;
  flight_fsm_e old_fsm_enum = MOVING;
  /* end old flight phase */

  /* calibration data */
  calibration_data_t calibration = {.angle = 1, .axis = 2};
  uint8_t imu_counter = 0;
  imu_data_t rolling_imu[10];
  imu_data_t average_imu = {};
  uint8_t pressure_counter = 0;
  int32_t rolling_pressure[10];
  float average_pressure = P_INITIAL;
  /* end calibration data */

  /* End Initialization */
  osDelay(1000);

  /* Initialize State Estimation */
  state_estimation_data_t state_data = {};
  sensor_elimination_t elimination = {};
  kalman_filter_t filter = {.pressure_0 = P_INITIAL, .t_sampl = 1.0f / (float)(CONTROL_SAMPLING_FREQ)};
  transform_data(&state_data, &filter, &calibration, &global_flight_state.flight_state);
  filter.pressure_0 = (state_data.pressure[0] + state_data.pressure[1] + state_data.pressure[2]) / 3;

  init_filter_struct(&filter);
  initialize_matrices(&filter);
  /* For Logging */
  float raw_accel;
  float raw_altitude_AGL;

  /* initialize Orientation State Estimation */
#ifdef USE_ORIENTATION_KF
  orientation_kf_t orientation_filter = {.t_sampl = 1.0f / (float)(CONTROL_SAMPLING_FREQ)};
  init_orientation_filter(&orientation_filter);
  reset_orientation_filter(&orientation_filter);
#endif

#ifdef USE_ORIENTATION_FILTER
  orientation_filter_t orientation_filter = {.t_sampl = 1.0f / (float)(CONTROL_SAMPLING_FREQ)};
  init_orientation_filter(&orientation_filter);
  reset_orientation_filter(&orientation_filter);
#endif

#ifdef USE_MEDIAN_FILTER
  /* Arrays for median Filtering */
  median_filter_t filter_data = {0};
#endif

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    new_fsm_enum = global_flight_state.flight_state;
    if (global_flight_state.flight_state == INVALID) {
      log_error("Invalid FSM state!");
    }

    /* Reset IMU when we go from moving to READY */
    if ((new_fsm_enum == READY) && (new_fsm_enum != old_fsm_enum)) {
      reset_kalman(&filter, average_pressure);
      calibrate_imu(&average_imu, &calibration);
    }
    /* Remove Accel Data when we enter apogee for the KF */
    if ((new_fsm_enum == APOGEE) && (new_fsm_enum != old_fsm_enum)) {
      float32_t Q_dash[4] = {0, 0, 0, 10.0f};
      memcpy(filter.Q_data, Q_dash, sizeof(Q_dash));
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    transform_data(&state_data, &filter, &calibration, &new_fsm_enum);
    raw_accel = 0;
    raw_altitude_AGL = 0;
      uint8_t num_faulty_imus = 0;
      if (elimination.faulty_accel[HIGH_G_ACC_INDEX] == 1) {
          num_faulty_imus = elimination.num_faulty_accel - 1;
      } else {
          num_faulty_imus = elimination.num_faulty_accel;
      }
      for (uint8_t i = 0; i < NUM_IMU; i++) {
          if (elimination.faulty_accel[i] == 0) {
              raw_accel += state_data.acceleration[i] / (float)(NUM_IMU - num_faulty_imus);
          }
      }
      for (uint8_t i = 0; i < NUM_PRESSURE; i++) {
          if (elimination.faulty_baro[i] == 0) {
              raw_altitude_AGL += state_data.calculated_AGL[i] / (float)(NUM_PRESSURE - elimination.num_faulty_baros);
          }
      }

      /* Check Sensor Readings (The Sensor Readings are checked before the median Filter!)*/
      check_sensors(&state_data, &elimination);

    /* Filter Data */
#ifdef USE_MEDIAN_FILTER
    median_filter(&filter_data, &state_data);
    float filtered_acc = 0;
    float filtered_AGL = 0;

    for (uint8_t i = 0; i < NUM_IMU; i++) {
      if (elimination.faulty_accel[i] == 0) {
        filtered_acc += state_data.acceleration[i] / (float)(NUM_IMU - num_faulty_imus);
      }
    }
    for (uint8_t i = 0; i < NUM_PRESSURE; i++) {
      if (elimination.faulty_baro[i] == 0) {
        filtered_AGL += state_data.calculated_AGL[i] / (float)(NUM_PRESSURE - elimination.num_faulty_baros);
      }
    }
    filtered_data_info_t filtered_data_info = {.ts = osKernelGetTickCount(),
                                               .measured_altitude_AGL = raw_altitude_AGL,
                                               .measured_acceleration = raw_accel,
                                               .filtered_acceleration = filtered_acc,
                                               .filtered_altitude_AGL = filtered_AGL};
    record(FILTERED_DATA_INFO, &filtered_data_info);
#endif



    /* Write the elimination Data into the global variable */
    global_elimination_data = elimination;

    /* Do the preprocessing on the IMU and BARO for calibration */
    /* Only do if we are in MOVING */
    if (new_fsm_enum == MOVING) {
      average_data(rolling_imu, &imu_counter, rolling_pressure, &pressure_counter, &elimination, &average_imu,
                   &average_pressure);
    }

    /* Do a Kalman Step */
    kalman_step(&filter, &state_data, &elimination, new_fsm_enum);

    /* write the Data into the global variable */
    global_kf_data.height = (float)filter.x_bar.pData[0];
    global_kf_data.velocity = (float)filter.x_bar.pData[1];
    global_kf_data.acceleration = state_data.acceleration[1];

    uint32_t ts = osKernelGetTickCount();

    /* Do Orientation Kalman */
#if defined(USE_ORIENTATION_KF) || defined(USE_ORIENTATION_FILTER)
    read_sensor_data(&global_magneto, &global_imu[0], &orientation_filter);
    orientation_filter_step(&orientation_filter);
    orientation_info_t orientation_info;
    orientation_info.ts = ts;
#endif
#ifdef USE_ORIENTATION_KF
    /* DO ORIENTATION KALMAN */

    log_trace("KF: q0: %ld; q1: %ld; q2: %ld; q3: %ld; |RAW|: q0: %ld; q1: %ld; q2: %ld; q3: %ld;",
              (int32_t)(orientation_filter.x_bar_data[0] * 1000), (int32_t)(orientation_filter.x_bar_data[1] * 1000),
              (int32_t)(orientation_filter.x_bar_data[2] * 1000), (int32_t)(orientation_filter.x_bar_data[3] * 1000),
              (int32_t)(orientation_filter.raw_computed_orientation[0] * 1000),
              (int32_t)(orientation_filter.raw_computed_orientation[1] * 1000),
              (int32_t)(orientation_filter.raw_computed_orientation[2] * 1000),
              (int32_t)(orientation_filter.raw_computed_orientation[3] * 1000));

    for (uint8_t i = 0; i < 4; i++) {
      orientation_info.raw_orientation[i] = (int16_t)(orientation_filter.raw_computed_orientation[i] * 10000.0f);
      orientation_info.estimated_orientation[i] = (int16_t)(orientation_filter.x_bar_data[i] * 10000.0f);
    }

    record(ORIENTATION_INFO, &orientation_info);
#endif
#ifdef USE_ORIENTATION_FILTER
    /* DO ORIENTATION Filter */
    /*
    log_trace("KF: q0: %ld; q1: %ld; q2: %ld; q3: %ld", (int32_t)(orientation_filter.estimate_data[0] * 1000),
              (int32_t)(orientation_filter.estimate_data[1] * 1000),
              (int32_t)(orientation_filter.estimate_data[2] * 1000),
              (int32_t)(orientation_filter.estimate_data[3] * 1000));
    */
    for (uint8_t i = 0; i < 4; i++) {
      orientation_info.raw_orientation[i] = (int16_t)(orientation_filter.estimate_data[i] * 10000.0f);
      orientation_info.estimated_orientation[i] = (int16_t)(orientation_filter.estimate_data[i] * 10000.0f);
    }

    record(ORIENTATION_INFO, &orientation_info);
#endif
    /* Log Covariance Data of KF */
    covariance_info_t cov_info = {.ts = ts, .height_cov = filter.P_bar.pData[1], .velocity_cov = filter.P_bar.pData[5]};
    record(COVARIANCE_INFO, &cov_info);

    /* Log KF outputs */
    flight_info_t flight_info = {.ts = ts,
                                 .height = filter.x_bar.pData[0],
                                 .velocity = filter.x_bar.pData[1],
                                 .acceleration = filtered_data_info.filtered_acceleration + filter.x_bar.pData[2]};
    if (new_fsm_enum >= APOGEE) {
      flight_info.height = filter.x_bar.pData[0];
      flight_info.velocity = filter.x_bar.pData[1];
      flight_info.acceleration = filter.x_bar.pData[2];
    }
    record(FLIGHT_INFO, &flight_info);
    /*
          log_info("P1: %ld; P2: %ld; P3: %ld; T1: %ld; T2: %ld; T3: %ld", (int32_t) ((float) state_data.pressure[0]),
                   (int32_t) ((float) state_data.pressure[1]),
                   (int32_t) ((float) state_data.pressure[2]),
                   (int32_t) ((float) state_data.temperature[0] * 100),
                   (int32_t) ((float) state_data.temperature[1] * 100),
                   (int32_t) ((float) state_data.temperature[2] * 100));
                   */
    /*
    log_info("H: %ld; V: %ld; A: %ld; O: %ld", (int32_t)((float)filter.x_bar.pData[0] * 1000),
             (int32_t)((float)filter.x_bar.pData[1] * 1000), (int32_t)(filtered_data_info.filtered_acceleration * 1000),
             (int32_t)((float)filter.x_bar.pData[2] * 1000));
             */

    //            log_trace("Calibrated IMU 1: Z: %ld",
    //            (int32_t)(1000*state_data.acceleration[0]));
    //            log_trace("Calibrated IMU 2: Z: %ld",
    //            (int32_t)(1000*state_data.acceleration[1]));
    //            log_trace("Calibrated IMU 3: Z: %ld",
    //            (int32_t)(1000*state_data.acceleration[2]));
    /* END DEBUGGING */

    old_fsm_enum = new_fsm_enum;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

inline static float calculate_height(float pressure_initial, float pressure, float temperature) {
  return ((powf(pressure_initial / pressure, (1 / 5.257f)) - 1) * (temperature + 273.15f) / 0.0065f);
}

static void transform_data(state_estimation_data_t *state_data, kalman_filter_t *filter,
                           calibration_data_t *calibration, flight_fsm_e *fsm_state) {
  /* Get Data from the Sensors */
  /* Use calibration step to get the correct acceleration */
  /* Todo: Correct conversion for the accelerometer Data */
  switch (calibration->axis) {
    case 0:
      /* Choose X Axis */
      /* Fill up state data with IMU and once this is done, continue filling up with accel data */
      for (uint8_t i = 0; i < NUM_ACC; i++) {
        if (i == HIGH_G_ACC_INDEX) {
          state_data->acceleration[i] = (float)(global_accel.acc_x) * (7.6640625f) / calibration->angle - GRAVITY;
        } else {
          state_data->acceleration[i] = (float)(global_imu[i].acc_x) / (1024) * GRAVITY / calibration->angle - GRAVITY;
        }
      }
      break;
    case 1:
      /* Choose Y Axis */
      /* Fill up state data with IMU and once this is done, continue filling up with accel data */
      for (uint8_t i = 0; i < NUM_ACC; i++) {
        if (i == HIGH_G_ACC_INDEX) {
          state_data->acceleration[i] = (float)(global_accel.acc_y) * (7.6640625f) / calibration->angle - GRAVITY;
        } else {
          state_data->acceleration[i] = (float)(global_imu[i].acc_y) / (1024) * GRAVITY / calibration->angle - GRAVITY;
        }
      }
      break;
    case 2:
      /* Choose Z Axis */
      /* Fill up state data with IMU and once this is done, continue filling up with accel data */
      for (uint8_t i = 0; i < NUM_ACC; i++) {
        if (i == HIGH_G_ACC_INDEX) {
          state_data->acceleration[i] = (float)(global_accel.acc_z) * (7.6640625f) / calibration->angle - GRAVITY;
        } else {
          state_data->acceleration[i] = (float)(global_imu[i].acc_z) / (1024) * GRAVITY / calibration->angle - GRAVITY;
        }
      }
      break;
    default:
      break;
  }

  for (uint8_t i = 0; i < NUM_PRESSURE; i++) {
    state_data->pressure[i] = (float)(global_baro[i].pressure);
  }
  for (uint8_t i = 0; i < NUM_TEMPERATURE; i++) {
    state_data->temperature[i] = (float)global_baro[i].temperature / 100.f;
  }

  /* Add Sensor Noise if asked to */
  // if (*fsm_state == THRUSTING_1) {
#ifdef INCLUDE_NOISE
  float rand_pressure[3] = {0};
  float rand_acc[3] = {0};
  rand_pressure[0] = PRESSURE_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);
  rand_pressure[1] = PRESSURE_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);
  rand_pressure[2] = PRESSURE_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);
  rand_acc[0] = ACC_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);
  rand_acc[1] = ACC_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);
  rand_acc[2] = ACC_NOISE_MAX_AMPL * ((float)rand() - 2147483648 / 2) / (2147483648 / 2);

  state_data->pressure[0] += rand_pressure[0];
  state_data->pressure[1] += rand_pressure[1];
  state_data->pressure[2] += rand_pressure[2];
  state_data->acceleration[0] += rand_acc[0];
  state_data->acceleration[1] += rand_acc[1];
  state_data->acceleration[2] += rand_acc[2];
#endif
  /* Add Spikes in the Data if asked to */
#ifdef INCLUDE_SPIKES
  float spike = (float)rand() / 2147483648;
  if (spike < SPIKE_THRESHOLD) {
#ifdef SPIKE_BARO
    state_data->pressure[SPIKE_SENSOR_CHOICE] += 10000000;
#endif
#ifdef SPIKE_IMU
    state_data->acceleration[SPIKE_SENSOR_CHOICE] += 10000000;
#endif
  }
#endif
  /* Add Offset to one Sensor if asked to */
#ifdef INCLUDE_OFFSET
#ifdef OFFSET_BARO
  state_data->pressure[OFFSET_SENSOR_CHOICE] += OFFSET_P;
#endif
#ifdef OFFSET_IMU
  state_data->acceleration[OFFSET_SENSOR_CHOICE] += OFFSET_ACC;
#endif
#endif
  //}
  for (uint8_t i = 0; i < NUM_PRESSURE; i++) {
    state_data->calculated_AGL[i] =
        calculate_height(filter->pressure_0, state_data->pressure[i], state_data->temperature[i]);
  }
}

static void average_data(imu_data_t *rolling_imu, uint8_t *imu_counter, int32_t *rolling_pressure,
                         uint8_t *pressure_counter, sensor_elimination_t *elimination, imu_data_t *average_imu,
                         float *average_pressure) {
  imu_data_t average_imu_from_global = {0};
  /* First average the 3 IMU measurements if no IMUs have been eliminated */
  average_imu_from_global.acc_x = 0;
  average_imu_from_global.acc_y = 0;
  average_imu_from_global.acc_z = 0;
  /* compute number of faulty IMU's because we do not want to use the accelerometer in this calculation */
  uint8_t num_faulty_imus;

  if (elimination->faulty_accel[HIGH_G_ACC_INDEX] == 1) {
    num_faulty_imus = elimination->num_faulty_accel - 1;
  } else {
    num_faulty_imus = elimination->num_faulty_accel;
  }

  /* If all accels are eliminated output Hard Fault */
  /* Todo: If we are already flying this not a hard fault! It is only a hard fault if we are in moving */
  if (elimination->num_faulty_accel == NUM_ACC) {
    add_error(CATS_ERR_FILTER);
  }
  /* If both IMU's were filtered out use accelerometer */
  /* Todo: This assumes that the index of the accel is larger than the index of the IMU's */
  if ((elimination->num_faulty_accel == NUM_IMU) && (elimination->faulty_accel[HIGH_G_ACC_INDEX] == 0)) {
    average_imu_from_global.acc_x += global_accel.acc_x;
    average_imu_from_global.acc_y += global_accel.acc_y;
    average_imu_from_global.acc_z += global_accel.acc_z;
  } else {
    /* Otherwise use non eliminated IMU's */
    for (int i = 0; i < NUM_IMU; i++) {
      if (elimination->faulty_accel[i] == 0) {
        average_imu_from_global.acc_x += global_imu[i].acc_x / (NUM_IMU - num_faulty_imus);
        average_imu_from_global.acc_y += global_imu[i].acc_y / (NUM_IMU - num_faulty_imus);
        average_imu_from_global.acc_z += global_imu[i].acc_z / (NUM_IMU - num_faulty_imus);
      }
    }
  }

  /* Write this into the rolling IMU array */
  rolling_imu[*imu_counter] = average_imu_from_global;

  /* Average the rolling IMU Array */
  average_imu->acc_x = 0;
  average_imu->acc_y = 0;
  average_imu->acc_z = 0;
  for (int i = 0; i < 10; i++) {
    average_imu->acc_x += rolling_imu[i].acc_x;
    average_imu->acc_y += rolling_imu[i].acc_y;
    average_imu->acc_z += rolling_imu[i].acc_z;
  }
  average_imu->acc_x /= 10;
  average_imu->acc_y /= 10;
  average_imu->acc_z /= 10;

  /* Increase the counter for the rolling IMU array */
  (*imu_counter)++;
  if ((*imu_counter) > 9) {
    (*imu_counter) = 0;
  }

  /* Do the Baro */
  /* First average the 3 Baro measurements if no Baros have been eliminated
   */
  int32_t global_average_pressure = 0;
  /* If all Baros are eliminated this is a Filter error, always!*/
  if (elimination->num_faulty_baros == NUM_PRESSURE) {
    add_error(CATS_ERR_FILTER);
  }
  for (int i = 0; i < NUM_PRESSURE; i++) {
    if (elimination->faulty_baro[i] == 0)
      global_average_pressure += global_baro[i].pressure / (NUM_PRESSURE - elimination->num_faulty_baros);
  }

  /* Write this into the rolling Baro array */
  rolling_pressure[*pressure_counter] = global_average_pressure;

  /* Average the rolling IMU Array */
  *average_pressure = 0;
  for (int i = 0; i < 10; i++) {
    *average_pressure += (float)rolling_pressure[i];
  }
  *average_pressure /= 10.0f;

  /* Increase the counter for the rolling Baro array */
  (*pressure_counter)++;
  if ((*pressure_counter) > 9) {
    (*pressure_counter) = 0;
  }
}

#ifdef USE_MEDIAN_FILTER

static void median_filter(median_filter_t *filter_data, state_estimation_data_t *state_data) {
  filter_data->acc_data[0][filter_data->counter] = state_data->acceleration[0];
  filter_data->acc_data[1][filter_data->counter] = state_data->acceleration[1];
  filter_data->acc_data[2][filter_data->counter] = state_data->acceleration[2];
  filter_data->height_AGL_data[0][filter_data->counter] = state_data->calculated_AGL[0];
  filter_data->height_AGL_data[1][filter_data->counter] = state_data->calculated_AGL[1];
  filter_data->height_AGL_data[2][filter_data->counter] = state_data->calculated_AGL[2];
  filter_data->counter++;
  filter_data->counter = filter_data->counter % MEDIAN_FILTER_SIZE;

  state_data->acceleration[0] = median(*(filter_data->acc_data + 0), MEDIAN_FILTER_SIZE);

  state_data->acceleration[1] = median(*(filter_data->acc_data + 1), MEDIAN_FILTER_SIZE);

  state_data->acceleration[2] = median(*(filter_data->acc_data + 2), MEDIAN_FILTER_SIZE);

  state_data->calculated_AGL[0] = median(*(filter_data->height_AGL_data + 0), MEDIAN_FILTER_SIZE);

  state_data->calculated_AGL[1] = median(*(filter_data->height_AGL_data + 1), MEDIAN_FILTER_SIZE);

  state_data->calculated_AGL[2] = median(*(filter_data->height_AGL_data + 2), MEDIAN_FILTER_SIZE);
}

#endif
