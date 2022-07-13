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

#include "tasks/task_preprocessing.h"
#include "config/globals.h"
#include "config/sensor_config.h"
#include "control/calibration.h"
#include "control/data_processing.h"
#include "control/sensor_elimination.h"
#include "util/log.h"

/** Private Constants **/

/** Private Function Declarations **/
static void avg_and_to_SI(SI_data_t *SI_data, SI_data_t *SI_data_old, const sensor_elimination_t *elimination_data);
static void median_filter(median_filter_t *filter_data, state_estimation_input_t *state_data);
static void transform_data(float32_t pressure_0, state_estimation_input_t *state_data, const SI_data_t *SI_data,
                           const calibration_data_t *calibration);
inline static float calculate_height(float pressure_initial, float pressure);
/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_preprocessing thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_preprocessing(void *argument) {
  /* Create data structs */
  static SI_data_t SI_data = {0};
  static SI_data_t SI_data_old = {0};
  SI_data_old.acc.x = 9.81f;
  SI_data_old.acc.y = 0.0f;
  SI_data_old.acc.z = 0.0f;
  SI_data_old.pressure = 98000.0f;
#ifdef USE_MEDIAN_FILTER
  median_filter_t filter_data = {0};
#endif
  sensor_elimination_t sensor_elimination = {0};
  /* Calibration Data including the gyro calibration as the first three values and then the angle and axis are for
   * the linear acceleration calibration */
  calibration_data_t calibration = {.gyro_calib = {.x = 0, .y = 0, .z = 0}, .angle = 1, .axis = 2};
  state_estimation_input_t state_est_input = {.acceleration_z = 0.0f, .height_AGL = 0.0f};
  float32_t pressure_0 = P_INITIAL;

  /* Gyro Calib tag */
  bool gyro_calibrated = false;

  /* local fsm enum */
  flight_fsm_e new_fsm_enum = MOVING;
  flight_fsm_e old_fsm_enum = MOVING;

  /* Infinite loop */
  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (1) {
    /* update fsm enum */
    new_fsm_enum = global_flight_state.flight_state;

    /* Do the sensor elimination */
    check_sensors(&sensor_elimination);

    /* average and construct SI Data */
    avg_and_to_SI(&SI_data, &SI_data_old, &sensor_elimination);

    // log_info("acc: %ld; height: %ld", (int32_t)((float)SI_data.acc.x*1000),
    //(int32_t)((float)SI_data.pressure*1000));

    /* Compute gravity when changing to IDLE */
    if ((new_fsm_enum == READY) && (new_fsm_enum != old_fsm_enum)) {
      calibrate_imu(&SI_data.acc, &calibration);
      pressure_0 = SI_data.pressure;
    }

    if (!gyro_calibrated) {
      gyro_calibrated = compute_gyro_calibration(&SI_data.gyro, &calibration);
    } else {
      calibrate_gyro(&calibration, &SI_data.gyro);
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    transform_data(pressure_0, &state_est_input, &SI_data, &calibration);

#ifdef USE_MEDIAN_FILTER
    /* Filter the data */
    median_filter(&filter_data, &state_est_input);
#endif

    /* reset old fsm enum */
    old_fsm_enum = new_fsm_enum;

    memcpy(&SI_data_old, &SI_data, sizeof(SI_data));

    /* write input data into global struct */
    global_estimation_input = state_est_input;

    /* Global SI data is only used in the fsm task */
    global_SI_data = SI_data;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

static void avg_and_to_SI(SI_data_t *SI_data, SI_data_t *SI_data_old, const sensor_elimination_t *elimination_data) {
  float32_t counter = 0;
#if NUM_IMU > 0
  /* Reset SI data */
  SI_data->acc.x = 0;
  SI_data->acc.y = 0;
  SI_data->acc.z = 0;
  SI_data->gyro.x = 0;
  SI_data->gyro.y = 0;
  SI_data->gyro.z = 0;

  /* Sum up all non-eliminated IMUs and transform to SI */
  for (int i = 0; i < NUM_IMU; i++) {
    if (elimination_data->faulty_imu[i] == 0) {
      counter++;
      SI_data->acc.x += (float32_t)global_imu[i].acc.x * acc_info[i].conversion_to_SI;
      SI_data->acc.y += (float32_t)global_imu[i].acc.y * acc_info[i].conversion_to_SI;
      SI_data->acc.z += (float32_t)global_imu[i].acc.z * acc_info[i].conversion_to_SI;
      SI_data->gyro.x += (float32_t)global_imu[i].gyro.x * gyro_info[i].conversion_to_SI;
      SI_data->gyro.y += (float32_t)global_imu[i].gyro.y * gyro_info[i].conversion_to_SI;
      SI_data->gyro.z += (float32_t)global_imu[i].gyro.z * gyro_info[i].conversion_to_SI;
    }
  }

#if NUM_ACCELEROMETER > 0
  /* If all IMUs have been eliminated use high G accel */
  if (counter == 0) {
    for (int i = 0; i < NUM_ACCELEROMETER; i++) {
      if (elimination_data->faulty_acc[i] == 0) {
        counter++;
        SI_data->acc.x += (float32_t)global_imu[i].acc.x * acc_info[NUM_IMU + i].conversion_to_SI;
        SI_data->acc.y += (float32_t)global_imu[i].acc.y * acc_info[NUM_IMU + i].conversion_to_SI;
        SI_data->acc.z += (float32_t)global_imu[i].acc.z * acc_info[NUM_IMU + i].conversion_to_SI;
      }
    }
  }
#endif

  /* average for SI data */
  if (counter > 0) {
    SI_data->acc.x /= counter;
    SI_data->acc.y /= counter;
    SI_data->acc.z /= counter;
    SI_data->gyro.x /= counter;
    SI_data->gyro.y /= counter;
    SI_data->gyro.z /= counter;
    clear_error(CATS_ERR_FILTER_ACC);
  } else {
    SI_data->acc = SI_data_old->acc;
    SI_data->gyro = SI_data_old->gyro;
    add_error(CATS_ERR_FILTER_ACC);
  }

#endif

#if NUM_BARO > 0
  counter = 0;
  SI_data->pressure = 0;
  for (int i = 0; i < NUM_BARO; i++) {
    if (elimination_data->faulty_baro[i] == 0) {
      counter++;
      SI_data->pressure += (float32_t)global_baro[i].pressure * baro_info[i].conversion_to_SI;
    }
  }
  if (counter > 0) {
    SI_data->pressure /= counter;
    clear_error(CATS_ERR_FILTER_HEIGHT);
  } else {
    SI_data->pressure = SI_data_old->pressure;
    add_error(CATS_ERR_FILTER_HEIGHT);
  }
#endif

#if NUM_MAGNETO > 0
  counter = 0;
  SI_data->mag.x = 0;
  SI_data->mag.y = 0;
  SI_data->mag.z = 0;
  for (int i = 0; i < NUM_MAGNETO; i++) {
    if (elimination_data->faulty_mag[i] == 0) {
      counter++;
      SI_data->mag.x += (float32_t)global_magneto[i].x * mag_info[i].conversion_to_SI;
      SI_data->mag.y += (float32_t)global_magneto[i].y * mag_info[i].conversion_to_SI;
      SI_data->mag.z += (float32_t)global_magneto[i].z * mag_info[i].conversion_to_SI;
    }
  }
  if (counter > 0) {
    SI_data->mag.x /= counter;
    SI_data->mag.y /= counter;
    SI_data->mag.z /= counter;
  } else {
    /* Todo: Add error */
    SI_data->mag = SI_data_old->mag;
  }
#endif
}

static void median_filter(median_filter_t *filter_data, state_estimation_input_t *state_data) {
  /* Insert into array */
  filter_data->acc[filter_data->counter] = state_data->acceleration_z;
  filter_data->height_AGL[filter_data->counter] = state_data->height_AGL;

  /* Update Counter */
  filter_data->counter++;
  filter_data->counter = filter_data->counter % MEDIAN_FILTER_SIZE;

  /* Filter data */
  state_data->acceleration_z = median(filter_data->acc);
  state_data->height_AGL = median(filter_data->height_AGL);
}

static void transform_data(float32_t pressure_0, state_estimation_input_t *state_data, const SI_data_t *SI_data,
                           const calibration_data_t *calibration) {
  /* Get Data from the Sensors */
  /* Use calibration step to get the correct acceleration */
  switch (calibration->axis) {
    case 0:
      /* Choose X Axis */
      state_data->acceleration_z = SI_data->acc.x / calibration->angle - GRAVITY;
      break;
    case 1:
      /* Choose Y Axis */
      state_data->acceleration_z = SI_data->acc.y / calibration->angle - GRAVITY;
      break;
    case 2:
      /* Choose Z Axis */
      state_data->acceleration_z = SI_data->acc.z / calibration->angle - GRAVITY;
      break;
    default:
      break;
  }
  state_data->height_AGL = calculate_height(pressure_0, SI_data->pressure);
}

inline static float calculate_height(float32_t pressure_initial, float32_t pressure) {
  return (-(powf(pressure / pressure_initial, (1 / 5.257f)) - 1) * (TEMPERATURE_0 + 273.15f) / 0.0065f);
}
