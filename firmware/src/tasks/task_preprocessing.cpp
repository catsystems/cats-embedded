/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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
#include "control/calibration.h"
#include "control/data_processing.h"
#include "control/sensor_elimination.h"
#include "target.h"
#include "tasks/task_sensor_read.h"
#include "util/log.h"
#include "util/task_util.h"

/** Private Constants **/

/** Private Function Declarations **/
static void avg_and_to_SI(SI_data_t *SI_data, SI_data_t *SI_data_old, const sensor_elimination_t *elimination_data);

static void median_filter(median_filter_t *filter_data, state_estimation_input_t *state_data);

static void transform_data(const SI_data_t *SI_data, const calibration_data_t *calibration, float32_t height_0,
                           state_estimation_input_t *state_data);

namespace task {

std::optional<Preprocessing> Preprocessing::s_instance = std::nullopt;

SET_TASK_PARAMS(task_preprocessing, 512);

void Preprocessing::Run() { osThreadNew(task_preprocessing, nullptr, &task_preprocessing_attributes); }

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_preprocessing thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_preprocessing(void *argument) {
  /* Get Tasks */
  auto &sensor_read_task = SensorRead::GetInstance();
  auto &preprocessing_task = Preprocessing::GetInstance();

  /* Initialise data structs */
  preprocessing_task.m_si_data_old.acc.x = GRAVITY;
  preprocessing_task.m_si_data_old.acc.y = 0.0f;
  preprocessing_task.m_si_data_old.acc.z = 0.0f;
  preprocessing_task.m_si_data_old.pressure = P_INITIAL;

  /* Infinite loop */
  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (1) {
    /* update fsm enum */
    preprocessing_task.new_fsm_enum = global_flight_state.flight_state;

    /* get new sensor data */
    preprocessing_task.m_baro_data[0] = sensor_read_task.GetBaro(0);
    preprocessing_task.m_imu_data[0] = sensor_read_task.GetImu(0);
    preprocessing_task.m_magneto_data[0] = sensor_read_task.GetMag(0);
    preprocessing_task.m_accel_data[0] = sensor_read_task.GetAccel(0);

    /* Do the sensor elimination */
    check_sensors(&preprocessing_task.sensor_elimination);

    /* average and construct SI Data */
    avg_and_to_SI(&preprocessing_task.m_si_data, &preprocessing_task.m_si_data_old,
                  &preprocessing_task.sensor_elimination);

    /* Compute gravity when changing to READY */
    if ((preprocessing_task.new_fsm_enum != preprocessing_task.old_fsm_enum) &&
        (preprocessing_task.new_fsm_enum == READY)) {
      calibrate_imu(&preprocessing_task.m_si_data.acc, &preprocessing_task.calibration);
      global_flight_stats.calibration_data.angle = preprocessing_task.calibration.angle;
      global_flight_stats.calibration_data.axis = preprocessing_task.calibration.axis;
    }

    /* calibrate gyro once at startup */
    if (!preprocessing_task.gyro_calibrated) {
      preprocessing_task.gyro_calibrated =
          compute_gyro_calibration(&preprocessing_task.m_si_data.gyro, &preprocessing_task.calibration);
    } else {
      calibrate_gyro(&preprocessing_task.calibration, &preprocessing_task.m_si_data.gyro);
      global_flight_stats.calibration_data.gyro_calib = preprocessing_task.calibration.gyro_calib;
    }

    /* Compute current height constantly before liftoff. If the state is moving, the filter is much faster. */
    if (preprocessing_task.new_fsm_enum == MOVING) {
      preprocessing_task.height_0 =
          approx_moving_average(calculate_height(preprocessing_task.m_si_data.pressure), true);
      global_flight_stats.height_0 = preprocessing_task.height_0;
    }
    /* Compute current height constantly before liftoff. If the state is ready, the filter is much slower. */
    if (preprocessing_task.new_fsm_enum == READY) {
      preprocessing_task.height_0 =
          approx_moving_average(calculate_height(preprocessing_task.m_si_data.pressure), false);
      global_flight_stats.height_0 = preprocessing_task.height_0;
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    transform_data(&preprocessing_task.m_si_data, &preprocessing_task.calibration, preprocessing_task.height_0,
                   &preprocessing_task.state_est_input);

#ifdef USE_MEDIAN_FILTER
    /* Filter the data */
    median_filter(&preprocessing_task.filter_data, &preprocessing_task.state_est_input);
#endif

    /* reset old fsm enum */
    preprocessing_task.old_fsm_enum = preprocessing_task.new_fsm_enum;

    memcpy(&preprocessing_task.m_si_data_old, &preprocessing_task.m_si_data, sizeof(preprocessing_task.m_si_data));

    /* write input data into global struct */
    global_estimation_input = preprocessing_task.state_est_input;

    /* Global SI data is only used in the fsm task */
    global_SI_data = preprocessing_task.m_si_data;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task

static void avg_and_to_SI(SI_data_t *SI_data, SI_data_t *SI_data_old, const sensor_elimination_t *elimination_data) {
  float32_t counter = 0;
  auto &preprocessing_task = task::Preprocessing::GetInstance();
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
      SI_data->acc.x += (float32_t)preprocessing_task.m_imu_data[i].acc.x * acc_info[i].conversion_to_SI;
      SI_data->acc.y += (float32_t)preprocessing_task.m_imu_data[i].acc.y * acc_info[i].conversion_to_SI;
      SI_data->acc.z += (float32_t)preprocessing_task.m_imu_data[i].acc.z * acc_info[i].conversion_to_SI;
      SI_data->gyro.x += (float32_t)preprocessing_task.m_imu_data[i].gyro.x * gyro_info[i].conversion_to_SI;
      SI_data->gyro.y += (float32_t)preprocessing_task.m_imu_data[i].gyro.y * gyro_info[i].conversion_to_SI;
      SI_data->gyro.z += (float32_t)preprocessing_task.m_imu_data[i].gyro.z * gyro_info[i].conversion_to_SI;
    }
  }

#if NUM_ACCELEROMETER > 0
  /* If all IMUs have been eliminated use high G accel */
  if (counter == 0) {
    for (int i = 0; i < NUM_ACCELEROMETER; i++) {
      if (elimination_data->faulty_acc[i] == 0) {
        counter++;
        SI_data->acc.x += (float32_t)global_acc[i].x * acc_info[NUM_IMU + i].conversion_to_SI;
        SI_data->acc.y += (float32_t)global_acc[i].y * acc_info[NUM_IMU + i].conversion_to_SI;
        SI_data->acc.z += (float32_t)global_acc[i].z * acc_info[NUM_IMU + i].conversion_to_SI;
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
      SI_data->pressure += (float32_t)preprocessing_task.m_baro_data[i].pressure * baro_info[i].conversion_to_SI;
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

static void transform_data(const SI_data_t *SI_data, const calibration_data_t *calibration, float32_t height_0,
                           state_estimation_input_t *state_data) {
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
  state_data->height_AGL = calculate_height(SI_data->pressure) - height_0;
}
