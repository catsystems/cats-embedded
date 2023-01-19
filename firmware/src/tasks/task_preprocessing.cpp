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

namespace task {

/**
 * @brief Function implementing the task_preprocessing thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void Preprocessing::Run() {
  /* Get Tasks */
  auto &sensor_read_task = SensorRead::GetInstance();

  /* Infinite loop */
  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;
  while (true) {
    /* update fsm enum */
    m_new_fsm_enum = global_flight_state.flight_state;

    /* get new sensor data */
    m_baro_data[0] = sensor_read_task.GetBaro(0);
    m_imu_data[0] = sensor_read_task.GetImu(0);
    m_magneto_data[0] = sensor_read_task.GetMag(0);
    m_accel_data[0] = sensor_read_task.GetAccel(0);

    /* Do the sensor elimination */
    CheckSensors();

    /* average and construct SI Data */
    AvgToSi();

    /* Compute gravity when changing to READY */
    if ((m_new_fsm_enum != m_old_fsm_enum) && (m_new_fsm_enum == READY)) {
      calibrate_imu(&m_si_data.acc, &m_calibration);
      global_flight_stats.calibration_data.angle = m_calibration.angle;
      global_flight_stats.calibration_data.axis = m_calibration.axis;
    }

    /* calibrate gyro once at startup */
    if (!m_gyro_calibrated) {
      m_gyro_calibrated = compute_gyro_calibration(&m_si_data.gyro, &m_calibration);
    } else {
      calibrate_gyro(&m_calibration, &m_si_data.gyro);
      global_flight_stats.calibration_data.gyro_calib = m_calibration.gyro_calib;
    }

    /* Compute current height constantly before liftoff. If the state is moving, the filter is much faster. */
    if (m_new_fsm_enum == MOVING) {
      m_height_0 = approx_moving_average(calculate_height(m_si_data.pressure), true);
      global_flight_stats.height_0 = m_height_0;
    }
    /* Compute current height constantly before liftoff. If the state is ready, the filter is much slower. */
    if (m_new_fsm_enum == READY) {
      m_height_0 = approx_moving_average(calculate_height(m_si_data.pressure), false);
      global_flight_stats.height_0 = m_height_0;
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    TransformData();

#ifdef USE_MEDIAN_FILTER
    /* Filter the data */
    MedianFilter();
#endif

    /* reset old fsm enum */
    m_old_fsm_enum = m_new_fsm_enum;

    memcpy(&m_si_data_old, &m_si_data, sizeof(m_si_data));

    /* write input data into global struct */
    global_estimation_input = m_state_est_input;

    /* Global SI data is only used in the fsm task */
    global_SI_data = m_si_data;

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

void Preprocessing::AvgToSi() {
  float32_t counter = 0;
#if NUM_IMU > 0
  /* Reset SI data */
  m_si_data.acc.x = 0;
  m_si_data.acc.y = 0;
  m_si_data.acc.z = 0;
  m_si_data.gyro.x = 0;
  m_si_data.gyro.y = 0;
  m_si_data.gyro.z = 0;

  /* Sum up all non-eliminated IMUs and transform to SI */
  for (int i = 0; i < NUM_IMU; i++) {
    if (m_sensor_elimination.faulty_imu[i] == 0) {
      counter++;
      m_si_data.acc.x += (float32_t)m_imu_data[i].acc.x * acc_info[i].conversion_to_SI;
      m_si_data.acc.y += (float32_t)m_imu_data[i].acc.y * acc_info[i].conversion_to_SI;
      m_si_data.acc.z += (float32_t)m_imu_data[i].acc.z * acc_info[i].conversion_to_SI;
      m_si_data.gyro.x += (float32_t)m_imu_data[i].gyro.x * gyro_info[i].conversion_to_SI;
      m_si_data.gyro.y += (float32_t)m_imu_data[i].gyro.y * gyro_info[i].conversion_to_SI;
      m_si_data.gyro.z += (float32_t)m_imu_data[i].gyro.z * gyro_info[i].conversion_to_SI;
    }
  }

#if NUM_ACCELEROMETER > 0
  /* If all IMUs have been eliminated use high G accel */
  if (counter == 0) {
    for (int i = 0; i < NUM_ACCELEROMETER; i++) {
      if (elimination_data->faulty_acc[i] == 0) {
        counter++;
        this->m_si_data.acc.x += (float32_t)global_acc[i].x * acc_info[NUM_IMU + i].conversion_to_SI;
        this->m_si_data.acc.y += (float32_t)global_acc[i].y * acc_info[NUM_IMU + i].conversion_to_SI;
        this->m_si_data.acc.z += (float32_t)global_acc[i].z * acc_info[NUM_IMU + i].conversion_to_SI;
      }
    }
  }
#endif

  /* average for SI data */
  if (counter > 0) {
    m_si_data.acc.x /= counter;
    m_si_data.acc.y /= counter;
    m_si_data.acc.z /= counter;
    m_si_data.gyro.x /= counter;
    m_si_data.gyro.y /= counter;
    m_si_data.gyro.z /= counter;
    clear_error(CATS_ERR_FILTER_ACC);
  } else {
    m_si_data.acc = m_si_data_old.acc;
    m_si_data.gyro = m_si_data_old.gyro;
    add_error(CATS_ERR_FILTER_ACC);
  }

#endif

#if NUM_BARO > 0
  counter = 0;
  m_si_data.pressure = 0;
  for (int i = 0; i < NUM_BARO; i++) {
    if (m_sensor_elimination.faulty_baro[i] == 0) {
      counter++;
      m_si_data.pressure += (float32_t)m_baro_data[i].pressure * baro_info[i].conversion_to_SI;
    }
  }
  if (counter > 0) {
    m_si_data.pressure /= counter;
    clear_error(CATS_ERR_FILTER_HEIGHT);
  } else {
    m_si_data.pressure = m_si_data_old.pressure;
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

void Preprocessing::MedianFilter() {
  /* Insert into array */
  m_filter_data.acc[m_filter_data.counter] = m_state_est_input.acceleration_z;
  m_filter_data.height_AGL[m_filter_data.counter] = m_state_est_input.height_AGL;

  /* Update Counter */
  m_filter_data.counter++;
  m_filter_data.counter = m_filter_data.counter % MEDIAN_FILTER_SIZE;

  /* Filter data */
  m_state_est_input.acceleration_z = median(m_filter_data.acc);
  m_state_est_input.height_AGL = median(m_filter_data.height_AGL);
}

void Preprocessing::TransformData() {
  /* Get Data from the Sensors */
  /* Use calibration step to get the correct acceleration */
  switch (this->m_calibration.axis) {
    case 0:
      /* Choose X Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.x / m_calibration.angle - GRAVITY;
      break;
    case 1:
      /* Choose Y Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.y / m_calibration.angle - GRAVITY;
      break;
    case 2:
      /* Choose Z Axis */
      m_state_est_input.acceleration_z = m_si_data.acc.z / m_calibration.angle - GRAVITY;
      break;
    default:
      break;
  }
  this->m_state_est_input.height_AGL = calculate_height(m_si_data.pressure) - m_height_0;
}

}  // namespace task