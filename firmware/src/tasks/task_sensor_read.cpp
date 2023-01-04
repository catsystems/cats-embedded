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

#include "tasks/task_sensor_read.h"
#include "cmsis_os.h"
#include "config/globals.h"
#include "flash/recorder.h"
#include "sensors/h3lis100dl.h"
#include "util/task_util.h"

#if IMU_TYPE == LSM6DSR_TYPE

#include "sensors/lsm6dsr.h"

#elif IMU_TYPE == ICM20601_TYPE
#include "sensors/icm20601.h"
#endif

#include "sensors/mmc5983ma.h"
#include "sensors/ms5607.h"
#include "util/log.h"

#include <cstring>

/** Private Function Declarations **/

static void read_imu(vi16_t &gyroscope, vi16_t &acceleration, int32_t id);

static void prepare_temp();

static void prepare_pres();

static void get_temp_pres(int32_t *temperature, int32_t *pressure);

static void read_baro();

static void read_mag(vf32_t *data);

static void read_accel(vi8_t *data);

namespace task {
/* Todo: Check with Trace if can be reduced */
SET_TASK_PARAMS(task_sensor_read, 512)

void SensorRead::Run() { osThreadNew(task_sensor_read, nullptr, &task_sensor_read_attributes); }

baro_data_t SensorRead::GetBaro(uint8_t index) { return m_baro_data[index]; }

imu_data_t SensorRead::GetImu(uint8_t index) { return m_imu_data[index]; }

magneto_data_t SensorRead::GetMag(uint8_t index) { return m_magneto_data[index]; }

accel_data_t SensorRead::GetAccel(uint8_t index) { return m_accel_data[index]; }

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_sens_read thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_sensor_read(void *argument [[maybe_unused]]) {
  /* Initialize IMU data variables */

  auto &task = SensorRead::GetInstance();

  prepare_temp();
  osDelay(5);

  uint32_t tick_count = osKernelGetTickCount();
  /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
   * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
   * two times. */
  const uint32_t tick_update = osKernelGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
  while (true) {
    // Readout the baro register
    read_baro();

    // Prepare new readout for the baro
    if (task.m_current_readout == SensorRead::BaroReadoutType::kReadBaroPressure) {
      prepare_pres();
      task.m_current_readout = SensorRead::BaroReadoutType::kReadBaroTemperature;
    } else {
      prepare_temp();
      task.m_current_readout = SensorRead::BaroReadoutType::kReadBaroPressure;
      /* For Simulator */
      if (simulation_started) {
        for (int i = 0; i < NUM_BARO; i++) {
          task.m_baro_data[i].pressure = global_baro_sim[i].pressure;
        }
      } else {
        get_temp_pres(&(task.m_baro_data[0].temperature), &(task.m_baro_data[0].pressure));
      }

      /* Read and Save Barometric Data */
      for (int i = 0; i < NUM_BARO; i++) {
        record(tick_count, add_id_to_record_type(BARO, i), &(task.m_baro_data[0]));
      }

      /* Read and Save Magnetometer Data */
      for (int i = 0; i < NUM_MAGNETO; i++) {
        read_mag(&task.m_magneto_data[i]);
        record(tick_count, add_id_to_record_type(MAGNETO, i), &(task.m_magneto_data[i]));
      }

      /* Read and Save High-G ACC Data */
      for (int i = 0; i < NUM_ACCELEROMETER; i++) {
        read_accel(&task.m_accel_data[i]);
        record(tick_count, add_id_to_record_type(ACCELEROMETER, i), &(task.m_accel_data[i]));
      }

      /* Read and Save IMU Data */
      for (int i = 0; i < NUM_IMU; i++) {
        if (simulation_started) {
          task.m_imu_data[i].acc = global_imu_sim[i].acc;
        } else {
          read_imu(task.m_imu_data[i].gyro, task.m_imu_data[i].acc, i);
        }
        record(tick_count, add_id_to_record_type(IMU, i), &(task.m_imu_data[i]));
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}
}  // namespace task

/** Private Function Definitions **/

static void read_imu(vi16_t &gyroscope, vi16_t &acceleration, int32_t id) {
  int16_t acc[3] = {};
  int16_t gyro[3] = {};
  if (id >= NUM_IMU) {
    return;
  }
#if IMU_TYPE == ICM20601_TYPE
  icm20601_read_accel_raw(&IMU_DEV[id], acceleration);
  icm20601_read_gyro_raw(&IMU_DEV[id], gyroscope);
#elif IMU_TYPE == LSM6DSR_TYPE
  // lsm6dsr_read_accel_raw(&IMU_DEV[id], acc);
  // lsm6dsr_read_gyro_raw(&IMU_DEV[id], gyro);
#endif
  memcpy(&acceleration, acc, 3 * sizeof(int16_t));
  memcpy(&gyroscope, gyro, 3 * sizeof(int16_t));
}

static void prepare_temp() {
  for (int32_t i = 0; i < NUM_BARO; ++i) {
    ms5607_prepare_temp(&BARO_DEV[i]);
  }
}

//
static void prepare_pres() {
  for (int32_t i = 0; i < NUM_BARO; ++i) {
    ms5607_prepare_pres(&BARO_DEV[i]);
  }
}

static void read_baro() {
  for (int32_t i = 0; i < NUM_BARO; ++i) {
    ms5607_read_raw(&BARO_DEV[i]);
  }
}

static void get_temp_pres(int32_t *temperature, int32_t *pressure) {
  for (int32_t i = 0; i < NUM_BARO; ++i) {
    ms5607_get_temp_pres(&BARO_DEV[i], &temperature[i], &pressure[i]);
  }
}

static void read_mag(vf32_t *data) {
  float tmp[3] = {};
  mmc5983ma_read_calibrated(&MAG, tmp);
  data->x = tmp[0];
  data->y = tmp[1];
  data->z = tmp[2];
}

static void read_accel(vi8_t *data) {
  int8_t tmp[3] = {};
  h3lis100dl_read_raw(&ACCEL, tmp);
  data->x = tmp[0];
  data->y = tmp[1];
  data->z = tmp[2];
}
