/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include <cstring>

#include "sensors/ms5607.h"
#include "util/log.h"
#include "util/task_util.h"

/** Private Function Declarations **/

static void read_imu(vi16_t &gyroscope, vi16_t &acceleration, int32_t id);

static void prepare_temp();

static void prepare_pres();

static void get_temp_pres(int32_t *temperature, int32_t *pressure);

static void read_baro();

static void read_mag(vf32_t *data);

static void read_accel(vi8_t *data);

namespace task {

baro_data_t SensorRead::GetBaro(uint8_t index) const noexcept { return m_baro_data[index]; }

imu_data_t SensorRead::GetImu(uint8_t index) const noexcept { return m_imu_data[index]; }

magneto_data_t SensorRead::GetMag(uint8_t index) const noexcept { return m_magneto_data[index]; }

accel_data_t SensorRead::GetAccel(uint8_t index) const noexcept { return m_accel_data[index]; }

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_sens_read thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void SensorRead::Run() noexcept {
  /* Initialize IMU data variables */
  m_barometer->Prepare(sensor::Ms5607::Request::kTemperature);
  osDelay(5);

  uint32_t tick_count = osKernelGetTickCount();
  /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
   * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
   * two times. */
  constexpr uint32_t tick_update = sysGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
  while (true) {
    // Readout the baro register
    m_barometer->Read();

    // Prepare new readout for the baro
    if (m_current_readout == SensorRead::BaroReadoutType::kReadBaroPressure) {
      m_barometer->Prepare(sensor::Ms5607::Request::kPressure);
      m_current_readout = SensorRead::BaroReadoutType::kReadBaroTemperature;
    } else {
      m_barometer->Prepare(sensor::Ms5607::Request::kTemperature);
      m_current_readout = SensorRead::BaroReadoutType::kReadBaroPressure;
      /* For Simulator */
      if (simulation_started) {
        for (int i = 0; i < NUM_BARO; i++) {
          m_baro_data[i].pressure = global_baro_sim[i].pressure;
        }
      } else {
        m_barometer->GetMeasurement(m_baro_data[0].pressure, m_baro_data[0].temperature);
      }

      /* Read and Save Barometric Data */
      for (int i = 0; i < NUM_BARO; i++) {
        record(tick_count, add_id_to_record_type(BARO, i), &(m_baro_data[0]));
      }

      /* Read and Save Magnetometer Data */
      for (int i = 0; i < NUM_MAGNETO; i++) {
        record(tick_count, add_id_to_record_type(MAGNETO, i), &(m_magneto_data[i]));
      }

      /* Read and Save High-G ACC Data */
      for (int i = 0; i < NUM_ACCELEROMETER; i++) {
        record(tick_count, add_id_to_record_type(ACCELEROMETER, i), &(m_accel_data[i]));
      }

      /* Read and Save IMU Data */
      for (int i = 0; i < NUM_IMU; i++) {
        if (simulation_started) {
          m_imu_data[i].acc = global_imu_sim[i].acc;
        } else {
          m_imu->ReadGyroRaw(reinterpret_cast<int16_t *>(&m_imu_data[i].gyro));
          m_imu->ReadAccelRaw(reinterpret_cast<int16_t *>(&m_imu_data[i].acc));
        }
        record(tick_count, add_id_to_record_type(IMU, i), &(m_imu_data[i]));
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

}  // namespace task
