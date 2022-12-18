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
#if IMU_TYPE == LSM6DSR_TYPE
#include "sensors/lsm6dsr.h"
#elif IMU_TYPE == ICM20601_TYPE
#include "sensors/icm20601.h"
#endif
#include "sensors/mmc5983ma.h"
#include "sensors/ms5607.h"
#include "util/log.h"

#include <cstring>

/** Private Constants **/
enum {
  READ_BARO_TEMPERATURE = 1,
  READ_BARO_PRESSURE = 2,
};

/** Private Function Declarations **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int16_t *temperature, int32_t id);

static void prepare_temp();
static void prepare_pres();
static void get_temp_pres(int32_t *temperature, int32_t *pressure);
static void read_baro();

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_sens_read thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_sensor_read(void *argument) {
  /* Initialize IMU data variables */
  int16_t gyroscope[3] = {};    /* 0 = x, 1 = y, 2 = z */
  int16_t acceleration[3] = {}; /* 0 = x, 1 = y, 2 = z */
  int16_t temperature_imu = {};

  /* Initialize BARO data variables */
  uint32_t stage = READ_BARO_TEMPERATURE;
  int32_t temperature_baro[NUM_BARO] = {};
  int32_t pressure[NUM_BARO] = {};
  prepare_temp();
  osDelay(5);

  /* initialize MAGNETO data variables */
  float32_t mag_data[3] = {};

  uint32_t tick_count = osKernelGetTickCount();
  /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
   * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
   * two times. */
  const uint32_t tick_update = osKernelGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
  while (true) {
    // Readout the baro register
    read_baro();

    // Prepare new readout for the baro
    if (stage == READ_BARO_TEMPERATURE) {
      prepare_pres();
      stage = READ_BARO_PRESSURE;
        } else {
      prepare_temp();
      stage = READ_BARO_TEMPERATURE;
      /* For Simulator */
      if (simulation_started) {
        for (int i = 0; i < NUM_BARO; i++) {
          pressure[i] = global_baro_sim[i].pressure;
        }
      } else {
        get_temp_pres(temperature_baro, pressure);
      }

      /* Read and Save Barometric Data */
      for (int i = 0; i < NUM_BARO; i++) {
        global_baro[i].pressure = pressure[i];
        global_baro[i].temperature = temperature_baro[i];
        record(tick_count, add_id_to_record_type(BARO, i), &(global_baro[i]));
      }

      /* Read and Save Magnetometer Data */
      for   (int i = 0; i < NUM_MAGNETO; i++) {
        mmc5983ma_read_calibrated(&MAG, mag_data);
        memcpy(&(global_magneto[i].x), &mag_data, 3 * sizeof(float));
        record(tick_count, add_id_to_record_type(MAGNETO, i), &(global_magneto[i]));
      }

      /* Read and Save High-G ACC Data */
      for (int i = 0; i < NUM_ACCELEROMETER; i++) {
        int8_t tmp_data[3];
        h3lis100dl_read_raw(&ACCEL, tmp_data);
        memcpy(&(global_accel[i].x), &tmp_data, 3 * sizeof(int8_t));
            record  (tick_count, add_id_to_record_type(ACCELEROMETER, i), &(global_accel[i]));
      }

      /* Read and Save IMU Data */
      for (int i = 0; i < NUM_IMU; i++) {
            if (simulation_started) {
          acceleration[0] = global_imu_sim[i].acc.x;
          acceleration[1] = global_imu_sim[i].acc.y;
          acceleration[2] = global_imu_sim[i].acc.z;
        } else {
          read_imu(gyroscope, acceleration, &temperature_imu, i);
        }
        memcpy(&(global_imu[i].acc.x), &acceleration, 3 * sizeof(int16_t));
        memcpy(&(global_imu[i].gyro.x), &gyroscope, 3 * sizeof(int16_t));
        record(12, add_id_to_record_type(IMU, i), &(global_imu[i]));
        // log_debug("IMU_Ax %hd, IMU_Gx %hd, Baro %u", global_imu[i].acc.x, global_imu[i].gyro.x,
        // global_baro[0].pressure);
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int16_t *temperature, int32_t id) {
  if (id >= NUM_IMU) return;
#if IMU_TYPE == ICM20601_TYPE
  icm20601_read_accel_raw(&IMU_DEV[id], acceleration);
  icm20601_read_gyro_raw(&IMU_DEV[id], gyroscope);
#elif IMU_TYPE == LSM6DSR_TYPE
  lsm6dsr_read_accel_raw(&IMU_DEV[id], acceleration);
  lsm6dsr_read_gyro_raw(&IMU_DEV[id], gyroscope);
#endif
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
