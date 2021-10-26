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

#include "cmsis_os.h"
#include "tasks/task_imu_read.h"
#include "sensors/icm20601.h"
#include "sensors/mmc5983ma.h"
#include "sensors/h3lis100dl.h"
#include "util/recorder.h"
#include "config/globals.h"
#include "util/log.h"

#include <string.h>

//#define CALIBRATE_ACCEL

/** Private Constants **/

/** Private Function Declarations **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], /*int16_t *temperature,*/ int32_t id);

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_imu_read thread.
 * @param argument: Not used
 * @retval None
 */
void task_imu_read(void *argument) {
  uint32_t tick_count, tick_update;

  /* Initialize IMU data variables */
  int16_t gyroscope[3] = {0};    /* 0 = x, 1 = y, 2 = z */
  int16_t acceleration[3] = {0}; /* 0 = x, 1 = y, 2 = z */
  /*int16_t temperature = {0};*/

  /* Initialize MAGNETO data variables */
  magneto_data_t magneto_data = {0};
  float tmp_mag[3] = {0};

  /* Initialize ACCELEROMETER data variables */
  accel_data_t accel_data = {0};
  int8_t tmp_accel[3];

  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  /* Infinite loop */
  while (1) {
    tick_count += tick_update;

    /* Read and Save Magnetometer Data */
    for (int i = 0; i < NUM_MAGNETO; i++) {
      /* TODO: currently we are reading only MAG; in case we have MAG2 we have to change this function call */
      mmc5983ma_read_calibrated(&MAG, tmp_mag);
      magneto_data.magneto_x = tmp_mag[0];
      magneto_data.magneto_y = tmp_mag[1];
      magneto_data.magneto_z = tmp_mag[2];
      magneto_data.ts = osKernelGetTickCount();
      // log_info("Magneto %ld: RAW Mx: %ld, My:%ld, Mz:%ld", 1, (int32_t)((float)tmp_mag[0] * 1000),
      //         (int32_t)((float)tmp_mag[1] * 1000), (int32_t)((float)tmp_mag[2] * 1000));

      global_magneto[i] = magneto_data;
      record(add_id_to_record_type(MAGNETO, i), &magneto_data);
    }

    /* Read and Save High-G IMU Data */
    for (int i = 0; i < NUM_ACCELEROMETER; i++) {
      /* TODO: currently we are reading only ACCEL; in case we have ACCEL2 we have to change this function call */
      h3lis100dl_read_raw(&ACCEL, tmp_accel);
      accel_data.acc_x = tmp_accel[0];
      accel_data.acc_y = tmp_accel[1];
      accel_data.acc_z = tmp_accel[2];
      accel_data.ts = tick_count;
      // memcpy(&(global_accel.acc_x), &(accel_data.acc_x), 3 * sizeof(int8_t));
      // global_accel.ts = tick_count;
      record(add_id_to_record_type(ACCELEROMETER, i), &(accel_data));
    }

    for (int i = 0; i < NUM_IMU; i++) {
      read_imu(gyroscope, acceleration, /*&temperature,*/ i);
      memcpy(&(global_imu[i].acc_x), &acceleration, 3 * sizeof(int16_t));
      memcpy(&(global_imu[i].gyro_x), &gyroscope, 3 * sizeof(int16_t));
      global_imu[i].ts = tick_count;
      record(add_id_to_record_type(IMU, i), &(global_imu[i]));
    }

    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], /*int16_t *temperature,*/ int32_t id) {
  switch (id) {
    case 0:
      icm20601_read_accel_raw(&ICM1, acceleration);
      icm20601_read_gyro_raw(&ICM1, gyroscope);
      // icm20601_read_temp_raw(&ICM1, temperature);
      break;
    case 1:
      icm20601_read_accel_raw(&ICM2, acceleration);
      icm20601_read_gyro_raw(&ICM2, gyroscope);
      // icm20601_read_temp_raw(&ICM2, temperature);
      break;
    default:
      break;
  }
}