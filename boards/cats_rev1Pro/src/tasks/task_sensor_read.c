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
#include "tasks/task_sensor_read.h"
#include "sensors/icm20601.h"
#include "sensors/mmc5983ma.h"
#include "sensors/h3lis100dl.h"
#include "sensors/ms5607.h"
#include "util/log.h"
#include "util/recorder.h"
#include "config/globals.h"

#include <string.h>

//#define CALIBRATE_ACCEL

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
void task_sensor_read(void *argument) {
  /* Initialize IMU data variables */
  int16_t gyroscope[3] = {0};    /* 0 = x, 1 = y, 2 = z */
  int16_t acceleration[3] = {0}; /* 0 = x, 1 = y, 2 = z */
  int16_t temperature_imu = {0};

  /* Initialize BARO data variables */
  uint32_t stage = READ_BARO_TEMPERATURE;
  int32_t temperature_baro[NUM_BARO];
  int32_t pressure[NUM_BARO];
  prepare_temp();
  osDelay(5);

  /* initialize MAGNETO data variables */
  float mag_data[3] = {0};

  uint32_t tick_count = osKernelGetTickCount();
  /* This task is sampled with 2 times the control sampling frequency to maximize speed of the barometer. In one
   * timestep the Baro pressure is read out and then the Baro Temperature. The other sensors are only read out one in
   * two times. */
  uint32_t tick_update = osKernelGetTickFreq() / (2 * CONTROL_SAMPLING_FREQ);
  while (1) {
    // Readout the baro register
    read_baro();

    // Prepare new readout for the baro
    if (stage == READ_BARO_TEMPERATURE) {
      prepare_pres();
      stage = READ_BARO_PRESSURE;
    } else {
      prepare_temp();
      stage = READ_BARO_TEMPERATURE;

      get_temp_pres(temperature_baro, pressure);

      /* Read and Save Barometric Data */
      for (int i = 0; i < NUM_BARO; i++) {
        global_baro[i].pressure = pressure[i];
        global_baro[i].temperature = temperature_baro[i];
        record(tick_count, add_id_to_record_type(BARO, i), &(global_baro[i]));
      }

      /* Read and Save Magnetometer Data */
      for (int i = 0; i < NUM_MAGNETO; i++) {
        mmc5983ma_read_calibrated(&MAG, mag_data);
        memcpy(&(global_magneto[i].magneto_x), &mag_data, 3 * sizeof(float));
        record(tick_count, add_id_to_record_type(MAGNETO, i), &(global_magneto[i]));
      }

      /* Read and Save High-G ACC Data */
      for (int i = 0; i < NUM_ACCELEROMETER; i++) {
        int8_t tmp_data[3];
        h3lis100dl_read_raw(&ACCEL, tmp_data);
        memcpy(&(global_accel[i].acc_x), &tmp_data, 3 * sizeof(int8_t));
        record(tick_count, add_id_to_record_type(ACCELEROMETER, i), &(global_accel[i]));
      }

      /* Read and Save IMU Data */
      for (int i = 0; i < NUM_IMU; i++) {
        read_imu(gyroscope, acceleration, &temperature_imu, i);
        memcpy(&(global_imu[i].acc_x), &acceleration, 3 * sizeof(int16_t));
        memcpy(&(global_imu[i].gyro_x), &gyroscope, 3 * sizeof(int16_t));
        record(tick_count, add_id_to_record_type(IMU, i), &(global_imu[i]));
      }
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int16_t *temperature, int32_t id) {
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

static void prepare_temp() {
  ms5607_prepare_temp(&MS1);
  ms5607_prepare_temp(&MS2);
  ms5607_prepare_temp(&MS3);
}
//
static void prepare_pres() {
  ms5607_prepare_pres(&MS1);
  ms5607_prepare_pres(&MS2);
  ms5607_prepare_pres(&MS3);
}

static void read_baro() {
  ms5607_read_raw(&MS1);
  ms5607_read_raw(&MS2);
  ms5607_read_raw(&MS3);
}

static void get_temp_pres(int32_t *temperature, int32_t *pressure) {
  ms5607_get_temp_pres(&MS1, &temperature[0], &pressure[0]);
  ms5607_get_temp_pres(&MS2, &temperature[1], &pressure[1]);
  ms5607_get_temp_pres(&MS3, &temperature[2], &pressure[2]);
}
