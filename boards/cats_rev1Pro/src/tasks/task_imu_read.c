/*
 * task_imu_read.c
 *
 *  Created on: Nov 3, 2019
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "tasks/task_imu_read.h"
#include "sensors/icm20601.h"
#include "sensors/mmc5983ma.h"
#include "util/recorder.h"
#include "config/globals.h"
#include "util/log.h"

#include <string.h>

//#define CALIBRATE_ACCEL

/** Private Constants **/

/** Private Function Declarations **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int16_t *temperature, int32_t id);

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
void task_imu_read(void *argument) {
  uint32_t tick_count, tick_update;

  /* initialize IMU data variables */
  int16_t gyroscope[3] = {0};    /* 0 = x, 1 = y, 2 = z */
  int16_t acceleration[3] = {0}; /* 0 = x, 1 = y, 2 = z */
  int16_t temperature = {0};
  accel_data_t accel_data = {0};

  /* initialize MAGNETO data variables */
  magneto_data_t magneto_data = {0};
  float data[3] = {0};

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;

    /* Debugging */

    //        log_info(
    //            "IMU %ld: RAW Gx: %hd, Gy:%hd, Gz:%hd; Ax: %hd, Ay:%hd, Az:%hd, "
    //            "T:%hd",
    //            2, gyroscope[0], gyroscope[1], gyroscope[2],
    //            acceleration[0], acceleration[1], acceleration[2], temperature);

    /* Read and Save Magnetometer Data */
    mmc5983ma_read_calibrated(&MAG, data);
    magneto_data.magneto_x = data[0];
    magneto_data.magneto_y = data[1];
    magneto_data.magneto_z = data[2];
    magneto_data.ts = osKernelGetTickCount();
    // log_info("Magneto %ld: RAW Mx: %ld, My:%ld, Mz:%ld", 1, (int32_t)((float)data[0] * 1000),
    //         (int32_t)((float)data[1] * 1000), (int32_t)((float)data[2] * 1000));

    global_magneto = magneto_data;
    record(MAGNETO, &magneto_data);

    /* Read and Save High-G IMU Data */
    /* Todo: Read High-G IMU Data */
    memcpy(&(global_accel.acc_x), &(accel_data.acc_x), 3 * sizeof(int8_t));
    global_accel.ts = tick_count;
    record(ACCELEROMETER, &(global_accel));

    /* TODO: The speed of copying looks to be the same, code size reduced by 16B
     * with memcpy vs. assignment with -0g */
    for (int i = 0; i < NUM_IMU; i++) {
      read_imu(gyroscope, acceleration, &temperature, i);
      memcpy(&(global_imu[i].acc_x), &acceleration, 3 * sizeof(int16_t));
      memcpy(&(global_imu[i].gyro_x), &gyroscope, 3 * sizeof(int16_t));
      global_imu[i].ts = tick_count;
      record(IMU0 << i, &(global_imu[i]));
    }

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
    case 2:
      icm20601_read_accel_raw(&ICM3, acceleration);
      icm20601_read_gyro_raw(&ICM3, gyroscope);
      // icm20601_read_temp_raw(&ICM3, temperature);
      break;
    default:
      break;
  }
}