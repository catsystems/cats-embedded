/*
 * task_imu_read.c
 *
 *  Created on: Nov 3, 2019
 *      Author: Jonas
 */

#include "cmsis_os.h"
#include "tasks/task_imu_read.h"
#include "sensors/icm20601.h"
#include "util/recorder.h"
#include "config/globals.h"
#include "util/log.h"

#include <string.h>

//#define CALIBRATE_ACCEL

/** Private Constants **/

/** Private Function Declarations **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3],
                     int16_t *temperature, int32_t id);

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_baro_read thread.
 * @param argument: Not used
 * @retval None
 */
_Noreturn void task_imu_read(void *argument) {
  uint32_t tick_count, tick_update;

  /* initialize data variables */
  int16_t gyroscope[3] = {0};    /* 0 = x, 1 = y, 2 = z */
  int16_t acceleration[3] = {0}; /* 0 = x, 1 = y, 2 = z */
  int16_t temperature;

  /* initialize queue message */
  // imu_data_t queue_data = { 0 };

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    /* Debugging */

    //    log_info(
    //        "IMU %ld: RAW Gx: %hd, Gy:%hd, Gz:%hd; Ax: %hd, Ay:%hd, Az:%hd, "
    //        "T:%hd",
    //        imu_idx, gyroscope[0], gyroscope[1], gyroscope[2],
    //        acceleration[0], acceleration[1], acceleration[2], temperature);

    //    global_imu[imu_idx].acc_x = acceleration[0];
    //    global_imu[imu_idx].acc_y = acceleration[1];
    //    global_imu[imu_idx].acc_z = acceleration[2];
    //    global_imu[imu_idx].gyro_x = gyroscope[0];
    //    global_imu[imu_idx].gyro_y = gyroscope[1];
    //    global_imu[imu_idx].gyro_z = gyroscope[2];

    /* TODO: The speed of copying looks to be the same, code size reduced by 16B
     * with memcpy vs. assignment with -0g */
    for (int i = 0; i < 3; i++) {
      read_imu(gyroscope, acceleration, &temperature, i);
      memcpy(&(global_imu[i].acc_x), &acceleration, 3 * sizeof(int16_t));
      memcpy(&(global_imu[i].gyro_x), &gyroscope, 3 * sizeof(int16_t));
      global_imu[i].ts = tick_count;
      record(IMU0 << i, &(global_imu[i]));
    }

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3],
                     int16_t *temperature, int32_t id) {
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
