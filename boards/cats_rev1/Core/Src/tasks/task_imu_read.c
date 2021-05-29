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

#include <string.h>

//#define CALIBRATE_ACCEL

/** Private Constants **/

/** Private Function Declarations **/

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int32_t id);

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

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  while (1) {
    for (int i = 0; i < 3; i++) {
      read_imu(gyroscope, acceleration, i);
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

static void read_imu(int16_t gyroscope[3], int16_t acceleration[3], int32_t id) {
  switch (id) {
    case 0:
      icm20601_read_accel_raw(&ICM1, acceleration);
      icm20601_read_gyro_raw(&ICM1, gyroscope);
      break;
    case 1:
      icm20601_read_accel_raw(&ICM2, acceleration);
      icm20601_read_gyro_raw(&ICM2, gyroscope);
      break;
    case 2:
      icm20601_read_accel_raw(&ICM3, acceleration);
      icm20601_read_gyro_raw(&ICM3, gyroscope);
      break;
    default:
      break;
  }
}
