/*
 * task_state_est.h
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#pragma once

_Noreturn void task_state_est(void *argument);

/* Offset Settings */
#define OFFSET_BARO
#define OFFSET_IMU
#define OFFSET_SENSOR_CHOICE 1
#define OFFSET_P             1500 /* Pa */
#define OFFSET_ACC           5    /* m/s^2 */

/* Spike Settings */
#define SPIKE_BARO
#define SPIKE_IMU
#define SPIKE_SENSOR_CHOICE 1
#define SPIKE_THRESHOLD                                                    \
  0.001f /* rng between 0 and 1 and it it is smaller than the threshold we \
            inject a spike */

/* Noise Settings */
#define ACC_NOISE_MAX_AMPL      0.2f  /* In m/s^2 */
#define PRESSURE_NOISE_MAX_AMPL 10.0f /* In Pa */
