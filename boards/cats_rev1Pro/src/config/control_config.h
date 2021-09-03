//
// Created by stoja on 29.05.21.
//

#pragma once
#include "sensor_config.h"
//#define INCLUDE_NOISE

//#define INCLUDE_SPIKES
//#define INCLUDE_OFFSET

#define USE_ORIENTATION_FILTER
#define USE_ADAPTIVE_GAIN

//#define USE_ORIENTATION_KF
#ifdef USE_ORIENTATION_KF
#define INIT_COV   0.1f
#define NOISE_VEL  0.01f
#define NOISE_POS  0.03f
#define NOISE_BIAS 0.01f
#endif

#if defined(USE_ORIENTATION_FILTER) && defined(USE_ORIENTATION_KF)
#undef USE_ORIENTATION_KF
#endif

#define USE_MEDIAN_FILTER
#define MEDIAN_FILTER_SIZE 9

#define HIGH_G_ACC_INDEX 2
#define NUM_ACC          (NUM_IMU + NUM_ACCELEROMETER)
#define NUM_GYRO         NUM_IMU
#define NUM_PRESSURE     NUM_BARO
#define NUM_TEMPERATURE  NUM_BARO
#define NUM_MAG_FIELD    NUM_MAGNETO