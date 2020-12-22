/*
 * util.h
 *
 *  Created on: Feb 24, 2020
 *      Author: stoja
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "cmsis_os.h"

/* Constants */
#define PREPROCESS_QUEUE_SIZE 32
#define BARO_MUTEX_TIMEOUT    0
#define IMU_MUTEX_TIMEOUT     0
#define GRAVITY               9.81f

/** BASIC TYPES **/

/* Timestamp */
typedef uint32_t timestamp_t;

/** SENSOR DATA TYPES **/

/* IMU data */
typedef struct {
  timestamp_t ts;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t acc_x, acc_y, acc_z;
} imu_data_t;

/* Barometer data */
typedef struct {
  timestamp_t ts;
  int32_t pressure;
  int32_t temperature;
} baro_data_t;

typedef struct {
  float pressure[3];
  float temperature[3];
  float acceleration[3];
  float calculated_AGL[3];
  timestamp_t ts;
} state_estimation_data_t;

typedef struct {
  int32_t num_freeze[9];
  int32_t num_maj_vote[9];
  float last_value[9];
  uint8_t faulty_imu[3];
  uint8_t faulty_baro[3];
  uint8_t num_faulty_imus;
  uint8_t num_faulty_baros;
} sensor_elimination_t;

typedef struct {
  float angle;
  uint8_t axis;
} calibration_data_t;

typedef enum {
  MOVING = 0,
  IDLE,
  THRUSTING_1,
  THRUSTING_2,
  COASTING,
  TRANSSONIC_1,
  SUPERSONIC,
  TRANSSONIC_2,
  APOGEE,
  PARACHUTE,
  TOUCHDOWN
} flight_fsm_e;

typedef struct {
  flight_fsm_e flight_state;
  imu_data_t old_imu_data;
  uint32_t clock_memory;
  uint32_t memory;
  uint8_t state_changed;
} flight_fsm_t;

typedef struct {
  float Ad[3][3];
  float Ad_T[3][3];
  float Gd[3];
  float Bd[3];
  float Q;
  float GdQGd_T[3][3];
  float H_full[3][3];
  float H_full_T[3][3];
  float H_eliminated[2][3];
  float H_eliminated_T[3][2];
  float R_full[3][3];
  float R_eliminated[2][2];
  float K_full[3][3];
  float K_eliminated[3][2];
  float x_bar[3];
  float P_bar[3][3];
  float x_hat[3];
  float P_hat[3][3];
  float pressure_0;
  float t_sampl;
} kalman_filter_t;

typedef enum {
  CATS_OK = 0,
  CATS_BARO_ERROR,
  CATS_IMU_ERROR,
  CATS_FILTER_ERROR,
  CATS_HARD_FAULT
} cats_status_e;

static const imu_data_t EMPTY_IMU = {0};

/** DEBUGGING **/

/* Read In Fake Sensor Data */
#define USB_DATA_ENABLE 0

/* Debug flag */
#ifdef CATS_DEBUG
#undef CATS_DEBUG
#endif
/* Comment the next line in order to disable debug mode */
#define CATS_DEBUG

#if (configUSE_TRACE_FACILITY == 1) && defined(CATS_DEBUG)
#undef CATS_DEBUG
#endif

#ifdef CATS_DEBUG
#define PRINT_BUFFER_LEN 200
extern osMutexId_t print_mutex;
extern char print_buffer[PRINT_BUFFER_LEN];
#endif

uint8_t UsbPrint(const char *format, ...);

#endif /* INC_UTIL_H_ */
