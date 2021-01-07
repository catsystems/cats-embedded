//
// Created by stoja on 23.12.20.
//

#ifndef CATS_REV1_TYPES_H
#define CATS_REV1_TYPES_H

#include <stdint.h>

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
  INVALID = 0,
  MOVING = 1,
  IDLE,
  THRUSTING_1,
  THRUSTING_2,
  COASTING,
  TRANSONIC_1,
  SUPERSONIC,
  TRANSONIC_2,
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

typedef enum {
  CATS_ERROR_OK = 0,
  CATS_ERROR_NO_CONFIG,
  CATS_ERROR_NO_PYRO,
  CATS_ERROR_LOG_FULL,
  CATS_ERROR_USB_CONNECTED,
  CATS_ERROR_BAT_LOW,
  CATS_ERROR_BAT_CRIT,
  CATS_ERROR_IMU_ERROR,
  CATS_ERROR_BARO_ERROR,
  CATS_ERROR_FILTER_ERROR,
  CATS_ERROR_HARD_FAULT
} cats_error_e;

/** CONVERSION FUNCTIONS **/

inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low) {
  return (src_high << 8 | src_low);
}

/* TODO: is this really the same? It's taken from the macros.. */
inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low) {
  return uint8_to_uint16(src_high, src_low);
}

#endif  // CATS_REV1_TYPES_H