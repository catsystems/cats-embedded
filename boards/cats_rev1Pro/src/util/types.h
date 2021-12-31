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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"
#include "cmsis_os2.h"
#include "config/control_config.h"

/** DEFINES **/

#define NUM_EVENTS 9
#define NUM_TIMERS 4

/** BASIC TYPES **/

/* Timestamp */
typedef uint32_t timestamp_t;  // ms

/** SENSOR DATA TYPES **/

/* IMU data */
typedef struct {
  timestamp_t ts;
  int16_t gyro_x, gyro_y, gyro_z;  // IMU unit
  int16_t acc_x, acc_y, acc_z;     // IMU unit
} imu_data_t;

/* Accel data */
typedef struct {
  timestamp_t ts;
  int8_t acc_x, acc_y, acc_z;  // Accel unit
} accel_data_t;

/* Barometer data */
typedef struct {
  timestamp_t ts;
  int32_t pressure;     // Baro unit
  int32_t temperature;  // Baro unit
} baro_data_t;

/* Magnetometer data */
typedef struct {
  timestamp_t ts;
  float magneto_x, magneto_y, magneto_z;  // Mag unit
} magneto_data_t;

/* Estimator Data */
typedef struct {
  float32_t acceleration_z;  // m/s^2
  float32_t height_AGL;      // m
} state_estimation_input_t;

/* Sensor Data */

typedef struct {
  float32_t x, y, z;  // m/s^2
} vec_t;

typedef struct {
  float32_t v;  // hPa
} scalar_t;

/* Todo: #if on SI data */
typedef struct {
  vec_t accel;        // m/s^2
  vec_t gyro;         // dps
  vec_t mag;          // mG
  scalar_t pressure;  // hPa
} SI_data_t;

/* Elimination Data */
typedef struct {
  int16_t freeze_counter_imu[NUM_IMU];
  int16_t freeze_counter_baro[NUM_BARO];
  int16_t freeze_counter_magneto[NUM_MAGNETO];
  int16_t freeze_counter_accel[NUM_ACCELEROMETER];
  int16_t last_value_imu[NUM_IMU];
  int32_t last_value_baro[NUM_BARO];
  float32_t last_value_magneto[NUM_MAGNETO];
  int8_t last_value_accel[NUM_ACCELEROMETER];
  uint8_t faulty_imu[NUM_IMU];
  uint8_t faulty_baro[NUM_BARO];
  uint8_t faulty_mag[NUM_MAGNETO];
  uint8_t faulty_acc[NUM_ACCELEROMETER];
} sensor_elimination_t;

typedef struct {
  float32_t acc[MEDIAN_FILTER_SIZE];         // m/s^2
  float32_t height_AGL[MEDIAN_FILTER_SIZE];  // m
  uint8_t counter;
} median_filter_t;

typedef struct {
  float32_t height;        // m
  float32_t velocity;      // m/s
  float32_t acceleration;  // m/s^2
} estimation_output_t;

typedef struct {
  float32_t angle;
  uint8_t axis;
} calibration_data_t;

typedef struct {
  float magneto_beta[3];
  float magneto_radius;
} magneto_calibration_data_t;

extern const char *flight_fsm_map[14];
extern const char *drop_test_fsm_map[7];

typedef enum {
  INVALID = 0,
  MOVING = 1,
  READY,
  THRUSTING_1,
  THRUSTING_2,
  COASTING,
  TRANSONIC_1,
  SUPERSONIC,
  TRANSONIC_2,
  APOGEE,
  DROGUE,
  MAIN,
  TOUCHDOWN,
  HEHE2 = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
} flight_fsm_e;

typedef struct {
  flight_fsm_e flight_state;
  vec_t old_acc_data;
  vec_t old_gyro_data;
  float old_height;
  float angular_movement[3];
  uint32_t clock_memory;
  uint32_t memory[3];
  bool state_changed;
} flight_fsm_t;

typedef enum {
  DT_INVALID = 0,
  DT_READY,
  DT_WAITING,
  DT_DROGUE,
  DT_MAIN,
  DT_TOUCHDOWN,
  HEHE3 = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
} drop_test_fsm_e;

typedef struct {
  drop_test_fsm_e flight_state;
  uint32_t timer_start_drogue;
  uint32_t timer_start_main;
  uint32_t memory;
  bool state_changed;
} drop_test_fsm_t;

typedef struct {
  uint8_t set_waiting;
  uint8_t set_drogue;
  uint8_t set_main;
} dt_telemetry_trigger_t;

typedef struct {
  float32_t Ad_data[9];
  float32_t Ad_T_data[9];
  float32_t Bd_data[3];
  float32_t GdQGd_T_data[9];
  float32_t H_data[3];
  float32_t H_T_data[3];
  float32_t K_data[3];
  float32_t x_bar_data[3];
  float32_t x_hat_data[3];
  float32_t P_bar_data[9];
  float32_t P_hat_data[9];
  arm_matrix_instance_f32 Ad;
  arm_matrix_instance_f32 Ad_T;
  arm_matrix_instance_f32 GdQGd_T;
  arm_matrix_instance_f32 Bd;
  arm_matrix_instance_f32 H;
  arm_matrix_instance_f32 H_T;
  arm_matrix_instance_f32 K;
  arm_matrix_instance_f32 x_bar;
  arm_matrix_instance_f32 x_hat;
  arm_matrix_instance_f32 P_bar;
  arm_matrix_instance_f32 P_hat;
  float32_t measured_acceleration;
  float32_t measured_AGL;
  float32_t R;
  float32_t t_sampl;
} kalman_filter_t;

typedef struct {
  uint16_t liftoff_acc_threshold;
  uint16_t mach_timer_duration;
  uint16_t main_altitude;
} control_settings_t;

typedef struct {
  uint32_t duration;
  uint8_t start_event;
  uint8_t end_event;
} config_timer_t;

typedef struct {
  int16_t action_idx;
  int16_t arg;
} config_action_t;

typedef bool (*peripheral_act_fp)(int16_t);

typedef struct {
  /* Action function pointer */
  peripheral_act_fp func_ptr;
  /* Action function argument */
  int16_t func_arg;
} peripheral_act_t;

typedef struct {
  /* Number of actions tied to an event */
  uint8_t num_actions;
  /* List of actions tied to an event */
  peripheral_act_t *action_list;
} event_action_map_elem_t;

typedef enum {
  EV_MOVING = 0,
  EV_READY,
  EV_LIFTOFF,
  EV_MAX_V,
  EV_APOGEE,
  EV_POST_APOGEE,
  EV_TOUCHDOWN,
  EV_CUSTOM_1,
  EV_CUSTOM_2,
  EV_MACHTIMER,
  EV_HEHE = 0xFFFFFFFF /* TODO <- optimize these enums and remove this guy */
} cats_event_e;

typedef enum { REC_OFF = 0, REC_FILL_QUEUE, REC_WRITE_TO_FLASH } recorder_status_e;

typedef struct {
  cats_event_e timer_init_event;
  cats_event_e execute_event;
  osTimerId_t timer_id;
  uint32_t timer_duration_ticks;
} cats_timer_t;

/** CONVERSION FUNCTIONS **/

inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low) { return (src_high << 8 | src_low); }

/* TODO: is this really the same? It's taken from the macros.. */
inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low) { return (int16_t)uint8_to_uint16(src_high, src_low); }
