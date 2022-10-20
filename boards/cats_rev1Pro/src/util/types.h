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

#include <stdbool.h>
#include <stdint.h>
#include "target.h"

#include "arm_math.h"
#include "cmsis_os2.h"
#include "config/control_config.h"

/** DEFINES **/

#define NUM_EVENTS 9
#define NUM_TIMERS 4

/** BASIC TYPES **/

/* Timestamp */
typedef uint32_t timestamp_t;  // ms

/* 3D vector types */
typedef struct {
  int8_t x, y, z;
} vi8_t;

typedef struct {
  int16_t x, y, z;
} vi16_t;

typedef struct {
  float32_t x, y, z;
} vf32_t;

/** SENSOR DATA TYPES **/

/* Accelerometer data */
typedef vi8_t accel_data_t;  // Accelerometer units

/* IMU data */
typedef struct {
  vi16_t acc;   // IMU unit
  vi16_t gyro;  // IMU unit
} imu_data_t;

/* Magnetometer data */
typedef vf32_t magneto_data_t;  // Magnetometer units

/* Barometer data */
typedef struct {
  int32_t pressure;     // Baro unit
  int32_t temperature;  // Baro unit
} baro_data_t;

/* Estimator Data */
typedef struct {
  float32_t acceleration_z;  // m/s^2
  float32_t height_AGL;      // m
} state_estimation_input_t;

/* Todo: #if on SI data */
typedef struct {
  vf32_t acc;          // m/s^2
  vf32_t gyro;         // dps
  vf32_t mag;          // mG
  float32_t pressure;  // hPa
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
  vf32_t gyro_calib;
  float32_t angle;
  uint8_t axis;
} calibration_data_t;

typedef struct {
  float magneto_beta[3];
  float magneto_radius;
} magneto_calibration_data_t;

typedef enum {
  INVALID = 0,
  MOVING = 1,
  READY,
  THRUSTING,
  COASTING,
  DROGUE,
  MAIN,
  TOUCHDOWN,
  HEHE2 = 0x7FFFFFFF /* TODO <- optimize these enums and remove this guy */
} flight_fsm_e;

/* Todo: Comment out this struct */
typedef struct {
  flight_fsm_e flight_state;
  vf32_t old_acc_data;
  vf32_t old_gyro_data;
  float old_height;
  float angular_movement[3];
  uint32_t clock_memory;
  uint32_t memory[3];
  bool state_changed;
} flight_fsm_t;

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
  uint16_t liftoff_acc_threshold;  // m/s^2
  uint16_t main_altitude;          // m
} control_settings_t;

typedef struct {
  uint32_t duration;
  /* Event on which the timer starts. */
  uint8_t start_event;
  /* Event that the timer triggers. */
  uint8_t trigger_event;
} config_timer_t;

typedef struct {
  int16_t action_idx;
  int16_t arg;
} config_action_t;

#ifdef CATS_VEGA
typedef struct {
  uint8_t link_phrase[8];
  uint8_t power_level;
} config_telemetry_t;
#endif

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
  EV_MAIN,
  EV_TOUCHDOWN,
  EV_CUSTOM_1,
  EV_CUSTOM_2,
  EV_HEHE = 0xFFFFFFFF /* TODO <- optimize these enums and remove this guy */
} cats_event_e;

typedef enum { SIM_INVALID = 0, SIM_HOP, SIM_300M, SIM_PML } cats_sim_choice_e;

typedef struct {
  cats_sim_choice_e sim_choice;
  uint32_t noise_seed;
  int32_t sim_axis;
} cats_sim_config_t;

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
