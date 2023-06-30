/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include <cstdint>
#include "target.h"

#include "arm_math.h"
#include "cmsis_os2.h"
#include "config/control_config.hpp"
#include "util/actions.hpp"

/** DEFINES **/

#define NUM_EVENTS 9
#define NUM_TIMERS 4

/** BASIC TYPES **/

/* Timestamp */
using timestamp_t = uint32_t;  // ms

/* 3D vector types */
struct vi8_t {
  int8_t x, y, z;
};

struct vi16_t {
  int16_t x, y, z;
};

struct vf32_t {
  float32_t x, y, z;
};

/** SENSOR DATA TYPES **/

/* IMU data */
struct imu_data_t {
  vi16_t acc;   // IMU unit
  vi16_t gyro;  // IMU unit
};

/* Barometer data */
struct baro_data_t {
  int32_t pressure;     // Baro unit
  int32_t temperature;  // Baro unit
};

/* Estimator Data */
struct state_estimation_input_t {
  float32_t acceleration_z;  // m/s^2
  float32_t height_AGL;      // m
};

/* Todo: #if on SI data */
struct SI_data_t {
  vf32_t acc;          // m/s^2
  vf32_t gyro;         // dps
  float32_t pressure;  // hPa
};

/* Elimination Data */
struct sensor_elimination_t {
  int16_t freeze_counter_imu[NUM_IMU];
  int16_t freeze_counter_baro[NUM_BARO];
  int16_t last_value_imu[NUM_IMU];
  int32_t last_value_baro[NUM_BARO];
  uint8_t faulty_imu[NUM_IMU];
  uint8_t faulty_baro[NUM_BARO];
};

struct median_filter_t {
  float32_t acc[MEDIAN_FILTER_SIZE];         // m/s^2
  float32_t height_AGL[MEDIAN_FILTER_SIZE];  // m
  uint8_t counter;
};

struct estimation_output_t {
  float32_t height;        // m
  float32_t velocity;      // m/s
  float32_t acceleration;  // m/s^2
};

struct calibration_data_t {
  vf32_t gyro_calib;
  float32_t angle;
  uint8_t axis;
};

enum flight_fsm_e : uint32_t { INVALID = 0, CALIBRATING = 1, READY, THRUSTING, COASTING, DROGUE, MAIN, TOUCHDOWN };

/* Todo: Comment out this struct */
struct flight_fsm_t {
  flight_fsm_e flight_state;
  vf32_t old_acc_data;
  vf32_t old_gyro_data;
  uint32_t clock_memory;
  uint32_t memory[3];
  timestamp_t thrust_trigger_time;
  bool state_changed;
};

struct kalman_filter_t {
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
};

struct control_settings_t {
  uint16_t liftoff_acc_threshold;  // m/s^2
  uint16_t main_altitude;          // m
};

struct config_timer_t {
  uint32_t duration;
  /* Event on which the timer starts. */
  uint8_t start_event;
  /* Event that the timer triggers. */
  uint8_t trigger_event;
};

struct config_action_t {
  int16_t action_idx;
  int16_t arg;
};

enum adaptive_power_e { OFF, ON };

constexpr int kMinConnPhraseChars = 4;
constexpr int kMaxConnPhraseChars = 16;

struct config_telemetry_t {
  /* +1 for null terminator */
  char link_phrase[kMaxConnPhraseChars + 1]{};
  char test_phrase[kMaxConnPhraseChars + 1]{};
  uint8_t power_level;
  bool enable_telemetry;
  adaptive_power_e adaptive_power;
};

struct peripheral_act_t {
  /* Action type */
  action_function_e action;
  /* Action argument */
  int16_t action_arg;
};

struct event_action_map_elem_t {
  /* Number of actions tied to an event */
  uint8_t num_actions;
  /* List of actions tied to an event */
  peripheral_act_t *action_list;
};

enum cats_event_e : uint32_t {
  EV_CALIBRATE = 0,
  EV_READY,
  EV_LIFTOFF,
  EV_MAX_V,
  EV_APOGEE,
  EV_MAIN_DEPLOYMENT,
  EV_TOUCHDOWN,
  EV_CUSTOM_1,
  EV_CUSTOM_2
};

enum cats_sim_choice_e { SIM_INVALID = 0, SIM_HOP, SIM_300M, SIM_PML };

struct cats_sim_config_t {
  uint32_t noise_seed;
  int32_t sim_axis;
  int32_t simulation_option;
};

enum recorder_status_e { REC_OFF = 0, REC_FILL_QUEUE, REC_WRITE_TO_FLASH };

struct cats_timer_t {
  cats_event_e timer_init_event;
  cats_event_e execute_event;
  osTimerId_t timer_id;
  uint32_t timer_duration_ticks;
};

enum battery_type_e : uint8_t { LI_ION = 0, LI_PO, ALKALINE };

/** CONVERSION FUNCTIONS **/

inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low) { return (src_high << 8 | src_low); }

/* TODO: is this really the same? It's taken from the macros.. */
inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low) { return (int16_t)uint8_to_uint16(src_high, src_low); }
