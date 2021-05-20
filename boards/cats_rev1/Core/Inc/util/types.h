//
// Created by stoja on 23.12.20.
//

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "../DSP/Inc/arm_math.h"
#include "cmsis_os2.h"

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

/* Magnetometer data */
typedef struct {
    timestamp_t ts;
    float magneto_x, magneto_y, magneto_z;
}magneto_data_t;

/* Estimator Data */
typedef struct {
  float pressure[3];
  float temperature[3];
  float acceleration[3];
  float calculated_AGL[3];
  timestamp_t ts;
} state_estimation_data_t;

/* Elimination Data */
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
    float data[6][10];
    float estimate[6];
    uint8_t counter;
} median_filter_t;

typedef struct {
  float height;
  float velocity;
  float acceleration;
} estimation_output_t;

typedef struct {
  float angle;
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
  IDLE,
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
  imu_data_t old_imu_data;
  float old_height;
  float angular_movement[3];
  uint32_t clock_memory;
  uint32_t memory[3];
  uint8_t state_changed;
} flight_fsm_t;

typedef enum {
  DT_INVALID = 0,
  DT_IDLE,
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
  uint8_t state_changed;
} drop_test_fsm_t;

typedef struct {
  uint8_t set_waiting;
  uint8_t set_drogue;
  uint8_t set_main;
} dt_telemetry_trigger_t;

typedef struct {
  float32_t Ad_data[9];
  float32_t Ad_T_data[9];
  float32_t Gd_data[6];
  float32_t Bd_data[3];
  float32_t Q_data[4];
  float32_t GdQGd_T_data[9];
  float32_t H_full_data[9];
  float32_t H_full_T_data[9];
  float32_t H_eliminated_data[6];
  float32_t H_eliminated_T_data[6];
  float32_t R_full_data[9];
  float32_t R_eliminated_data[4];
  float32_t K_full_data[9];
  float32_t K_eliminated_data[6];
  float32_t x_bar_data[3];
  float32_t P_bar_data[9];
  float32_t x_hat_data[3];
  float32_t P_hat_data[9];
  arm_matrix_instance_f32 Ad;
  arm_matrix_instance_f32 Ad_T;
  arm_matrix_instance_f32 Gd;
  arm_matrix_instance_f32 Bd;
  arm_matrix_instance_f32 Q;
  arm_matrix_instance_f32 GdQGd_T;
  arm_matrix_instance_f32 H_full;
  arm_matrix_instance_f32 H_full_T;
  arm_matrix_instance_f32 H_eliminated;
  arm_matrix_instance_f32 H_eliminated_T;
  arm_matrix_instance_f32 R_full;
  arm_matrix_instance_f32 R_eliminated;
  arm_matrix_instance_f32 K_full;
  arm_matrix_instance_f32 K_eliminated;
  arm_matrix_instance_f32 x_bar;
  arm_matrix_instance_f32 P_bar;
  arm_matrix_instance_f32 x_hat;
  arm_matrix_instance_f32 P_hat;
  float pressure_0;
  float t_sampl;
} kalman_filter_t;

typedef struct {
  float liftoff_acc_threshold;
  float apogee_timer;
  float second_stage_timer;
} control_settings_t;

typedef enum {
  CATS_BUZZ_NONE = 0,
  CATS_BUZZ_BOOTUP,
  CATS_BUZZ_READY,
  CATS_BUZZ_CHANGED_MOVING,
  CATS_BUZZ_CHANGED_READY,
} buzzer_status_e;

typedef bool (*peripheral_act_fp)(int32_t);

typedef struct {
  /* Action function pointer */
  peripheral_act_fp func_ptr;
  /* Action function argument */
  int32_t func_arg;
} peripheral_act_t;

typedef struct {
  /* Number of actions tied to an event */
  uint8_t num_actions;
  /* List of actions tied to an event */
  peripheral_act_t *action_list;
} event_action_map_elem_t;

typedef enum {
  EV_IDLE = 1,
  EV_MOVING,
  EV_LIFTOFF,
  EV_MAX_V,
  EV_APOGEE,
  EV_POST_APOGEE,
  EV_TOUCHDOWN,
  EV_TIMER_1,
  EV_TIMER_2,
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
inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low) { return uint8_to_uint16(src_high, src_low); }
