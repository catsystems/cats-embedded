//
// Created by stoja on 23.12.20.
//

#ifndef CATS_REV1_TYPES_H
#define CATS_REV1_TYPES_H

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

/* Magnetometer Data */
typedef struct {
  timestamp_t ts;
  float magneto_x, magneto_y, magneto_z;
} magneto_data_t;

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
  float height;
  float velocity;
  float acceleration;
} estimation_output_t;

typedef struct {
  float angle;
  uint8_t axis;
} calibration_data_t;

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
  float measured_height[10][3];
  float measured_acc[10][3];
  float height_avg;
  float acc_avg;
  float height_noise_est;
  float acc_noise_est;
  /* future use */
  float height_poly[3];
  float accel_poly[3];
} noise_estimator_t;

typedef enum {
  SERVO_1_TRIGGER = 0,
  SERVO_2_TRIGGER,
  SERVO_1_2_TRIGGER,
  PYRO_1_TRIGGER,
  PYRO_2_TRIGGER,
  PYRO_3_TRIGGER,
  ALL_PYROS_TRIGGER
} chute_trigger_e;

typedef struct {
  uint8_t stages;
  chute_trigger_e stage_type_1;
  int32_t servo_angle_1;
  chute_trigger_e stage_type_2;
  int32_t servo_angle_2;
} chute_type_t;

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

typedef enum { CATS_OK = 0, CATS_BARO_ERROR, CATS_IMU_ERROR, CATS_FILTER_ERROR, CATS_HARD_FAULT } cats_status_e;

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

typedef enum {
  CATS_STATUS_NONE = 0,
  CATS_STATUS_BOOTUP,
  CATS_STATUS_READY,
  CATS_STATUS_CHANGED_MOVING,
  CATS_STATUS_CHANGED_READY,
} buzzer_status_e;

typedef bool (*peripheral_out_fp)(int16_t);

typedef struct {
  /* Output function pointer */
  peripheral_out_fp func_ptr;
  /* Output function argument */
  int16_t func_arg;
  /* Time to wait before executing the next output */
  uint32_t delay_ms;
} peripheral_out_t;

typedef enum {
  EV_IDLE = 1,
  EV_MOVING,
  EV_LIFTOFF,
  EV_MAX_V,
  EV_APOGEE,
  EV_POST_APOGEE,
  EV_TOUCHDOWN,
  EV_HEHE = 0xFFFFFFFF /* TODO <- optimize these enums and remove this guy */
} cats_event_e;

typedef enum { REC_OFF = 0, REC_FILL_QUEUE, REC_WRITE_TO_FLASH } recorder_status_e;

typedef struct {
  cats_event_e timer_init_event;
  cats_event_e execute_event;
  osTimerId_t timer_id;
  uint32_t timer_duration;
} cats_timer_t;

typedef struct {
  /* Number of outputs tied to an event */
  uint8_t num_outputs;
  /* List of outputs tied to an event */
  peripheral_out_t *output_list;
} event_output_map_elem_t;

/** CONVERSION FUNCTIONS **/

inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low) { return (src_high << 8 | src_low); }

/* TODO: is this really the same? It's taken from the macros.. */
inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low) { return uint8_to_uint16(src_high, src_low); }

#endif  // CATS_REV1_TYPES_H
