/*
 * flight_phases.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "control/flight_phases.h"
#include <stdlib.h>

void check_moving_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data);
void check_idle_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data);

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data) {
  switch (fsm_state->flight_state) {
    case MOVING:
      check_moving_phase(fsm_state, imu_data);
      break;
    case IDLE:
      check_idle_phase(fsm_state, imu_data);
      break;
    case THRUSTING_1:
      break;
    case THRUSTING_2:
      break;
    case COASTING:
      break;
    case TRANSSONIC_1:
      break;
    case TRANSSONIC_2:
      break;
    case APOGEE:
      break;
    case PARACHUTE:
      break;
    case TOUCHDOWN:
      break;
    default:
      break;
  }
}

void check_moving_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data) {
  /* Check if the IMU moved between two timesteps */
  if ((abs(fsm_state->old_imu_data.acc_x - imu_data->acc_x) <
       ALLOWED_ACC_ERROR) &&
      (abs(fsm_state->old_imu_data.acc_y - imu_data->acc_y) <
       ALLOWED_ACC_ERROR) &&
      (abs(fsm_state->old_imu_data.acc_z - imu_data->acc_z) <
       ALLOWED_ACC_ERROR) &&
      (abs(fsm_state->old_imu_data.gyro_x - imu_data->gyro_x) <
       ALLOWED_GYRO_ERROR) &&
      (abs(fsm_state->old_imu_data.gyro_y - imu_data->gyro_y) <
       ALLOWED_GYRO_ERROR) &&
      (abs(fsm_state->old_imu_data.gyro_z - imu_data->gyro_z) <
       ALLOWED_GYRO_ERROR)) {
    fsm_state->memory++;
  } else {
    fsm_state->memory = 0;
  }

  /* Update old IMU value */
  fsm_state->old_imu_data = *imu_data;

  fsm_state->state_changed = 0;

  /* Check if we reached the threshold */
  if (fsm_state->memory > TIME_THRESHOLD_MOV_TO_IDLE) {
    fsm_state->flight_state = IDLE;
    fsm_state->state_changed = 1;
    fsm_state->memory = 0;
  }
}

void check_idle_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data) {
  /* Check if the IMU moved between two timesteps */
  if ((abs(fsm_state->old_imu_data.acc_x - imu_data->acc_x) >
       ALLOWED_ACC_ERROR) ||
      (abs(fsm_state->old_imu_data.acc_y - imu_data->acc_y) >
       ALLOWED_ACC_ERROR) ||
      (abs(fsm_state->old_imu_data.acc_z - imu_data->acc_z) >
       ALLOWED_ACC_ERROR) ||
      (abs(fsm_state->old_imu_data.gyro_x - imu_data->gyro_x) >
       ALLOWED_GYRO_ERROR) ||
      (abs(fsm_state->old_imu_data.gyro_y - imu_data->gyro_y) >
       ALLOWED_GYRO_ERROR) ||
      (abs(fsm_state->old_imu_data.gyro_z - imu_data->gyro_z) >
       ALLOWED_GYRO_ERROR)) {
    fsm_state->memory++;
  }

  /* Update Time */
  fsm_state->clock_memory += 1;

  /* Half of the samples have to be over the specified threshold to detect
   * movement */
  if (fsm_state->clock_memory > 2 * TIME_THRESHOLD_IDLE_TO_MOV) {
    fsm_state->clock_memory = 0;
    fsm_state->memory = 0;
  }

  /* Update old IMU value */
  fsm_state->old_imu_data = *imu_data;

  fsm_state->state_changed = 0;

  /* Check if we reached the threshold */
  if (fsm_state->memory > TIME_THRESHOLD_IDLE_TO_MOV) {
    fsm_state->flight_state = MOVING;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory = 0;
  }
}
