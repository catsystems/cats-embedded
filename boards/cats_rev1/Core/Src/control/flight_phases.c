/*
 * flight_phases.c
 *
 *  Created on: Dec 21, 2020
 *      Author: Jonas
 */

#include "control/flight_phases.h"
#include <stdlib.h>
#include <math.h>
#include "config/globals.h"

void check_moving_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data);
void check_idle_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data);
void check_thrusting_1_phase(flight_fsm_t *fsm_state,
                             estimation_output_t *state_data);
void check_coasting_phase(flight_fsm_t *fsm_state,
                          estimation_output_t *state_data);
void check_apogee_phase(flight_fsm_t *fsm_state,
                        estimation_output_t *state_data);
void check_descent_phase(flight_fsm_t *fsm_state,
                         estimation_output_t *state_data);

void check_flight_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data,
                        estimation_output_t *state_data) {
  fsm_state->state_changed = 0;
  switch (fsm_state->flight_state) {
    case MOVING:
      check_moving_phase(fsm_state, imu_data);
      break;
    case IDLE:
      check_idle_phase(fsm_state, imu_data);
      break;
    case THRUSTING_1:
      check_thrusting_1_phase(fsm_state, state_data);
      break;
    case THRUSTING_2:
      break;
    case COASTING:
      check_coasting_phase(fsm_state, state_data);
      break;
    case TRANSONIC_1:
      break;
    case TRANSONIC_2:
      break;
    case APOGEE:
      check_apogee_phase(fsm_state, state_data);
      break;
    case PARACHUTE:
      check_descent_phase(fsm_state, state_data);
      break;
    case BALLISTIC:
      check_descent_phase(fsm_state, state_data);
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
    fsm_state->memory[1]++;
  } else {
    fsm_state->memory[1] = 0;
  }

  /* Update old IMU value */
  fsm_state->old_imu_data = *imu_data;

  fsm_state->state_changed = 0;

  /* Check if we reached the threshold */
  if (fsm_state->memory[1] > TIME_THRESHOLD_MOV_TO_IDLE) {
    fsm_state->flight_state = IDLE;
    fsm_state->state_changed = 1;
    fsm_state->memory[1] = 0;
  }
}

void check_idle_phase(flight_fsm_t *fsm_state, imu_data_t *imu_data) {
  /* Check if we move from IDLE Back to MOVING */

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
    fsm_state->memory[1]++;
  }

  /* Update Time */
  fsm_state->clock_memory += 1;

  /* Half of the samples have to be over the specified threshold to detect
   * movement */
  if (fsm_state->clock_memory > 2 * TIME_THRESHOLD_IDLE_TO_MOV) {
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Update old IMU value */
  fsm_state->old_imu_data = *imu_data;

  fsm_state->state_changed = 0;

  /* Check if we reached the threshold */
  if (fsm_state->memory[1] > TIME_THRESHOLD_IDLE_TO_MOV) {
    fsm_state->flight_state = MOVING;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Integrate Gyro Movement */
  /* Calculate Angle Movement in all directions */
  float angle_movement = (float)(imu_data->gyro_x) / 16.4f;
  if (angle_movement > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[0] += angle_movement / SAMPLING_FREQUENCY;
  }
  angle_movement = (float)(imu_data->gyro_y) / 16.4f;
  if (angle_movement > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[1] += angle_movement / SAMPLING_FREQUENCY;
  }
  angle_movement = (float)(imu_data->gyro_z) / 16.4f;
  if (angle_movement > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[2] += angle_movement / SAMPLING_FREQUENCY;
  }

  if ((abs(fsm_state->angular_movement[0]) +
       abs(fsm_state->angular_movement[1]) +
       abs(fsm_state->angular_movement[2])) > ANGLE_MOVE_MAX) {
    fsm_state->flight_state = MOVING;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Check if we move from IDLE To THRUSTING_1 */
  /* To Make sure that the timers start any acceleration direction is accepted
   * here */
  int32_t acceleration = imu_data->acc_x * imu_data->acc_x +
                         imu_data->acc_y * imu_data->acc_y + imu_data->acc_z +
                         imu_data->acc_z;

  if (acceleration > LIFTOFF_ACC_THRESHOLD_SQUARED) {
    fsm_state->memory[2]++;
  } else {
    fsm_state->memory[2] = 0;
  }

  if (fsm_state->memory[2] > LIFTOFF_SAFETY_COUNTER) {
    fsm_state->flight_state = THRUSTING_1;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }
}

void check_thrusting_1_phase(flight_fsm_t *fsm_state,
                             estimation_output_t *state_data) {
  if (state_data->acceleration < 0) {
    fsm_state->memory[1]++;
  } else {
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > COASTING_SAFETY_COUNTER) {
    fsm_state->flight_state = COASTING;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

void check_coasting_phase(flight_fsm_t *fsm_state,
                          estimation_output_t *state_data) {
  if (state_data->velocity < 0) {
    fsm_state->memory[1]++;
  }

  if (fsm_state->memory[1] > APOGEE_SAFETY_COUNTER) {
    fsm_state->flight_state = APOGEE;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

void check_apogee_phase(flight_fsm_t *fsm_state,
                        estimation_output_t *state_data) {
  if (state_data->velocity > PARACHUTE_DESCENT_SPEED) {
    /* Parachute Deployed */
    fsm_state->memory[1]++;
  } else {
    /* Parachute Not deployed */
    fsm_state->memory[2]++;
  }

  if (fsm_state->memory[1] > PARACHUTE_SAFETY_COUNTER) {
    fsm_state->flight_state = PARACHUTE;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }

  if (fsm_state->memory[2] > BALISTIC_SAFETY_COUNTER) {
    fsm_state->flight_state = BALLISTIC;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

void check_descent_phase(flight_fsm_t *fsm_state,
                         estimation_output_t *state_data) {
  /* If the position doesnt change we have touchdown */
  float error = abs(state_data->height - fsm_state->old_height);
  if (error < HEIGHT_ERROR_BOUND) {
    /* Parachute Deployed */
    fsm_state->memory[1]++;
  } else {
    /* Parachute Not deployed */
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > TOUCHDOWN_SAFETY_COUNTER) {
    fsm_state->flight_state = TOUCHDOWN;
    fsm_state->state_changed = 1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}
