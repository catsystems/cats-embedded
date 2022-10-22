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

#include "control/flight_phases.h"
#include "config/cats_config.h"
#include "tasks/task_peripherals.h"

static void check_moving_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data,
                               bool ready_transition_allowed);
static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, float32_t height_AGL,
                              bool ready_transition_allowed, const control_settings_t *settings);
static void check_thrusting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);

static void clear_fsm_memory(flight_fsm_t *fsm_state);

void check_flight_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, estimation_output_t *state_data,
                        float32_t height_AGL, bool ready_transition_allowed, const control_settings_t *settings) {
  /* Save old FSM State */
  flight_fsm_t old_fsm_state = *fsm_state;

  /* Check FSM State */
  switch (fsm_state->flight_state) {
    case MOVING:
      check_moving_phase(fsm_state, acc_data, gyro_data, ready_transition_allowed);
      break;
    case READY:
      check_ready_phase(fsm_state, acc_data, gyro_data, height_AGL, ready_transition_allowed, settings);
      break;
    case THRUSTING:
      check_thrusting_phase(fsm_state, state_data);
      break;
    case COASTING:
      check_coasting_phase(fsm_state, state_data);
      break;
    case DROGUE:
      check_drogue_phase(fsm_state, state_data);
      break;
    case MAIN:
      check_main_phase(fsm_state, state_data);
    case TOUCHDOWN:
      break;
    default:
      break;
  }

  if (old_fsm_state.flight_state != fsm_state->flight_state) {
    fsm_state->state_changed = true;
  } else {
    fsm_state->state_changed = false;
  }
}

static void check_moving_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data,
                               bool ready_transition_allowed) {

  /* Check if the IMU moved between two timesteps */
  /* Add an error bound as the IMU is noisy which is accepted */
  if ((fabsf(fsm_state->old_acc_data.x - acc_data->x) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_acc_data.y - acc_data->y) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_acc_data.z - acc_data->z) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.x - gyro_data->x) < ALLOWED_GYRO_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.y - gyro_data->y) < ALLOWED_GYRO_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.z - gyro_data->z) < ALLOWED_GYRO_ERROR)) {
    fsm_state->memory[0]++;
  } else {
    fsm_state->memory[0] = 0;
  }

  /* Update old IMU value */
  fsm_state->old_acc_data = *acc_data;
  fsm_state->old_gyro_data = *gyro_data;

  /* Check if we reached the threshold */
  if ((fsm_state->memory[0] > TIME_THRESHOLD_MOV_TO_READY) && ready_transition_allowed) {
    trigger_event(EV_READY);
    fsm_state->flight_state = READY;
    clear_fsm_memory(fsm_state);
  }
}

static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, float32_t height_AGL,
                              bool ready_transition_allowed, const control_settings_t *settings) {
  /* Check if we move from READY Back to MOVING */

  /* Check if the IMU moved between two timesteps */
  if ((fabsf(fsm_state->old_acc_data.x - acc_data->x) > ALLOWED_ACC_ERROR) ||
      (fabsf(fsm_state->old_acc_data.y - acc_data->y) > ALLOWED_ACC_ERROR) ||
      (fabsf(fsm_state->old_acc_data.z - acc_data->z) > ALLOWED_ACC_ERROR) ||
      (fabsf(fsm_state->old_gyro_data.x - gyro_data->x) > ALLOWED_GYRO_ERROR) ||
      (fabsf(fsm_state->old_gyro_data.y - gyro_data->y) > ALLOWED_GYRO_ERROR) ||
      (fabsf(fsm_state->old_gyro_data.z - gyro_data->z) > ALLOWED_GYRO_ERROR)) {
    fsm_state->memory[0]++;
  }

  /* Update Time */
  fsm_state->clock_memory += 1;

  /* Half of the samples have to be over the specified threshold to detect
   * movement */
  /* Periodically reset the counter */
  if (fsm_state->clock_memory > 2 * TIME_THRESHOLD_READY_TO_MOV) {
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Update old IMU value */
  fsm_state->old_acc_data = *acc_data;
  fsm_state->old_gyro_data = *gyro_data;

  /* Check if we reached the threshold */
  if (fsm_state->memory[0] > TIME_THRESHOLD_READY_TO_MOV) {
    trigger_event(EV_MOVING);
    fsm_state->flight_state = MOVING;
    clear_fsm_memory(fsm_state);
  }

  /* Integrate Gyro Movement, but only if the value is big enough */
  if (fabsf(gyro_data->x) > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[0] += fabsf(gyro_data->x) / CONTROL_SAMPLING_FREQ;
  }

  if (fabsf(gyro_data->y) > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[1] += fabsf(gyro_data->y) / CONTROL_SAMPLING_FREQ;
  }

  if (fabsf(gyro_data->z) > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[2] += fabsf(gyro_data->z) / CONTROL_SAMPLING_FREQ;
  }

  /* If the total angle is larger than the threshold, move back to moving */
  if ((fabsf(fsm_state->angular_movement[0]) + fabsf(fsm_state->angular_movement[1]) +
       fabsf(fsm_state->angular_movement[2])) > ANGLE_MOVE_MAX) {
    trigger_event(EV_MOVING);
    fsm_state->flight_state = MOVING;
    clear_fsm_memory(fsm_state);
  }

  /* Check if we move from READY To THRUSTING */
  /* The absolut value of the acceleration is used here to make sure that we detect liftoff */
  float32_t accel_x = acc_data->x * acc_data->x;
  float32_t accel_y = acc_data->y * acc_data->y;
  float32_t accel_z = acc_data->z * acc_data->z;
  float32_t acceleration = accel_x + accel_y + accel_z;

  if (acceleration > ((float)settings->liftoff_acc_threshold * (float)settings->liftoff_acc_threshold)) {
    fsm_state->memory[1]++;
  } else {
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > LIFTOFF_SAFETY_COUNTER) {
    trigger_event(EV_LIFTOFF);
    fsm_state->flight_state = THRUSTING;
    clear_fsm_memory(fsm_state);
  }

  /* Check if we move from Ready to Thrusting based on baro data */
  if (height_AGL > LIFTOFF_HEIGHT_AGL) {
    fsm_state->memory[2]++;
  } else {
    fsm_state->memory[2] = 0;
  }

  /* If the above condition is correct for some samples, detect liftoff */
  if (fsm_state->memory[2] > LIFTOFF_SAFETY_COUNTER_HEIGHT) {
    trigger_event(EV_LIFTOFF);
    fsm_state->flight_state = THRUSTING;
    clear_fsm_memory(fsm_state);
  }

  /* Check if we go back to moving due to telemetry */
  if (!ready_transition_allowed) {
    trigger_event(EV_MOVING);
    fsm_state->flight_state = MOVING;
    clear_fsm_memory(fsm_state);
  }
}

static void check_thrusting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {

  /* When acceleration is below 0, liftoff concludes */
  if (state_data->acceleration < 0) {
    fsm_state->memory[0]++;
  } else {
    fsm_state->memory[0] = 0;
  }

  if (fsm_state->memory[0] > COASTING_SAFETY_COUNTER) {
    trigger_event(EV_MAX_V);
    fsm_state->flight_state = COASTING;
    clear_fsm_memory(fsm_state);
  }
}

static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {

  /* When velocity is below 0, coasting concludes */
  if (state_data->velocity < 0) {
    fsm_state->memory[0]++;
  }

  if (fsm_state->memory[0] > APOGEE_SAFETY_COUNTER) {
    trigger_event(EV_APOGEE);
    fsm_state->flight_state = DROGUE;
    clear_fsm_memory(fsm_state);
  }
}

static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {

  /* If the height is smaller than the configured Main height, main deployment needs to be actuated */
  if (state_data->height < (float)global_cats_config.config.control_settings.main_altitude) {
    /* Achieved Height to deploy Main */
    fsm_state->memory[0]++;
  } else {
    /* Did Not Achieve */
    fsm_state->memory[0] = 0;
  }

  if (fsm_state->memory[1] > MAIN_SAFETY_COUNTER) {
    trigger_event(EV_MAIN);
    fsm_state->flight_state = MAIN;
    clear_fsm_memory(fsm_state);
  }
}

static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {

  /* If the velocity is very small we have touchdown */
  if (fabsf(state_data->velocity) < VELOCITY_BOUND_TOUCHDOWN) {
    /* Touchdown achieved */
    fsm_state->memory[0]++;
  } else {
    /* Touchdown not achieved */
    fsm_state->memory[0] = 0;
  }

  if (fsm_state->memory[0] > TOUCHDOWN_SAFETY_COUNTER) {
    trigger_event(EV_TOUCHDOWN);
    fsm_state->flight_state = TOUCHDOWN;
    clear_fsm_memory(fsm_state);
  }
}

/* Function which needs to be called, every time that a state transition is done */
static void clear_fsm_memory(flight_fsm_t *fsm_state) {
  fsm_state->clock_memory = 0;
  fsm_state->memory[0] = 0;
  fsm_state->memory[1] = 0;
  fsm_state->memory[2] = 0;
  fsm_state->angular_movement[0] = 0;
  fsm_state->angular_movement[1] = 0;
  fsm_state->angular_movement[2] = 0;
}