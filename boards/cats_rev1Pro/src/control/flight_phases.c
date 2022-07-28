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

#include "config/cats_config.h"
#include "control/flight_phases.h"
#include "tasks/task_peripherals.h"
#include "util/log.h"

#include <stdlib.h>

static void check_moving_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, bool ready_transition_allowed);
static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, float32_t height_AGL,
                              const control_settings_t *settings);
static void check_thrusting_1_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_apogee_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);
static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data);

void check_flight_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, estimation_output_t *state_data, float32_t height_AGL, bool ready_transition_allowed,
                        const control_settings_t *settings) {
  /* Save old FSM state */
  flight_fsm_t old_fsm_state = *fsm_state;

  switch (fsm_state->flight_state) {
    case MOVING:
      check_moving_phase(fsm_state, acc_data, gyro_data, ready_transition_allowed);
      break;
    case READY:
      check_ready_phase(fsm_state, acc_data, gyro_data, height_AGL, settings);
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

static void check_moving_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, bool ready_transition_allowed) {
  /* Check if the IMU moved between two timesteps */
  if ((fabsf(fsm_state->old_acc_data.x - acc_data->x) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_acc_data.y - acc_data->y) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_acc_data.z - acc_data->z) < ALLOWED_ACC_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.x - gyro_data->x) < ALLOWED_GYRO_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.y - gyro_data->y) < ALLOWED_GYRO_ERROR) &&
      (fabsf(fsm_state->old_gyro_data.z - gyro_data->z) < ALLOWED_GYRO_ERROR)) {
    fsm_state->memory[1]++;
  } else {
    fsm_state->memory[1] = 0;
  }

  /* Update old IMU value */
  fsm_state->old_acc_data = *acc_data;
  fsm_state->old_gyro_data = *gyro_data;

  /* Check if we reached the threshold */
  if ((fsm_state->memory[1] > TIME_THRESHOLD_MOV_TO_READY) && ready_transition_allowed) {
    trigger_event(EV_READY);
    fsm_state->flight_state = READY;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Check if we move from MOVING to THRUSTING_1 */
  /* To make sure that the timers start any acceleration direction is accepted here */
  float32_t accel_x = acc_data->x * acc_data->x;
  float32_t accel_y = acc_data->y * acc_data->y;
  float32_t accel_z = acc_data->z * acc_data->z;
  float32_t acceleration = accel_x + accel_y + accel_z;

  if (acceleration > (MOV_LIFTOFF_THRESHOLD * MOV_LIFTOFF_THRESHOLD)) {
    fsm_state->memory[2]++;
  } else {
    fsm_state->memory[2] = 0;
  }

  if (fsm_state->memory[2] > MOV_LIFTOFF_SAFETY_COUNTER) {
    trigger_event(EV_LIFTOFF);
    fsm_state->flight_state = THRUSTING_1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }
}

static void check_ready_phase(flight_fsm_t *fsm_state, vf32_t *acc_data, vf32_t *gyro_data, float32_t height_AGL,
                              const control_settings_t *settings) {
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
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Integrate Gyro Movement */
  if (gyro_data->x > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[0] += gyro_data->x / SAMPLING_FREQUENCY;
  }

  if (gyro_data->y > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[1] += gyro_data->y / SAMPLING_FREQUENCY;
  }

  if (gyro_data->z > GYRO_SENSITIVITY) {
    fsm_state->angular_movement[2] += gyro_data->z / SAMPLING_FREQUENCY;
  }

  if ((fabsf(fsm_state->angular_movement[0]) + fabsf(fsm_state->angular_movement[1]) +
       fabsf(fsm_state->angular_movement[2])) > ANGLE_MOVE_MAX) {
    trigger_event(EV_MOVING);
    fsm_state->flight_state = MOVING;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Check if we move from READY To THRUSTING_1 */
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
    fsm_state->flight_state = THRUSTING_1;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }

  /* Check if we move from Ready to Thrusting_1 based on baro data */
    if (height_AGL > LIFTOFF_HEIGHT_AGL) {
        fsm_state->memory[2]++;
    } else {
        fsm_state->memory[2] = 0;
    }

    if (fsm_state->memory[2] > LIFTOFF_SAFETY_COUNTER_HEIGHT) {
        trigger_event(EV_LIFTOFF);
        fsm_state->flight_state = THRUSTING_1;
        fsm_state->clock_memory = 0;
        fsm_state->memory[0] = 0;
        fsm_state->memory[1] = 0;
        fsm_state->memory[2] = 0;
        fsm_state->angular_movement[0] = 0;
        fsm_state->angular_movement[1] = 0;
        fsm_state->angular_movement[2] = 0;
    }
}

static void check_thrusting_1_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {
  if (state_data->acceleration < 0) {
    fsm_state->memory[1]++;
  } else {
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > COASTING_SAFETY_COUNTER) {
    trigger_event(EV_MAX_V);
    fsm_state->flight_state = COASTING;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
    fsm_state->angular_movement[0] = 0;
    fsm_state->angular_movement[1] = 0;
    fsm_state->angular_movement[2] = 0;
  }
}

static void check_coasting_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {
  if (osTimerIsRunning(mach_timer.timer_id)) {
    return;
  }

  if (state_data->velocity < 0) {
    fsm_state->memory[1]++;
  }

  if (fsm_state->memory[1] > APOGEE_SAFETY_COUNTER) {
    trigger_event(EV_APOGEE);
    fsm_state->flight_state = APOGEE;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

static void check_apogee_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {
  if (state_data->velocity > PARACHUTE_DESCENT_SPEED) {
    /* Parachute Deployed */
    fsm_state->memory[1]++;
  } else {
    /* Parachute Not deployed */
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > PARACHUTE_SAFETY_COUNTER) {
    fsm_state->flight_state = DROGUE;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }

  //  if (fsm_state->memory[2] > BALLISTIC_SAFETY_COUNTER) {
  //    fsm_state->flight_state = BALLISTIC;
  //    fsm_state->clock_memory = 0;
  //    fsm_state->memory[1] = 0;
  //    fsm_state->memory[2] = 0;
  //  }
}

static void check_drogue_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {
  if (state_data->height < (float)global_cats_config.config.control_settings.main_altitude) {
    /* Achieved Height to deploy Main */
    fsm_state->memory[1]++;
  } else {
    /* Did Not Achieve */
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > MAIN_SAFETY_COUNTER) {
    trigger_event(EV_POST_APOGEE);
    fsm_state->flight_state = MAIN;
    fsm_state->clock_memory = 0;
    fsm_state->memory[0] = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

static void check_main_phase(flight_fsm_t *fsm_state, estimation_output_t *state_data) {
  /* If the velocity is very small we have touchdown */
  if (fabsf(state_data->velocity) < VELOCITY_BOUND_TOUCHDOWN) {
    /* Touchdown achieved */
    fsm_state->memory[1]++;
  } else {
    /* Touchdown not achieved */
    fsm_state->memory[1] = 0;
  }

  if (fsm_state->memory[1] > TOUCHDOWN_SAFETY_COUNTER) {
    trigger_event(EV_TOUCHDOWN);
    fsm_state->flight_state = TOUCHDOWN;
    fsm_state->clock_memory = 0;
    fsm_state->memory[1] = 0;
    fsm_state->memory[2] = 0;
  }
}

/* Todo: write a memory clear function to use in each state */
