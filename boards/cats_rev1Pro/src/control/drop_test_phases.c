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

#include "control/drop_test_phases.h"
#include "tasks/task_peripherals.h"
#include "cmsis_os.h"

void check_dt_ready_phase(drop_test_fsm_t *fsm_state, dt_telemetry_trigger_t *telemetry_trigger);
void check_dt_waiting_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                            dt_telemetry_trigger_t *telemetry_trigger);
void check_dt_drogue_phase(drop_test_fsm_t *fsm_state, dt_telemetry_trigger_t *telemetry_trigger);
// void check_dt_main_phase(drop_test_fsm_t *fsm_state,
//		dt_telemetry_trigger_t *dt_telemetry_trigger);

void check_drop_test_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                           dt_telemetry_trigger_t *telemetry_trigger) {
  /* Save old FSM state */
  drop_test_fsm_t old_fsm_state = *fsm_state;

  switch (fsm_state->flight_state) {
    case DT_READY:
      check_dt_ready_phase(fsm_state, telemetry_trigger);
      break;
    case DT_WAITING:
      check_dt_waiting_phase(fsm_state, imu_data, telemetry_trigger);
      break;
    case DT_DROGUE:
      check_dt_drogue_phase(fsm_state, telemetry_trigger);
      break;
    case DT_MAIN:
      //		check_dt_main_phase(fsm_state, dt_telemetry_trigger);
      break;
    case DT_TOUCHDOWN:
      break;
    default:
      break;
  }

  if (old_fsm_state.flight_state != fsm_state->flight_state) {
    fsm_state->state_changed = 1;
  } else {
    fsm_state->state_changed = 0;
  }
}

void check_dt_ready_phase(drop_test_fsm_t *fsm_state, dt_telemetry_trigger_t *telemetry_trigger) {
  if (telemetry_trigger->set_waiting == 1) {
    trigger_event(EV_READY);
    fsm_state->flight_state = DT_WAITING;
    fsm_state->timer_start_drogue = osKernelGetTickCount();
    fsm_state->memory = 0;
  }
}
void check_dt_waiting_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                            dt_telemetry_trigger_t *telemetry_trigger) {
  /* Check IMU */
  int32_t acceleration =
      imu_data->acc_x * imu_data->acc_x + imu_data->acc_y * imu_data->acc_y + imu_data->acc_z * imu_data->acc_z;
  if (acceleration < FREE_FALL_DET_ACC_SQ) {
    fsm_state->memory++;
  } else {
    fsm_state->memory = 0;
  }

  if (fsm_state->memory > FREE_FALL_SAFETY_COUNTER) {
    trigger_event(EV_APOGEE);
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start_main = osKernelGetTickCount();
  }

  /* Check Timer */
  if (DROGUE_TIMER < (osKernelGetTickCount() - fsm_state->timer_start_drogue)) {
    trigger_event(EV_APOGEE);
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start_main = osKernelGetTickCount();
  }

  /* Check Remote Signal */
  if (telemetry_trigger->set_drogue == 1) {
    trigger_event(EV_APOGEE);
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start_main = osKernelGetTickCount();
  }

  /* Check if Disarming */
  if (telemetry_trigger->set_waiting == 0) {
    trigger_event(EV_TOUCHDOWN);
    fsm_state->flight_state = DT_READY;
  }
}

void check_dt_drogue_phase(drop_test_fsm_t *fsm_state, dt_telemetry_trigger_t *telemetry_trigger) {
  /* Check if Timer */
  if (MAIN_TIMER < (osKernelGetTickCount() - fsm_state->timer_start_main)) {
    trigger_event(EV_POST_APOGEE);
    fsm_state->flight_state = DT_MAIN;
  }

  /* Check Remote Signal */
  if (telemetry_trigger->set_main == 1) {
    trigger_event(EV_POST_APOGEE);
    fsm_state->flight_state = DT_MAIN;
  }
}
