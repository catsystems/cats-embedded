/*
 * drop_test_phases.c
 *
 *  Created on: Mar 17, 2021
 *      Author: jonas
 */

#include "control/drop_test_phases.h"
#include "util/types.h"
#include "cmsis_os.h"

void check_dt_idle_phase(drop_test_fsm_t *fsm_state,
                         dt_telemetry_trigger_t *dt_telemetry_trigger);
void check_dt_waiting_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                            dt_telemetry_trigger_t *dt_telemetry_trigger);
void check_dt_drogue_phase(drop_test_fsm_t *fsm_state,
                           dt_telemetry_trigger_t *dt_telemetry_trigger);
// void check_dt_main_phase(drop_test_fsm_t *fsm_state,
//		dt_telemetry_trigger_t *dt_telemetry_trigger);

void check_drop_test_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                           dt_telemetry_trigger_t *dt_telemetry_trigger) {
  /* Save old FSM state */
  drop_test_fsm_t old_fsm_state = *fsm_state;

  switch (fsm_state->flight_state) {
    case DT_IDLE:
      check_dt_idle_phase(fsm_state, dt_telemetry_trigger);
      break;
    case DT_WAITING:
      check_dt_waiting_phase(fsm_state, imu_data, dt_telemetry_trigger);
      break;
    case DT_DROGUE:
      check_dt_drogue_phase(fsm_state, dt_telemetry_trigger);
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

void check_dt_idle_phase(drop_test_fsm_t *fsm_state,
                         dt_telemetry_trigger_t *dt_telemetry_trigger) {
  if (dt_telemetry_trigger->set_waiting == 1) {
    fsm_state->flight_state = DT_WAITING;
    fsm_state->timer_start = osKernelGetTickCount();
    fsm_state->memory = 0;
  }
}
void check_dt_waiting_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                            dt_telemetry_trigger_t *dt_telemetry_trigger) {
  /* Check if Disarming */
  if (dt_telemetry_trigger->set_waiting == 0) {
    fsm_state->flight_state = DT_IDLE;
    fsm_state->timer_start = 0;
  }

  /* Check IMU */
  int32_t acceleration = imu_data->acc_x * imu_data->acc_x +
                         imu_data->acc_y * imu_data->acc_y +
                         imu_data->acc_z * imu_data->acc_z;
  if (acceleration < FREE_FALL_DET_ACC_SQ) {
    fsm_state->memory++;
  } else {
    fsm_state->memory = 0;
  }

  if (fsm_state->memory < FREE_FALL_SAFETY_COUNTER) {
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start = osKernelGetTickCount();
  }

  /* Check Timer */
  if ((fsm_state->timer_start + DROGUE_TIMER) < osKernelGetTickCount()) {
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start = osKernelGetTickCount();
  }

  /* Check Remote Signal */
  if (dt_telemetry_trigger->set_drogue == 1) {
    fsm_state->flight_state = DT_DROGUE;
    fsm_state->timer_start = osKernelGetTickCount();
  }
}

void check_dt_drogue_phase(drop_test_fsm_t *fsm_state,
                           dt_telemetry_trigger_t *dt_telemetry_trigger) {
  /* Check if Timer */
  if ((fsm_state->timer_start + MAIN_TIMER) < osKernelGetTickCount()) {
    fsm_state->flight_state = DT_MAIN;
  }

  /* Check Remote Signal */
  if (dt_telemetry_trigger->set_main == 1) {
    fsm_state->flight_state = DT_MAIN;
  }
}
