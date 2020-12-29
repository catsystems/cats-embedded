/*
 * task_baro_read.c
 *
 *  Created on: Nov 1, 2019
 *      Author: Jonas
 */

#include "tasks/task_state_est.h"
#include "control/kalman_filter.h"
#include "control/sensor_elimination.h"
#include "control/calibration.h"
#include "util/log.h"
#include "util/types.h"
#include "util/recorder.h"
#include "config/globals.h"

#include <math.h>
#include <stdlib.h>

inline static float calculate_height(float pressure_initial, float pressure,
                                     float temperature);

static void get_data_float(state_estimation_data_t *state_data,
                           kalman_filter_t *filter,
                           calibration_data_t *calibration);
/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
void task_state_est(void *argument) {
  /* For periodic update */
  uint32_t tick_count, tick_update;

  /* For Debugging */
  int32_t meters = 0;
  int32_t millimeters = 0;
  int32_t meters_per_s = 0;
  int32_t millimeters_per_s = 0;
  /* End Debugging */

  /* local flight phase */
  flight_fsm_t fsm_state = {.flight_state = MOVING};
  /* end local flight phase */

  /* calibration data */
  calibration_data_t calibration;
  calibration.angle = 1;
  calibration.axis = 2;
  /* end calibration data */

  /* Initialize State Estimation */
  state_estimation_data_t state_data = {0};
  sensor_elimination_t elimination = {0};
  kalman_filter_t filter = {0};
  filter.pressure_0 = P_INITIAL;
  filter.t_sampl = 1 / (float)(STATE_EST_SAMPLING_FREQ);

  initialize_matrices(&filter);
  /* End Initialization */

  osDelay(1000);

  /* Infinite loop */
  tick_count = osKernelGetTickCount();
  tick_update = osKernelGetTickFreq() / STATE_EST_SAMPLING_FREQ;

  while (1) {
    tick_count += tick_update;

    /* Update Flight Phase */
    fsm_state = global_flight_state;

    /* Really basic "Calibration" of the pressure just for testing purposes */
    if ((tick_count >= 5000) && (tick_count <= 5010)) {
      reset_kalman(&filter);
      filter.pressure_0 = state_data.pressure[0];
    }

    if (fsm_state.flight_state == INVALID) {
      log_error("Invalid FSM state!");
    }

    /* Reset IMU when we go from moving to IDLE */
    if ((fsm_state.flight_state == IDLE) && (fsm_state.state_changed == 1)) {
      reset_kalman(&filter);
      calibrate_imu(&global_imu[0], &calibration);
    }

    /* Get Sensor Readings already transformed in the right coordinate Frame */
    get_data_float(&state_data, &filter, &calibration);

    /* Check Sensor Readings */
    check_sensors(&state_data, &elimination);

    /* Do a Kalman Step */
    kalman_step(&filter, &state_data, &elimination);

    /* DEBUGGING: Making it Ready for Printing */
    if (filter.x_bar[0] > 0) {
      meters = abs((int32_t)(filter.x_bar[0]));
      millimeters = abs((int32_t)(filter.x_bar[0] * 1000) - meters * 1000);
    }

    else {
      meters = -abs((int32_t)(filter.x_bar[0]));
      millimeters = abs((int32_t)(filter.x_bar[0] * 1000) - meters * 1000);
    }

    if (filter.x_bar[1] > 0) {
      meters_per_s = abs((int32_t)(filter.x_bar[1]));
      millimeters_per_s =
          abs((int32_t)(filter.x_bar[1] * 1000) - meters_per_s * 1000);
    }

    else {
      meters_per_s = -abs((int32_t)(filter.x_bar[1]));
      millimeters_per_s =
          abs((int32_t)(filter.x_bar[1] * 1000) - meters_per_s * 1000);
    }

    log_trace("Height %ld.%ld: Velocity: %ld.%ld", meters, millimeters,
              meters_per_s, millimeters_per_s);
    /* END DEBUGGING */
    flight_info_t flight_info = {
        .ts = osKernelGetTickCount(),
        .height = meters + millimeters / 1000.f,
        .velocity = meters_per_s + millimeters_per_s / 1000.f};
    record(FLIGHT_INFO, &flight_info);

    /* TODO: Stuff with this Information */

    osDelayUntil(tick_count);
  }
}

inline static float calculate_height(float pressure_initial, float pressure,
                                     float temperature) {
  return ((powf(pressure_initial / pressure, (1 / 5.257f)) - 1) *
          (temperature + 273.15f) / 0.0065f);
}

void get_data_float(state_estimation_data_t *state_data,
                    kalman_filter_t *filter, calibration_data_t *calibration) {
  /* Get Data from the Sensors */
  /* Use calibration step to get the correct acceleration */
  switch (calibration->axis) {
    /* Choose X Axis */
    case 0:
      state_data->acceleration[0] =
          (float)(global_imu[0].acc_x) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[1] =
          (float)(global_imu[1].acc_x) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[2] =
          (float)(global_imu[2].acc_x) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      break;
      /* Choose Y Axis */
    case 1:
      state_data->acceleration[0] =
          (float)(global_imu[0].acc_y) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[1] =
          (float)(global_imu[1].acc_y) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[2] =
          (float)(global_imu[2].acc_y) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      break;
      /* Choose Z Axis */
    case 2:
      state_data->acceleration[0] =
          (float)(global_imu[0].acc_z) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[1] =
          (float)(global_imu[1].acc_z) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      state_data->acceleration[2] =
          (float)(global_imu[2].acc_z) / (1024) * GRAVITY / calibration->angle -
          GRAVITY;
      break;
    default:
      break;
  }

  state_data->pressure[0] = (float)(global_baro[0].pressure);
  state_data->pressure[1] = (float)(global_baro[1].pressure);
  state_data->pressure[2] = (float)(global_baro[2].pressure);

  state_data->temperature[0] = (float)(global_baro[0].temperature) / 100;
  state_data->temperature[1] = (float)(global_baro[1].temperature) / 100;
  state_data->temperature[2] = (float)(global_baro[2].temperature) / 100;

  state_data->calculated_AGL[0] = calculate_height(
      filter->pressure_0, state_data->pressure[0], state_data->temperature[0]);
  state_data->calculated_AGL[1] = calculate_height(
      filter->pressure_0, state_data->pressure[1], state_data->temperature[1]);
  state_data->calculated_AGL[2] = calculate_height(
      filter->pressure_0, state_data->pressure[2], state_data->temperature[2]);
}
