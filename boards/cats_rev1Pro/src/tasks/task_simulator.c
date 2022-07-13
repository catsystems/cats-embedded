//
// Created by jonas on 4/26/2022.
//

#include "config/globals.h"
#include "target.h"
#include "util/log.h"
#include "util/sim.h"

/** Private Constants **/

/** Private Function Declarations **/

/** Exported Function Definitions **/

/**
 * @brief Function implementing the task_state_est thread.
 * @param argument: Not used
 * @retval None
 */
[[noreturn]] void task_simulator(__attribute__((unused)) void *argument) {
  /* Change when the timestamp is reached */
  int8_t index_acc = 0;
  /* Linear interpolation */
  int8_t index_press = 0;
  imu_data_t sim_imu_data[NUM_IMU] = {};
  baro_data_t sim_baro_data[NUM_BARO] = {};

  /* RNG Init with known seed */
  srand((unsigned)1);

  uint32_t tick_count = osKernelGetTickCount();
  uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

  /* initialise time */
  timestamp_t sim_start = osKernelGetTickCount();
  timestamp_t time_since_start = 0;

  init_simulation_data();

  while (1) {
    time_since_start = osKernelGetTickCount() - sim_start;

    /* Check if we need to use new acceleration datapoint */
    if (time_since_start > acc_time_array[index_acc]) {
      index_acc++;
    }

    /* Compute wanted acceleration */
    sim_imu_data[0].acc.x = (int16_t)(1024 * acc_array[index_acc]);
    /* Add Noise */
    sim_imu_data[0].acc.x += (int16_t)rand_bounds(-10, 10);

    /* Check if we need to use new barometer datapoint */
    if (time_since_start > pressure_time_array[index_press + 1]) {
      index_press++;
    }
    /* Do linear interpolation for pressure measurement */
    sim_baro_data[0].pressure = linear_interpol(
        (float32_t)time_since_start, (float32_t)pressure_time_array[index_press],
        (float32_t)pressure_time_array[index_press + 1], pressure_array[index_press], pressure_array[index_press + 1]);
    /* Add Noise */
    sim_baro_data[0].pressure += rand_bounds(-25, 25);

    /* Set to Global Param */
    global_imu_sim[0].acc.x = sim_imu_data[0].acc.x;
    global_baro_sim[0].pressure = sim_baro_data[0].pressure;

    log_info("acc: %hd, pressure: %ld", sim_imu_data[0].acc.x, sim_baro_data[0].pressure);

    tick_count += tick_update;
    osDelayUntil(tick_count);
  }
}

/** Private Function Definitions **/
