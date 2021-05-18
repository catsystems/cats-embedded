//
// Created by stoja on 31.01.21.
//

#include "util/log.h"
#include "config/cats_config.h"
#include "config/globals.h"

#include <stdint.h>
#include <stdlib.h>

/**
 *
 * @return True if config update successful
 */


void task_simulator(void *argument) {
  log_raw("Simulation Mode Initialized");
  log_raw("C.A.T.S. is now ready to receive simulation data...");

    imu_data_t simulation_imu_data[3] = { 0 };
    baro_data_t simulation_baro_data[3] = { 0 };

  while (1) {
    /* Receive Simulation Data */
    // Todo

    for(int i = 0; i < 3; i++){
        global_baro[i] = simulation_baro_data[i];
        global_imu[i] = simulation_imu_data[i];
    }

  }
}

