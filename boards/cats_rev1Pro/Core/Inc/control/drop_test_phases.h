/*
 * drop_test_phases.h
 *
 *  Created on: Mar 17, 2021
 *      Author: jonas
 */

#pragma once

#include "util/types.h"

#define FREE_FALL_DET_ACC        500
#define FREE_FALL_DET_ACC_SQ     (FREE_FALL_DET_ACC * FREE_FALL_DET_ACC)
#define FREE_FALL_SAFETY_COUNTER 250
#define DROGUE_TIMER             12800 /* Is in ms */
#define MAIN_TIMER               16000 /* Is in ms */

void check_drop_test_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                           dt_telemetry_trigger_t *dt_telemetry_trigger);
