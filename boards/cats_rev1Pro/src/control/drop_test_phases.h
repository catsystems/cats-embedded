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

#pragma once

#include "util/types.h"

#define FREE_FALL_DET_ACC        500
#define FREE_FALL_DET_ACC_SQ     (FREE_FALL_DET_ACC * FREE_FALL_DET_ACC)
#define FREE_FALL_SAFETY_COUNTER 250
#define DROGUE_TIMER             12800 /* Is in ms */
#define MAIN_TIMER               16000 /* Is in ms */

void check_drop_test_phase(drop_test_fsm_t *fsm_state, imu_data_t *imu_data,
                           dt_telemetry_trigger_t *dt_telemetry_trigger);
