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

#include "tasks/task_airbrake_controller.h"
#include "util/log.h"
#include "config/globals.h"
#include "control/lqr_controller.h"
#include "sensors/epos4.h"


[[noreturn]] void task_airbrake_controller(__attribute__((unused)) void *argument) {

    /* Initialize the control_data struct */
    control_data_t control_data = {0};
    control_init(&control_data);


    /* Infinite loop */
    uint32_t tick_count = osKernelGetTickCount();
    uint32_t tick_update = osKernelGetTickFreq() / CONTROL_SAMPLING_FREQ;

    /* Motor Controller */
    flight_fsm_e old_fsm_state = MOVING;
    int32_t position_des = 1;
    int8_t sample_divider = 0;

    disable_motor();

    while (1) {
        /* Tick Update */
        tick_count += tick_update;

        /* Call the Controller */
        compute_control_input(&control_data, global_flight_state.flight_state, &global_estimation_data);


        if((old_fsm_state == COASTING) && (sample_divider >= 3)){
            /* Todo: Set the control output to this function */
            move_to_position(position_des);
            position_des += 3;
            sample_divider = 0;
        }

        if((old_fsm_state == THRUSTING_1) && (global_flight_state.flight_state == COASTING)){
            enable_motor();
            osDelay(3);
            set_position_mode(0x01); // SLOW BOY
            //set_position_mode(0x08); // FAST BOY
        }

        if((old_fsm_state == COASTING) && (global_flight_state.flight_state == APOGEE)){
            home_motor();
            osDelay(3);
            disable_motor();
        }


        old_fsm_state = global_flight_state.flight_state;

        ++sample_divider;

        /* Sleep */
        osDelayUntil(tick_count);
    }
}