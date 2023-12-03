/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "drivers/buzzer.hpp"
#include "drivers/servo.hpp"
#include "drivers/spi.hpp"
#include "flash/recorder.hpp"

#include "target.hpp"
#include "util/types.hpp"

/** Sampling Frequencies **/
inline constexpr uint16_t CONTROL_SAMPLING_FREQ = 100;   // in Hz
inline constexpr uint16_t TELEMETRY_SAMPLING_FREQ = 10;  // in Hz

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)

/** State Estimation **/

extern baro_data_t global_baro_sim[NUM_BARO];
extern imu_data_t global_imu_sim[NUM_BARO];

extern osEventFlagsId_t fsm_flag_id;

/** Timers **/
extern cats_timer_t ev_timers[NUM_TIMERS];

/** Recorder Queue **/
extern osMessageQueueId_t rec_queue;
extern osMessageQueueId_t rec_cmd_queue;
extern osMessageQueueId_t event_queue;

extern volatile bool global_usb_detection;
extern volatile bool usb_device_initialized;
extern volatile bool usb_communication_complete;
extern volatile bool simulation_started;
extern volatile bool imu_initialized[NUM_IMU];

/* recorder status is controlled by output functions, do not set manually! */
extern volatile recorder_status_e global_recorder_status;

extern event_action_map_elem_t* event_action_map;

// clang-format off
// __PLATFORMIO_BUILD_DEBUG__ adds '-dbg', CATS_DEV adds '-dev'
#ifdef __PLATFORMIO_BUILD_DEBUG__
    #ifdef CATS_DEV
        inline constexpr const char* code_version = FIRMWARE_VERSION "-dbg-dev";
    #else
        inline constexpr const char* code_version = FIRMWARE_VERSION "-dbg";
    #endif
#else
    #ifdef CATS_DEV
        inline constexpr const char* code_version = FIRMWARE_VERSION "-dev";
    #else
        inline constexpr const char* code_version = FIRMWARE_VERSION;
    #endif
#endif
// clang-format on

inline constexpr const char* board_name = "CATS Vega";
extern char telemetry_code_version[20];

// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
