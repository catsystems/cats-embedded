/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "config/globals.h"

#include "comm/fifo.h"
#include "drivers/spi.h"
#include "target.h"

/** Protocol Handles **/

/** Device Handles **/

SPI_BUS SPI_IMU[NUM_IMU] = {
#if NUM_IMU > 0
    {
        .cs_port = CS_IMU1_GPIO_Port,
        .cs_pin = CS_IMU1_Pin,
        .cs_type = LOW_ACTIVE,
        .spi_handle = &IMU_SPI_HANDLE,
    },
#endif
#if NUM_IMU > 1
    {
        .cs_port = CS_IMU2_GPIO_Port,
        .cs_pin = CS_IMU2_Pin,
        .cs_type = LOW_ACTIVE,
        .spi_handle = &IMU_SPI_HANDLE,
    }
#endif
#if NUM_IMU > 2
    {
        .cs_port = CS_IMU3_GPIO_Port,
        .cs_pin = CS_IMU3_Pin,
        .spi_handle = &IMU_SPI_HANDLE,
        .cs_type = LOW_ACTIVE,
    }
#endif
};
#if IMU_TYPE == ICM20601_TYPE
const ICM20601 IMU_DEV[NUM_IMU] = {
#if NUM_IMU > 0
    {
        .spi = &SPI_IMU[0],
        .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
        .accel_g = ICM20601_ACCEL_RANGE_32G,
        .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
        .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
    },
#endif
#if NUM_IMU > 1
    {
        .spi = &SPI_IMU[1],
        .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
        .accel_g = ICM20601_ACCEL_RANGE_32G,
        .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
        .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
    }
#endif
#if NUM_IMU > 2
    {
        .spi = &SPI_IMU[2],
        .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
        .accel_g = ICM20601_ACCEL_RANGE_32G,
        .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
        .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
    }
#endif
};
#elif IMU_TYPE == LSM6DSR_TYPE
LSM6DSR IMU_DEV[NUM_IMU] = {
#if NUM_IMU > 0
        {
            .spi_handle = &IMU_SPI_HANDLE,
            .accel_odr = LSM6DSR_XL_ODR_104Hz,
            .accel_range = LSM6DSR_16g,
            .gyro_odr = LSM6DSR_GY_ODR_104Hz,
            .gyro_range = LSM6DSR_2000dps,
        },
#endif
};
#endif

#if NUM_ACCELEROMETER > 0
SPI_BUS SPI_ACCEL = {
    .cs_port = CS_ACC_GPIO_Port,
    .cs_pin = CS_ACC_Pin,
    .cs_type = LOW_ACTIVE,
    .spi_handle = &ACCEL_SPI_HANDLE,
};

const H3LIS100DL ACCEL = {
    .spi = &SPI_ACCEL,
    .power_mode = H3LIS100DL_PM_NM_ODR,
    .sample_rate = H3LIS100DL_ODR_100,
    .filter = H3LIS100DL_NO_FILTER,
};
#endif

SPI_BUS SPI_BARO[NUM_BARO] = {
#if NUM_BARO > 0
    {
        .cs_port = CS_BARO1_GPIO_Port,
        .cs_pin = CS_BARO1_Pin,
        .cs_type = LOW_ACTIVE,
        .spi_handle = &BARO_SPI_HANDLE,
    },
#endif
#if NUM_BARO > 1
    {
        .cs_port = CS_BARO2_GPIO_Port,
        .cs_pin = CS_BARO2_Pin,
        .cs_type = LOW_ACTIVE,
        .spi_handle = &BARO_SPI_HANDLE,
    },
#endif
#if NUM_BARO > 2
    {
        .cs_port = CS_BARO3_GPIO_Port,
        .cs_pin = CS_BARO3_Pin,
        .cs_type = LOW_ACTIVE,
        .spi_handle = &BARO_SPI_HANDLE,
    }
#endif
};

MS5607 BARO_DEV[NUM_BARO] = {
#if NUM_BARO > 0
    {
        .cs_port = CS_BARO1_GPIO_Port,
        .cs_pin = CS_BARO1_Pin,
        .spi_bus = &SPI_BARO[0],
        .osr = MS5607_OSR_1024,
    },
#endif
#if NUM_BARO > 1
    {
        .cs_port = CS_BARO2_GPIO_Port,
        .cs_pin = CS_BARO2_Pin,
        .spi_bus = &SPI_BARO[1],
        .osr = MS5607_OSR_1024,
    },
#endif
#if NUM_BARO > 2
    {
        .cs_port = CS_BARO3_GPIO_Port,
        .cs_pin = CS_BARO3_Pin,
        .spi_bus = &SPI_BARO[2],
        .osr = MS5607_OSR_1024,
    }
#endif
};

#if NUM_MAGNETO > 0
SPI_BUS SPI_MAG = {
    .cs_port = CS_MAG_GPIO_Port,
    .cs_pin = CS_MAG_Pin,
    .cs_type = LOW_ACTIVE,
    .spi_handle = &MAG_SPI_HANDLE,
};

MMC5983MA MAG = {
    .spi = &SPI_MAG,
    .sample_rate = MMC5983MA_ODR_100Hz,
    .bandwidth = MMC5983MA_BW_100Hz,
    .setreset = MMC5983MA_SET_1000,
    .mag_bias = {0.0108642578f, 0.0267333984f, 0.0308837891f},
    .mag_scale = {0.986369789f, 1.03176177f, 0.983317614f},
};
#endif

BUZ BUZZER = {.timer = &BUZZER_TIMER_HANDLE,
              .channel = BUZZER_TIMER_CHANNEL,
              .arr = 4000,
              .started = 0,
              .start = 0,
              .volume = 100};

SERVO SERVO1 = {.timer = &SERVO_TIMER_HANDLE, .channel = SERVO_TIMER_CHANNEL_1, .pulse = 15000, .started = 0};

SERVO SERVO2 = {.timer = &SERVO_TIMER_HANDLE, .channel = SERVO_TIMER_CHANNEL_2, .pulse = 15000, .started = 0};

/** State Estimation **/

baro_data_t global_baro[NUM_BARO] = {0};
imu_data_t global_imu[NUM_IMU] = {0};
accel_data_t global_accel[NUM_ACCELEROMETER] = {};
magneto_data_t global_magneto[NUM_MAGNETO] = {};

baro_data_t global_baro_sim[NUM_BARO] = {};
imu_data_t global_imu_sim[NUM_BARO] = {};

state_estimation_input_t global_estimation_input = {0};
SI_data_t global_SI_data = {0};
estimation_output_t global_estimation_data = {0};
flight_fsm_t global_flight_state = {.flight_state = MOVING};

bool global_arming_bool = false;

/** Timers **/
cats_timer_t ev_timers[NUM_TIMERS] = {};

/** Recorder Queue **/
osMessageQueueId_t rec_queue;
osMessageQueueId_t rec_cmd_queue;
osMessageQueueId_t event_queue;

/** Tracing Channels **/

#if (configUSE_TRACE_FACILITY == 1)
traceString baro_channel;
traceString flash_channel;
#endif

volatile bool global_usb_detection = false;
volatile bool usb_communication_complete = false;
volatile bool simulation_started = false;

volatile recorder_status_e global_recorder_status = REC_OFF;

event_action_map_elem_t* event_action_map = NULL;

const char* code_version = "2.2.0";
