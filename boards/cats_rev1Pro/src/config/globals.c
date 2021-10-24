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

#include "config/globals.h"

/** Protocol Handles **/

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;

/** Device Handles **/

SPI_BUS SPI1_ICM1 = {.cs_port = CS_IMU1_GPIO_Port, .cs_pin = CS_IMU1_Pin, .spi_handle = &hspi1, .cs_type = LOW_ACTIVE};

const ICM20601 ICM1 = {
    .spi = &SPI1_ICM1,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};

SPI_BUS SPI1_ICM2 = {.cs_port = CS_IMU2_GPIO_Port, .cs_pin = CS_IMU2_Pin, .spi_handle = &hspi1, .cs_type = LOW_ACTIVE};

const ICM20601 ICM2 = {
    .spi = &SPI1_ICM2,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};

SPI_BUS SPI_ACCEL = {.cs_port = CS_ACC_GPIO_Port, .cs_pin = CS_ACC_Pin, .spi_handle = &hspi1, .cs_type = LOW_ACTIVE};

const H3LIS100DL ACCEL = {
    .spi = &SPI_ACCEL,
    .power_mode = H3LIS100DL_PM_NM_ODR,
    .sample_rate = H3LIS100DL_ODR_100,
    .filter = H3LIS100DL_NO_FILTER,
};

SPI_BUS SPI_BARO1 = {
    .cs_port = CS_BARO1_GPIO_Port, .cs_pin = CS_BARO1_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS1 = {
    .cs_port = CS_BARO1_GPIO_Port,
    .cs_pin = CS_BARO1_Pin,
    .spi_bus = &SPI_BARO1,
    .osr = MS5607_OSR_1024,
};

SPI_BUS SPI_BARO2 = {
    .cs_port = CS_BARO2_GPIO_Port, .cs_pin = CS_BARO2_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS2 = {
    .cs_port = CS_BARO2_GPIO_Port,
    .cs_pin = CS_BARO2_Pin,
    .spi_bus = &SPI_BARO2,
    .osr = MS5607_OSR_1024,
};

SPI_BUS SPI_BARO3 = {
    .cs_port = CS_BARO3_GPIO_Port, .cs_pin = CS_BARO3_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS3 = {
    .cs_port = CS_BARO3_GPIO_Port,
    .cs_pin = CS_BARO3_Pin,
    .spi_bus = &SPI_BARO3,
    .osr = MS5607_OSR_1024,
};

SPI_BUS SPI_MAG = {
    .cs_port = CS_MAG_GPIO_Port,
    .cs_pin = CS_MAG_Pin,
    .cs_type = LOW_ACTIVE,
    .spi_handle = &hspi1,
};

MMC5983MA MAG = {
    .spi = &SPI_MAG,
    .sample_rate = MMC5983MA_ODR_100Hz,
    .bandwidth = MMC5983MA_BW_100Hz,
    .setreset = MMC5983MA_SET_1000,
    .mag_bias = {0.0108642578f, 0.0267333984f, 0.0308837891f},
    .mag_scale = {0.986369789f, 1.03176177f, 0.983317614f},
};

fifo_t usb_input_fifo;
fifo_t usb_output_fifo;

uint8_t usb_fifo_out_buffer[USB_OUTPUT_BUFFER_SIZE];
uint8_t usb_fifo_in_buffer[USB_INPUT_BUFFER_SIZE];

BUZ BUZZER = {.timer = &htim15, .channel = TIM_CHANNEL_2, .arr = 4000, .start = 0, .started = 0, .volume = 100};

SERVO SERVO1 = {.timer = &htim2, .channel = TIM_CHANNEL_2, .pulse = 15000, .started = 0};

SERVO SERVO2 = {.timer = &htim2, .channel = TIM_CHANNEL_1, .pulse = 15000, .started = 0};

/** State Estimation **/

baro_data_t global_baro[NUM_BARO] = {0};
imu_data_t global_imu[NUM_IMU] = {0};
accel_data_t global_accel[NUM_ACCELEROMETER] = {0};
magneto_data_t global_magneto[NUM_MAGNETO] = {0};

state_estimation_input_t global_estimation_input = {0};
SI_data_t global_SI_data = {0};
estimation_output_t global_estimation_data = {0};
flight_fsm_t global_flight_state = {.flight_state = MOVING};
drop_test_fsm_t global_drop_test_state = {.flight_state = DT_READY};
dt_telemetry_trigger_t dt_telemetry_trigger = {0};

/** Timers **/
cats_timer_t ev_timers[NUM_TIMERS] = {};
cats_timer_t mach_timer = {};

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

volatile recorder_status_e global_recorder_status = REC_OFF;

event_action_map_elem_t* event_action_map = NULL;

const char* code_version = "2.0.0+";
