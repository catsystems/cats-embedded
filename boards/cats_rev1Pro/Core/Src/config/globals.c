//
// Created by stoja on 28.12.20.
//

#include "config/globals.h"

/** Protocol Handles **/

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;

/** Device Handles **/

SPI_BUS SPI1_CS1 = {.cs_port = GPIOB, .cs_pin = GPIO_PIN_0, .spi_handle = &hspi1, .cs_type = LOW_ACTIVE};

const ICM20601 ICM1 = {
    .cs_port = CS_IMU1_GPIO_Port,
    .cs_pin = CS_IMU1_Pin,
    .spi_bus = &hspi1,
    .spi = &SPI1_CS1,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};
const ICM20601 ICM2 = {
    .cs_port = CS_IMU2_GPIO_Port,
    .cs_pin = CS_IMU2_Pin,
    .spi_bus = &hspi1,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};
const ICM20601 ICM3 = {
    .cs_port = GPIOB,
    .cs_pin = GPIO_PIN_2,
    .spi_bus = &hspi1,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};

SPI_BUS SPI_BARO1 = {
    .cs_port = CS_BARO1_GPIO_Port, .cs_pin = CS_BARO1_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS1 = {
    .cs_port = CS_BARO1_GPIO_Port,
    .cs_pin = CS_BARO1_Pin,
    .spi_bus = &SPI_BARO1,
    .osr = MS5607_OSR_2048,
};

SPI_BUS SPI_BARO2 = {
    .cs_port = CS_BARO2_GPIO_Port, .cs_pin = CS_BARO2_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS2 = {
    .cs_port = CS_BARO2_GPIO_Port,
    .cs_pin = CS_BARO2_Pin,
    .spi_bus = &SPI_BARO2,
    .osr = MS5607_OSR_2048,
};

SPI_BUS SPI_BARO3 = {
    .cs_port = CS_BARO3_GPIO_Port, .cs_pin = CS_BARO3_Pin, .spi_handle = &hspi2, .cs_type = LOW_ACTIVE};

MS5607 MS3 = {
    .cs_port = CS_BARO3_GPIO_Port,
    .cs_pin = CS_BARO3_Pin,
    .spi_bus = &SPI_BARO3,
    .osr = MS5607_OSR_2048,
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
    .mag_bias = {0.0108642578, 0.0267333984, 0.0308837891},
    .mag_scale = {0.986369789, 1.03176177, 0.983317614},
};

fifo_t usb_input_fifo;
fifo_t usb_output_fifo;

uint8_t usb_fifo_out_buffer[256];
uint8_t usb_fifo_in_buffer[64];

BUZ BUZZER = {.timer = &htim15, .channel = TIM_CHANNEL_2, .arr = 4000, .start = 0, .started = 0, .volume = 100};

SERVO SERVO1 = {.timer = &htim2, .channel = TIM_CHANNEL_1, .pulse = 15000, .started = 0};

SERVO SERVO2 = {.timer = &htim2, .channel = TIM_CHANNEL_2, .pulse = 15000, .started = 0};

/** State Estimation **/

baro_data_t global_baro[3] = {0};
imu_data_t global_imu[3] = {0};
magneto_data_t global_magneto = {0};
estimation_output_t global_kf_data = {0};
sensor_elimination_t global_elimination_data = {0};
flight_fsm_t global_flight_state = {.flight_state = MOVING};
drop_test_fsm_t global_drop_test_state = {.flight_state = DT_IDLE};
dt_telemetry_trigger_t dt_telemetry_trigger = {0};

/** Timers **/
uint32_t num_timers = 8;
cats_timer_t ev_timers[8] = {};

/** Recorder Queue **/
osMessageQueueId_t rec_queue;
osMessageQueueId_t event_queue;

/** Tracing Channels **/

#if (configUSE_TRACE_FACILITY == 1)
#include "tracing/trcRecorder.h"
traceString baro_channel;
traceString flash_channel;
#endif

volatile bool global_usb_detection = false;
volatile bool usb_communication_complete = false;

volatile recorder_status_e global_recorder_status = REC_OFF;

event_action_map_elem_t *event_action_map = NULL;
