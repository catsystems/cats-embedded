//
// Created by stoja on 28.12.20.
//

#include "config/globals.h"
#include "drivers/icm20601.h"
#include "drivers/ms5607.h"
#include "drivers/buzzer.h"

/** Protocol Handles **/

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim15;

/** Device Handles **/

const ICM20601 ICM1 = {
    .cs_port = GPIOB,
    .cs_pin = GPIO_PIN_0,
    .spi_bus = &hspi1,
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ,
    .accel_g = ICM20601_ACCEL_RANGE_32G,
    .gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ,
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS,
};
const ICM20601 ICM2 = {
    .cs_port = GPIOB,
    .cs_pin = GPIO_PIN_1,
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

MS5607 MS1 = {
    .i2c_address = 0xEE,
    .i2c_bus = &hi2c1,
    .osr = MS5607_OSR_256,
};

MS5607 MS2 = {
    .i2c_address = 0xEC,
    .i2c_bus = &hi2c1,
    .osr = MS5607_OSR_256,
};

MS5607 MS3 = {
    .i2c_address = 0xEE,
    .i2c_bus = &hi2c2,
    .osr = MS5607_OSR_256,
};

BUZ BUZZER = {.timer = &htim15,
              .channel = TIM_CHANNEL_2,
              .arr = 4000,
              .started = 0,
              .volume = 100};

/** State Estimation **/

baro_data_t global_baro[3] = {0};
imu_data_t global_imu[3] = {0};
flight_fsm_t global_flight_state = {0};

/** Recorder Queue **/

osMessageQueueId_t rec_queue;