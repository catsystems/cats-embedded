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

// ICM-20601 IMU Device Library

// *** Includes *** //

#include "sensors/icm20601.h"
#include <stdlib.h>

/** Private Defines **/

#define REG_SELF_TEST_X_GYRO   0x00
#define REG_SELF_TEST_Y_GYRO   0x01
#define REG_SELF_TEST_Z_GYRO   0x02
#define REG_SELF_TEST_X_ACCEL  0x0D
#define REG_SELF_TEST_Y_ACCEL  0x0E
#define REG_SELF_TEST_Z_ACCEL  0x0F
#define REG_XG_OFFS_USRH       0x13
#define REG_XG_OFFS_USRL       0x14
#define REG_YG_OFFS_USRH       0x15
#define REG_YG_OFFS_USRL       0x16
#define REG_ZG_OFFS_USRH       0x17
#define REG_ZG_OFFS_USRL       0x18
#define REG_SMPLRT_DIV         0x19
#define REG_CONFIG             0x1A
#define REG_GYRO_CONFIG        0x1B
#define REG_ACCEL_CONFIG_1     0x1C
#define REG_ACCEL_CONFIG_2     0x1D
#define REG_LP_MODE_CFG        0x1E
#define REG_ACCEL_WOM_X_THR    0x20
#define REG_ACCEL_WOM_Y_THR    0x21
#define REG_ACCEL_WOM_Z_THR    0x22
#define REG_FIFO_EN            0x23
#define REG_FSYNC_INT          0x36
#define REG_INT_PIN_CFG        0x37
#define REG_INT_ENABLE         0x38
#define REG_FIFO_WM_INT_STATUS 0x39
#define REG_INT_STATUS         0x3A
#define REG_ACCEL_XOUT_H       0x3B
#define REG_ACCEL_XOUT_L       0x3C
#define REG_ACCEL_YOUT_H       0x3D
#define REG_ACCEL_YOUT_L       0x3E
#define REG_ACCEL_ZOUT_H       0x3F
#define REG_ACCEL_ZOUT_L       0x40
#define REG_TEMP_OUT_H         0x41
#define REG_TEMP_OUT_L         0x42
#define REG_GYRO_XOUT_H        0x43
#define REG_GYRO_XOUT_L        0x44
#define REG_GYRO_YOUT_H        0x45
#define REG_GYRO_YOUT_L        0x46
#define REG_GYRO_ZOUT_H        0x47
#define REG_GYRO_ZOUT_L        0x48
#define REG_SIGNAL_PATH_RESET  0x68
#define REG_ACCEL_INTEL_CTRL   0x69
#define REG_USER_CTRL          0x6A
#define REG_PWR_MGMT_1         0x6B
#define REG_PWR_MGMT_2         0x6C
#define REG_FIFO_COUNTH        0x72
#define REG_FIFO_COUNTL        0x73
#define REG_FIFO_R_W           0x74
#define REG_WHO_AM_I           0x75
#define REG_XA_OFFSET_H        0x77
#define REG_XA_OFFSET_L        0x78
#define REG_YA_OFFSET_H        0x7A
#define REG_YA_OFFSET_L        0x7B
#define REG_ZA_OFFSET_H        0x7D
#define REG_ZA_OFFSET_L        0x7E

#define REG_WHO_AM_I_CONST 0xAC

#define IMU20601_SPI_TIMEOUT 3000

#define SENS_reset       0x81
#define SENS_internalpll 0x01
#define SENS_standby     0x3F
#define SENS_nofifo      0x00
#define SENS_disablei2c  0x41

/** Private Constants **/

static const float temperature_sensitivity = 326.8f;

/** Private Function Declarations **/

// Used to convert raw accelerometer readings to G-force.
static float get_accel_sensitivity(enum icm20601_accel_g accel_g);
// Used to convert raw gyroscope readings to degrees per second.
static float get_gyro_sensitivity(enum icm20601_gyro_dps gyro_dps);
// Read bytes from MEMS
static void icm_read_bytes(const ICM20601 *dev, uint8_t reg, uint8_t *data, uint16_t length);
// Write bytes to MEMS
static void icm_write_bytes(const ICM20601 *dev, uint8_t reg, uint8_t *data, uint16_t length);

/** Exported Function Definitions **/

/**
 *
 * @param dev
 * @return true if initialization successful
 */
bool icm20601_init(const ICM20601 *dev) {
  // These need to be volatile otherwise it will break with optimizations
  volatile uint8_t tmp;
  volatile uint8_t reg;

  // General Procedure:
  //  1. reset chip
  //  2. set clock for PLL for optimum performance as documented in datasheet
  //  3. place accelerometer and gyroscope into standby
  //  4. disable fifo
  //  5. configure chip
  //  6. enable accelerometer and gyroscope

  // full reset of chip
  tmp = SENS_reset;  // 0x81
  reg = REG_PWR_MGMT_1;
  icm_write_bytes(dev, reg, &tmp, 1);
  HAL_Delay(1);

  // set clock to internal PLL
  tmp = SENS_internalpll;  // 0x01
  reg = REG_PWR_MGMT_1;
  icm_write_bytes(dev, reg, &tmp, 1);
  HAL_Delay(1);

  // verify we are able to read from the chip
  uint8_t buffer = 0;
  reg = REG_WHO_AM_I;
  icm_read_bytes(dev, reg, &buffer, 1);
  if (buffer != REG_WHO_AM_I_CONST) return false;

  // place accel and gyro on standby
  tmp = SENS_standby;  // 0x3F
  reg = REG_PWR_MGMT_2;
  icm_write_bytes(dev, reg, &tmp, 1);

  // disable fifo
  tmp = SENS_nofifo;  // 0x00
  reg = REG_USER_CTRL;
  icm_write_bytes(dev, reg, &tmp, 1);

  // disable chip I2C communications
  tmp = SENS_disablei2c;  // 0x41;
  reg = REG_USER_CTRL;
  icm_write_bytes(dev, reg, &tmp, 1);

  // Accelerometer filtering
  if (dev->accel_dlpf == ICM20601_ACCEL_DLPF_BYPASS_1046_HZ) {
    tmp = (0x01 << 3);
  } else {
    tmp = dev->accel_dlpf;
  }
  reg = REG_ACCEL_CONFIG_2;
  icm_write_bytes(dev, reg, &tmp, 1);

  // Accelerometer range
  tmp = (dev->accel_g) << 3;
  reg = REG_ACCEL_CONFIG_1;
  icm_write_bytes(dev, reg, &tmp, 1);

  // Gyro filtering
  // tmp = ((dev->gyro_dps) << 3) | SENS_gyrofilter; // filter: 0x02
  //_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp , 1);

  if (ICM20601_GYRO_DLPF_BYPASS_3281_HZ == dev->gyro_dlpf) {
    // bypass dpf and set dps
    tmp = 0x00;
    reg = REG_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);

    tmp = (dev->gyro_dps << 3) | 0x02;
    reg = REG_GYRO_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);
  } else if (ICM20601_GYRO_DLPF_BYPASS_8173_HZ == dev->gyro_dlpf) {
    // bypass dpf and set dps
    tmp = 0x00;
    reg = REG_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);

    tmp = (dev->gyro_dps << 3) | 0x01;
    reg = REG_GYRO_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);
  } else {
    // configure dpf and set dps
    tmp = dev->gyro_dlpf;
    reg = REG_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);

    tmp = dev->gyro_dps << 3;
    reg = REG_GYRO_CONFIG;
    icm_write_bytes(dev, reg, &tmp, 1);
  }

  tmp = 0x00;
  reg = REG_PWR_MGMT_2;
  icm_write_bytes(dev, reg, &tmp, 1);

  return true;
}

// Read out raw acceleration data
void icm20601_read_accel_raw(const ICM20601 *dev, int16_t *accel) {
  uint8_t accel_8bit[6] = {0};
  uint8_t reg = REG_ACCEL_XOUT_H;
  icm_read_bytes(dev, reg, accel_8bit, 6);

  accel[0] = uint8_to_int16(accel_8bit[0], accel_8bit[1]);
  accel[1] = uint8_to_int16(accel_8bit[2], accel_8bit[3]);
  accel[2] = uint8_to_int16(accel_8bit[4], accel_8bit[5]);
}

// Read out processed acceleration data
void icm20601_read_accel(const ICM20601 *dev, float *accel) {
  float accel_sensitivity;
  int16_t accel_raw[3] = {0};

  accel_sensitivity = get_accel_sensitivity(dev->accel_g);

  icm20601_read_accel_raw(dev, accel_raw);

  accel[0] = ((float)accel_raw[0]) / accel_sensitivity;
  accel[1] = ((float)accel_raw[1]) / accel_sensitivity;
  accel[2] = ((float)accel_raw[2]) / accel_sensitivity;
}

// Read out raw gyro data
void icm20601_read_gyro_raw(const ICM20601 *dev, int16_t *gyro) {
  uint8_t gyro_8bit[6] = {0};
  uint8_t reg = REG_GYRO_XOUT_H;
  icm_read_bytes(dev, reg, gyro_8bit, 6);

  gyro[0] = uint8_to_int16(gyro_8bit[0], gyro_8bit[1]);
  gyro[1] = uint8_to_int16(gyro_8bit[2], gyro_8bit[3]);
  gyro[2] = uint8_to_int16(gyro_8bit[4], gyro_8bit[5]);
}

// Read out processed gyro data
void icm20601_read_gyro(const ICM20601 *dev, float *gyro) {
  float gyro_sensitivity;
  int16_t gyro_raw[3] = {0};

  gyro_sensitivity = get_gyro_sensitivity(dev->gyro_dps);

  icm20601_read_gyro_raw(dev, gyro_raw);

  gyro[0] = ((float)gyro_raw[0]) / gyro_sensitivity;
  gyro[1] = ((float)gyro_raw[1]) / gyro_sensitivity;
  gyro[2] = ((float)gyro_raw[2]) / gyro_sensitivity;
}

// Read out raw temperature data
void icm20601_read_temp_raw(const ICM20601 *dev, int16_t *temp) {
  uint8_t temp_8bit[2] = {0};
  uint8_t reg = REG_TEMP_OUT_H;
  icm_read_bytes(dev, reg, temp_8bit, 2);

  *temp = uint8_to_int16(temp_8bit[0], temp_8bit[1]);
}

// Read out processed temperature in degC
void icm20601_read_temp(const ICM20601 *dev, float *temp) {
  int16_t temperature_raw;
  icm20601_read_temp_raw(dev, &temperature_raw);

  *temp = ((float)temperature_raw) / temperature_sensitivity +
          25.0f;  // TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)
                  // + 25degC
}

void icm20601_accel_calib(const ICM20601 *dev) {
  uint8_t accel_offset_8bit[2] = {0};
  int16_t accel_real[3] = {0};

  int16_t accel_offset[3];

  // Read current offset
  for (int i = 0; i < 3; i++) {
    uint8_t reg = REG_XA_OFFSET_H + (3 * i);
    icm_read_bytes(dev, reg, accel_offset_8bit, 2);
    accel_offset[i] = uint8_to_int16(accel_offset_8bit[0], accel_offset_8bit[1]);

    // Remove first bit as it is reserved and not used
    accel_offset[i] = (int16_t)(accel_offset[i] >> 1);
  }

  // Read acceleration from device
  icm20601_read_accel_raw(dev, accel_real);

  uint8_t down_axis = 0;
  // Decide which axis is down
  if (abs(accel_real[0]) > abs(accel_real[1]) && abs(accel_real[0]) > abs(accel_real[2]))
    down_axis = 0;
  else if (abs(accel_real[1]) > abs(accel_real[0]) && abs(accel_real[1]) > abs(accel_real[2]))
    down_axis = 1;
  else if (abs(accel_real[2]) > abs(accel_real[1]) && abs(accel_real[2]) > abs(accel_real[0]))
    down_axis = 2;

  // Do some calculations, the offset register is +- 16g
  for (int i = 0; i < 3; i++) {
    float diff;
    if (i == down_axis)
      diff = get_accel_sensitivity(dev->accel_g) - (float)accel_real[i];
    else
      diff = -(float)accel_real[i];
    float scale = get_accel_sensitivity(dev->accel_g) / 2048.0f;
    accel_offset[i] = (int16_t)(accel_offset[i] + (int16_t)(diff * scale));

    // Add the reserved bit back before setting the register
    accel_offset[i] = (int16_t)(accel_offset[i] << 1);

    accel_offset_8bit[0] = ((accel_offset[i] & 0xFF00) >> 8);
    accel_offset_8bit[1] = (accel_offset[i] & 0xFF);

    // Write to offset register
    uint8_t reg = REG_XA_OFFSET_H + (3 * i);
    icm_write_bytes(dev, reg, accel_offset_8bit, 2);
  }
}

void icm20601_gyro_cal(const ICM20601 *dev, uint8_t *data) {
  // uint8_t gyro_cal[8];
  icm_read_bytes(dev, REG_XG_OFFS_USRH, data, 6);
}

/** Private Function Definitions **/

// Used to convert raw accelerometer readings to G-force.
static float get_accel_sensitivity(enum icm20601_accel_g accel_g) {
  switch (accel_g) {
    case (ICM20601_ACCEL_RANGE_4G):
      return 8192.0f;
    case (ICM20601_ACCEL_RANGE_8G):
      return 4096.0f;
    case (ICM20601_ACCEL_RANGE_16G):
      return 2048.0f;
    case (ICM20601_ACCEL_RANGE_32G):
      return 1024.0f;
    default:
      return 0.0f;
  }
}

// Used to convert raw gyroscope readings to degrees per second.
static float get_gyro_sensitivity(enum icm20601_gyro_dps gyro_dps) {
  switch (gyro_dps) {
    case (ICM20601_GYRO_RANGE_500_DPS):
      return 65.5f;
    case (ICM20601_GYRO_RANGE_1000_DPS):
      return 32.8f;
    case (ICM20601_GYRO_RANGE_2000_DPS):
      return 16.4f;
    case (ICM20601_GYRO_RANGE_4000_DPS):
      return 8.2f;
    default:
      return 0.0f;
  }
}

// Read bytes from MEMS
static void icm_read_bytes(const ICM20601 *dev, uint8_t reg, uint8_t *data, uint16_t length) {
  reg = reg | 0x80;
  spi_transmit_receive(dev->spi, &reg, 1, data, length);
}

// Write bytes to MEMS
static void icm_write_bytes(const ICM20601 *dev, uint8_t reg, uint8_t *data, uint16_t length) {
  uint8_t tmp[8];
  tmp[0] = reg;
  if (length < 8) {
    memcpy(&tmp[1], data, length);
    spi_transmit(dev->spi, tmp, length + 1);
  }
}
