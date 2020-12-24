// ICM-20601 IMU Device Library
// Author: Luca Jost
// 11.06.2020

// *** Includes *** //

#include "drivers/icm20601.h"

// *** Macros *** //

#define UINT8_TO_INT16(dst, src_high, src_low) \
  do {                                         \
    dst = (src_high);                          \
    dst <<= 8;                                 \
    dst |= (src_low);                          \
  } while (0)

static const float temperature_sensitivity = 326.8f;

// *** Local functions *** //

// Used to convert raw accelerometer readings to G-force.
static float get_accel_sensitivity(enum icm20601_accel_g accel_g) {
  float f;

  switch (accel_g) {
    case (ICM20601_ACCEL_RANGE_4G):
      f = 8192.0f;
      break;
    case (ICM20601_ACCEL_RANGE_8G):
      f = 4096.0f;
      break;
    case (ICM20601_ACCEL_RANGE_16G):
      f = 2048.0f;
      break;
    case (ICM20601_ACCEL_RANGE_32G):
      f = 1024.0f;
      break;
    default:
      f = 0.0f;
  }
  return f;
}

// Used to convert raw gyroscope readings to degrees per second.
static float get_gyro_sensitivity(enum icm20601_gyro_dps gyro_dps) {
  float f;

  switch (gyro_dps) {
    case (ICM20601_GYRO_RANGE_500_DPS):
      f = 65.5f;
      break;
    case (ICM20601_GYRO_RANGE_1000_DPS):
      f = 32.8f;
      break;
    case (ICM20601_GYRO_RANGE_2000_DPS):
      f = 16.4f;
      break;
    case (ICM20601_GYRO_RANGE_4000_DPS):
      f = 8.2f;
      break;
    default:
      f = 0.0f;
  }
  return f;
}

// Read bytes from MEMS
static void icm_read_bytes(struct icm20601_dev *dev, uint8_t reg,
                           uint8_t *pData, uint16_t size) {
  reg = reg | 0x80;
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dev->spi_bus, &reg, 1, IMU20601_SPI_TIMEOUT);
  HAL_SPI_Receive(dev->spi_bus, pData, size, IMU20601_SPI_TIMEOUT);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// Write bytes to MEMS
static void icm_write_bytes(struct icm20601_dev *dev, uint8_t reg,
                            uint8_t *pData, uint16_t size) {
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dev->spi_bus, &reg, 1, IMU20601_SPI_TIMEOUT);
  HAL_SPI_Transmit(dev->spi_bus, pData, size, IMU20601_SPI_TIMEOUT);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// *** Global Functions *** //

int8_t icm20601_init(struct icm20601_dev *dev) {
  uint8_t tmp = 0;
  uint8_t r[1] = {0};

  // General Procedure:
  //  1. reset chip
  //  2. set clock for PLL for optimum performance as documented in datasheet
  //  3. place accelerometer and gyroscope into standby
  //  4. disable fifo
  //  5. configure chip
  //  6. enable accelerometer and gyroscope

  // full reset of chip
  tmp = SENS_reset;  // 0x81
  icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);
  HAL_Delay(1);

  // set clock to internal PLL
  tmp = SENS_internalpll;  // 0x01
  icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);
  HAL_Delay(1);

  // verify we are able to read from the chip
  icm_read_bytes(dev, REG_WHO_AM_I, r, 1);
  if (r[0] != REG_WHO_AM_I_CONST) return 0;

  // place accel and gyro on standby
  tmp = SENS_standby;  // 0x3F
  icm_write_bytes(dev, REG_PWR_MGMT_2, &tmp, 1);

  // disable fifo
  tmp = SENS_nofifo;  // 0x00
  icm_write_bytes(dev, REG_USER_CTRL, &tmp, 1);

  // disable chip I2C communications
  tmp = SENS_disablei2c;  // 0x41;
  icm_write_bytes(dev, REG_USER_CTRL, &tmp, 1);

  // Accelerometer filtering
  if (ICM20601_ACCEL_DLPF_BYPASS_1046_HZ == dev->accel_dlpf) {
    tmp = (0x01 << 3);
  } else {
    tmp = dev->accel_dlpf;
  }
  icm_write_bytes(dev, REG_ACCEL_CONFIG_2, &tmp, 1);

  // Accelerometer range
  tmp = (dev->accel_g) << 3;
  icm_write_bytes(dev, REG_ACCEL_CONFIG_1, &tmp, 1);

  // Gyro filtering
  // tmp = ((dev->gyro_dps) << 3) | SENS_gyrofilter; // filter: 0x02
  //_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp , 1);

  if (ICM20601_GYRO_DLPF_BYPASS_3281_HZ == dev->gyro_dlpf) {
    // bypass dpf and set dps
    tmp = 0x00;
    icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

    tmp = (dev->gyro_dps << 3) | 0x02;
    icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
  } else if (ICM20601_GYRO_DLPF_BYPASS_8173_HZ == dev->gyro_dlpf) {
    // bypass dpf and set dps
    tmp = 0x00;
    icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

    tmp = (dev->gyro_dps << 3) | 0x01;
    icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
  } else {
    // configure dpf and set dps
    tmp = dev->gyro_dlpf;
    icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

    tmp = dev->gyro_dps << 3;
    icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
  }

  tmp = 0x00;
  icm_write_bytes(dev, REG_PWR_MGMT_2, &tmp, 1);

  return 1;
}

// Read out raw acceleration data
void icm20601_read_accel_raw(struct icm20601_dev *dev, int16_t *accel) {
  uint8_t accel_8bit[6] = {0};
  icm_read_bytes(dev, REG_ACCEL_XOUT_H, accel_8bit, 6);

  UINT8_TO_INT16(accel[0], accel_8bit[0], accel_8bit[1]);
  UINT8_TO_INT16(accel[1], accel_8bit[2], accel_8bit[3]);
  UINT8_TO_INT16(accel[2], accel_8bit[4], accel_8bit[5]);
}

// Read out processed acceleration data
void icm20601_read_accel(struct icm20601_dev *dev, float *accel) {
  float accel_sensitivity;
  int16_t accel_raw[3] = {0};

  accel_sensitivity = get_accel_sensitivity(dev->accel_g);

  icm20601_read_accel_raw(dev, accel_raw);

  accel[0] = ((float)accel_raw[0]) / accel_sensitivity;
  accel[1] = ((float)accel_raw[1]) / accel_sensitivity;
  accel[2] = ((float)accel_raw[2]) / accel_sensitivity;
}

// Read out raw gyro data
void icm20601_read_gyro_raw(struct icm20601_dev *dev, int16_t *gyro) {
  uint8_t gyro_8bit[6] = {0};
  icm_read_bytes(dev, REG_GYRO_XOUT_H, gyro_8bit, 6);

  UINT8_TO_INT16(gyro[0], gyro_8bit[0], gyro_8bit[1]);
  UINT8_TO_INT16(gyro[1], gyro_8bit[2], gyro_8bit[3]);
  UINT8_TO_INT16(gyro[2], gyro_8bit[4], gyro_8bit[5]);
}

// Read out processed gyro data
void icm20601_read_gyro(struct icm20601_dev *dev, float *gyro) {
  float gyro_sensitivity;
  int16_t gyro_raw[3] = {0};

  gyro_sensitivity = get_gyro_sensitivity(dev->gyro_dps);

  icm20601_read_gyro_raw(dev, gyro_raw);

  gyro[0] = ((float)gyro_raw[0]) / gyro_sensitivity;
  gyro[1] = ((float)gyro_raw[1]) / gyro_sensitivity;
  gyro[2] = ((float)gyro_raw[2]) / gyro_sensitivity;
}

// Read out raw temperature data
void icm20601_read_temp_raw(struct icm20601_dev *dev, int16_t *temp) {
  uint8_t temp_8bit[2] = {0};
  icm_read_bytes(dev, REG_TEMP_OUT_H, temp_8bit, 2);

  UINT8_TO_INT16(*temp, temp_8bit[0], temp_8bit[1]);
}

// Read out processed temperature in degC
void icm20601_read_temp(struct icm20601_dev *dev, float *temp) {
  int16_t temperature_raw;
  icm20601_read_temp_raw(dev, &temperature_raw);

  *temp = ((float)temperature_raw) / temperature_sensitivity +
          25.0f;  // TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity)
                  // + 25degC
}

void icm20601_accel_z_calib(struct icm20601_dev *dev) {
  uint8_t accel_offset_8bit[2] = {0};
  int16_t accel_offset = 0;
  int16_t accel_real[3] = {0};

  // Read current offset
  icm_read_bytes(dev, REG_ZA_OFFSET_H, accel_offset_8bit, 2);
  UINT8_TO_INT16(accel_offset, accel_offset_8bit[0], accel_offset_8bit[1]);

  // Remove first bit as it is reserved and not used
  accel_offset = accel_offset >> 1;

  // Read acceleration from device
  icm20601_read_accel_raw(dev, accel_real);

  // Do some calculations, the offset register is +- 16g
  float diff = get_accel_sensitivity(dev->accel_g) - (float)accel_real[2];
  float scale = get_accel_sensitivity(dev->accel_g) / 2048.0f;
  accel_offset += (int16_t)(diff * scale);

  // Add the reserved bit back before setting the register
  accel_offset = accel_offset << 1;

  accel_offset_8bit[0] = ((accel_offset & 0xFF00) >> 8);
  accel_offset_8bit[1] = (accel_offset & 0xFF);

  // Write to offset register
  icm_write_bytes(dev, REG_ZA_OFFSET_H, accel_offset_8bit, 2);
}

void icm20601_accel_calib(struct icm20601_dev *dev, uint8_t axis) {
  if (axis > 2) return;
  HAL_Delay(100);
  uint8_t accel_offset_8bit[2] = {0};
  int16_t accel_offset = 0;
  int16_t accel_real[3] = {0};

  // Read current offset
  icm_read_bytes(dev, REG_XA_OFFSET_H + (3 * axis), accel_offset_8bit, 2);
  UINT8_TO_INT16(accel_offset, accel_offset_8bit[0], accel_offset_8bit[1]);

  // Remove first bit as it is reserved and not used
  accel_offset = accel_offset >> 1;

  // Read acceleration from device
  icm20601_read_accel_raw(dev, accel_real);

  // Do some calculations, the offset register is +- 16g
  float diff = get_accel_sensitivity(dev->accel_g) - (float)accel_real[axis];
  float scale = get_accel_sensitivity(dev->accel_g) / 2048.0f;
  accel_offset += (int16_t)(diff * scale);

  // Add the reserved bit back before setting the register
  accel_offset = accel_offset << 1;

  accel_offset_8bit[0] = ((accel_offset & 0xFF00) >> 8);
  accel_offset_8bit[1] = (accel_offset & 0xFF);

  // Write to offset register
  icm_write_bytes(dev, REG_XA_OFFSET_H + (3 * axis), accel_offset_8bit, 2);
}
