// ICM-20601 Device Library
// Author: Luca Jost
// 11.06.2020

#ifndef CATS_ICM20602_H
#define CATS_ICM20602_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>

/** Exported Types **/

/** Enumerated value corresponds with A_DLPF_CFG in the ACCEL_CONFIG2 register
 * unless BYPASS is specified in the name. If BYPASS is used, the DLPF is
 * removed from the signal path and ACCEL_FCHOICE_B is set in the
 * ACCEL_CONFIG2 register. */
enum icm20601_accel_dlpf {
  ICM20601_ACCEL_DLPF_218_1_HZ = 0,    // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_99_HZ = 2,       // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_44_8_HZ = 3,     // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_21_2_HZ = 4,     // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_10_2_HZ = 5,     // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_5_1_HZ = 6,      // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_420_HZ = 7,      // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_BYPASS_1046_HZ,  // no filter, data clocked at 4kHz
};

enum icm20601_accel_g {
  ICM20601_ACCEL_RANGE_4G = 0,
  ICM20601_ACCEL_RANGE_8G = 1,
  ICM20601_ACCEL_RANGE_16G = 2,
  ICM20601_ACCEL_RANGE_32G = 3,
};

/** Enumerated value corresponds with DLPF_CFG in the CONFIG register unless
 * BYPASS is specified in the name. If BYPASS is used, the DLPF is removed
 * from the signal path and FCHOICE_B is set in GYRO_CONFIG register. */
enum icm20601_gyro_dlpf {
  ICM20601_GYRO_DLPF_250_HZ = 0,      // data clocked at 8kHz
  ICM20601_GYRO_DLPF_176_HZ = 1,      // data clocked at 1kHz
  ICM20601_GYRO_DLPF_92_HZ = 2,       // data clocked at 1kHz
  ICM20601_GYRO_DLPF_41_HZ = 3,       // data clocked at 1kHz
  ICM20601_GYRO_DLPF_20_HZ = 4,       // data clocked at 1kHz
  ICM20601_GYRO_DLPF_10_HZ = 5,       // data clocked at 1kHz
  ICM20601_GYRO_DLPF_5_HZ = 6,        // data clocked at 1kHz
  ICM20601_GYRO_DLPF_3281_HZ = 7,     // data clocked at 8kHz
  ICM20601_GYRO_DLPF_BYPASS_3281_HZ,  // no filter, data clocked at 32kHz
  ICM20601_GYRO_DLPF_BYPASS_8173_HZ,  // no filter, data clocked at 32kHz
};

/** Enumerated value corresponds with FS_SEL in the GYRO_CONFIG register.
 * Values listed are the full +/- DPS range. */
enum icm20601_gyro_dps {
  ICM20601_GYRO_RANGE_500_DPS = 0,
  ICM20601_GYRO_RANGE_1000_DPS = 1,
  ICM20601_GYRO_RANGE_2000_DPS = 2,
  ICM20601_GYRO_RANGE_4000_DPS = 3,
};

typedef struct icm20601_dev {
  // Hardware Configuration
  GPIO_TypeDef *const cs_port;
  uint16_t cs_pin;
  SPI_HandleTypeDef *const spi_bus;
  // Sensor Configuration
  enum icm20601_accel_dlpf accel_dlpf;
  enum icm20601_accel_g accel_g;
  enum icm20601_gyro_dlpf gyro_dlpf;
  enum icm20601_gyro_dps gyro_dps;
} ICM20601;

/** Exported Functions **/

bool icm20601_init(const ICM20601 *dev);
void icm20601_read_accel(const ICM20601 *dev, float *accel);
void icm20601_read_accel_raw(const ICM20601 *dev, int16_t *accel);
void icm20601_read_gyro(const ICM20601 *dev, float *gyro);
void icm20601_read_gyro_raw(const ICM20601 *dev, int16_t *gyro);
void icm20601_read_temp_raw(const ICM20601 *dev, int16_t *temp);
void icm20601_accel_calib(const ICM20601 *dev, uint8_t axis);

#endif
