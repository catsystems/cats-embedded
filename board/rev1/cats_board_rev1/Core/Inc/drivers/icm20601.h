// ICM-20601 Device Library
// Author: Luca Jost
// 11.06.2020


#ifndef _ICM20602_H
#define _ICM20602_H

#include "stm32l4xx_hal.h"

// *** Macros *** //

#define ICM20601_INIT() \
  { \
    .cs_port = GPIOA, \
    .cs_pin = GPIO_PIN_4, \
    .spi_bus = &hspi1, \
    .accel_dlpf = ICM20601_ACCEL_DLPF_10_2_HZ, \
	.accel_g = ICM20601_ACCEL_RANGE_32G, \
	.gyro_dlpf = ICM20601_GYRO_DLPF_10_HZ, \
    .gyro_dps = ICM20601_GYRO_RANGE_2000_DPS, \
  }


// *** Defines *** //

#define REG_SELF_TEST_X_GYRO 0x00
#define REG_SELF_TEST_Y_GYRO 0x01
#define REG_SELF_TEST_Z_GYRO 0x02
#define REG_SELF_TEST_X_ACCEL 0x0D
#define REG_SELF_TEST_Y_ACCEL 0x0E
#define REG_SELF_TEST_Z_ACCEL 0x0F
#define REG_XG_OFFS_USRH 0x13
#define REG_XG_OFFS_USRL 0x14
#define REG_YG_OFFS_USRH 0x15
#define REG_YG_OFFS_USRL 0x16
#define REG_ZG_OFFS_USRH 0x17
#define REG_ZG_OFFS_USRL 0x18
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG_1 0x1C
#define REG_ACCEL_CONFIG_2 0x1D
#define REG_LP_MODE_CFG 0x1E
#define REG_ACCEL_WOM_X_THR 0x20
#define REG_ACCEL_WOM_Y_THR 0x21
#define REG_ACCEL_WOM_Z_THR 0x22
#define REG_FIFO_EN 0x23
#define REG_FSYNC_INT 0x36
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_FIFO_WM_INT_STATUS 0x39
#define REG_INT_STATUS 0x3A
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_XOUT_L 0x3C
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_YOUT_L 0x3E
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_ACCEL_ZOUT_L 0x40
#define REG_TEMP_OUT_H 0x41
#define REG_TEMP_OUT_L 0x42
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_XOUT_L 0x44
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_YOUT_L 0x46
#define REG_GYRO_ZOUT_H 0x47
#define REG_GYRO_ZOUT_L 0x48
#define REG_SIGNAL_PATH_RESET 0x68
#define REG_ACCEL_INTEL_CTRL 0x69
#define REG_USER_CTRL 0x6A
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_FIFO_COUNTH 0x72
#define REG_FIFO_COUNTL 0x73
#define REG_FIFO_R_W 0x74
#define REG_WHO_AM_I 0x75
#define REG_XA_OFFSET_H 0x77
#define REG_XA_OFFSET_L 0x78
#define REG_YA_OFFSET_H 0x7A
#define REG_YA_OFFSET_L 0x7B
#define REG_ZA_OFFSET_H 0x7D
#define REG_ZA_OFFSET_L 0x7E

#define REG_WHO_AM_I_CONST 0xAC

#define SENS_reset 0x81
#define SENS_internalpll 0x01
#define SENS_standby 0x3F
#define SENS_nofifo 0x00
#define SENS_disablei2c 0x41

#define IMU20601_SPI_TIMEOUT 3000


// *** enums *** //

/** Enumerated value corresponds with A_DLPF_CFG in the ACCEL_CONFIG2 register
  * unless BYPASS is specified in the name. If BYPASS is used, the DLPF is
  * removed from the signal path and ACCEL_FCHOICE_B is set in the
  * ACCEL_CONFIG2 register. */
enum icm20601_accel_dlpf {
  ICM20601_ACCEL_DLPF_218_1_HZ = 0, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_99_HZ = 2, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_44_8_HZ = 3, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_21_2_HZ = 4, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_10_2_HZ = 5, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_5_1_HZ = 6, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_420_HZ = 7, // data clocked at 1kHz
  ICM20601_ACCEL_DLPF_BYPASS_1046_HZ, // no filter, data clocked at 4kHz
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
  ICM20601_GYRO_DLPF_250_HZ = 0, // data clocked at 8kHz
  ICM20601_GYRO_DLPF_176_HZ = 1, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_92_HZ = 2, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_41_HZ = 3, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_20_HZ = 4, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_10_HZ = 5, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_5_HZ = 6, // data clocked at 1kHz
  ICM20601_GYRO_DLPF_3281_HZ = 7, // data clocked at 8kHz
  ICM20601_GYRO_DLPF_BYPASS_3281_HZ, // no filter, data clocked at 32kHz
  ICM20601_GYRO_DLPF_BYPASS_8173_HZ, // no filter, data clocked at 32kHz
};

/** Enumerated value corresponds with FS_SEL in the GYRO_CONFIG register.
  * Values listed are the full +/- DPS range. */
enum icm20601_gyro_dps {
  ICM20601_GYRO_RANGE_500_DPS = 0,
  ICM20601_GYRO_RANGE_1000_DPS = 1,
  ICM20601_GYRO_RANGE_2000_DPS = 2,
  ICM20601_GYRO_RANGE_4000_DPS = 3,
};


// *** structs *** //

typedef struct icm20601_dev {

	// Hardware Configuration
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	SPI_HandleTypeDef* spi_bus;
	// Sensor Configuration
	enum icm20601_accel_dlpf accel_dlpf;
	enum icm20601_accel_g accel_g;
	enum icm20601_gyro_dlpf gyro_dlpf;
	enum icm20601_gyro_dps gyro_dps;
} ICM20601;

// *** Global Functions *** //

extern int8_t icm20601_init(struct icm20601_dev * dev);

extern void icm20601_read_accel(struct icm20601_dev * dev, float *accel);

extern void icm20601_read_accel_raw(struct icm20601_dev * dev, int16_t *accel);

extern void icm20601_read_gyro(struct icm20601_dev * dev, float *gyro);

extern void icm20601_read_gyro_raw(struct icm20601_dev * dev, int16_t *gyro);

extern void icm20601_read_temp_raw(struct icm20601_dev * dev, int16_t *temp);


extern SPI_HandleTypeDef hspi1;

#endif
