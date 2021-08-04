// H3LIS100DL Device Library
// Author: Luca Jost
// 04.08.2021

#pragma once

#include "stm32l4xx_hal.h"
#include "drivers/spi.h"
#include <stdbool.h>

/*	REGISTERS	*/
#define H3LIS100DL_WHO_AM_I  0x0F  // Answer should be 0x32
#define H3LIS100DL_CTRL_REG1 0x20
#define H3LIS100DL_CTRL_REG2 0x21
#define H3LIS100DL_CTRL_REG3 0x22
#define H3LIS100DL_CTRL_REG4 0x23
#define H3LIS100DL_CTRL_REG5 0x24

#define H3LIS100DL_FILTER_RESET 0x25
#define H3LIS100DL_REFERENCE    0x26
#define H3LIS100DL_STATUS_REG   0x27

#define H3LIS100DL_OUT_X 0x28
#define H3LIS100DL_OUT_Y 0x2B
#define H3LIS100DL_OUT_Z 0x2D

#define H3LIS100DL_INT_CFG1 0x30
#define H3LIS100DL_INT_SRC1 0x31
#define H3LIS100DL_INT_THS1 0x32
#define H3LIS100DL_INT_DUR1 0x33

#define H3LIS100DL_INT_CFG2 0x34
#define H3LIS100DL_INT_SRC2 0x35
#define H3LIS100DL_INT_THS2 0x36
#define H3LIS100DL_INT_DUR2 0x37

#define H3LIS100DL_WHO_AM_I_CONST 0x32

// CTRL_REG_1
enum h3lis100dl_power_mode {
  H3LIS100DL_PM_PD = 0x00,      // Power Down
  H3LIS100DL_PM_NM_ODR = 0x20,  // Normal Mode ODR
  H3LIS100DL_PM_LP_05 = 0x40,   // Low power 0.5Hz
  H3LIS100DL_PM_LP_1 = 0x60,    // Low power 1Hz
  H3LIS100DL_PM_LP_2 = 0x80,    // Low power 2Hz
  H3LIS100DL_PM_LP_5 = 0xA0,    // Low power 5Hz
  H3LIS100DL_PM_LP_10 = 0xC0,   // Low power 10Hz
};

enum h3lis100dl_sample_rate {
  H3LIS100DL_ODR_50 = 0x00,
  H3LIS100DL_ODR_100 = 0x08,
  H3LIS100DL_ODR_400 = 0x10,
};

// CTRL_REG_2
// High Pass fiter cutoff:
// ft = fs/(6*HPC)
enum h3lis100dl_filter {
  H3LIS100DL_HPC_8 = 0x00,
  H3LIS100DL_HPC_16 = 0x01,
  H3LIS100DL_HPC_32 = 0x02,
  H3LIS100DL_HPC_64 = 0x03,
};

typedef struct h3lis100dl_dev {
  // Hardware configuration
  SPI_BUS *spi;
  // Sensor configuration
  enum h3lis100dl_power_mode power_mode;
  enum h3lis100dl_sample_rate sample_rate;
  enum h3lis100dl_filter filter;
} H3LIS100DL;

bool h3lis100dl_init(const H3LIS100DL *dev);
void h3lis100dl_read_raw(const H3LIS100DL *dev, int8_t *data);
void h3lis100dl_read(const H3LIS100DL *dev, float *data);