#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#include "main.h"
extern SPI_HandleTypeDef hspi2;

#define _W25QXX_SPI     hspi2
#define _W25QXX_CS_GPIO SPI2_CS_GPIO_Port
#define _W25QXX_CS_PIN  SPI2_CS_Pin
#define _W25QXX_DEBUG   0

#endif
