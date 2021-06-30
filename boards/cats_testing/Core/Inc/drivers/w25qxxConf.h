#ifndef CATS_W25QXXCONFIG_H
#define CATS_W25QXXCONFIG_H

#include "main.h"
extern SPI_HandleTypeDef hspi2;

#define CATS_W25QXX_SPI     hspi2
#define CATS_W25QXX_CS_GPIO IO2_GPIO_Port
#define CATS_W25QXX_CS_PIN  IO2_Pin
#define CATS_W25QXX_DEBUG   0

#endif
