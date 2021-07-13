#pragma once

#include "main.h"
extern SPI_HandleTypeDef hspi2;

#define CATS_W25QXX_SPI     hspi2
#define CATS_W25QXX_CS_GPIO CS_BARO3_Pin
#define CATS_W25QXX_CS_PIN  CS_BARO3_Pin
#define CATS_W25QXX_DEBUG   0
