#pragma once

#include "main.h"
extern SPI_HandleTypeDef hspi2;

#define CATS_W25QXX_SPI     hspi2
#define CATS_W25QXX_CS_GPIO SPI2_CS_GPIO_Port
#define CATS_W25QXX_CS_PIN  SPI2_CS_Pin
#define CATS_W25QXX_DEBUG   0
