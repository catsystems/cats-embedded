#pragma once

#include "main.h"
extern QSPI_HandleTypeDef hqspi;

#define CATS_W25QXX_SPI     hqspi;
#define CATS_W25QXX_CS_GPIO GPIOB
#define CATS_W25QXX_CS_PIN  GPIO_PIN_11
#define CATS_W25QXX_DEBUG   0
