#pragma once

#include <cstdint>

enum BitOrder : uint8_t { LSBFIRST = 0, MSBFIRST = 1 };
enum : uint8_t { SPI_MODE0 = 0, SPI_MODE1 = 1, SPI_MODE2 = 2, SPI_MODE3 = 3 };

class SPISettings {};
class SPIClass {};
inline SPIClass SPI;
