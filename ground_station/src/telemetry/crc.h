#pragma once

#include <cstddef>
#include <cstdint>

uint32_t crc32(const uint8_t *buf, size_t size);
uint8_t crc8(const uint8_t *buf, size_t size);
