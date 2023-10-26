/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>

uint32_t crc32(const uint8_t *buf, size_t size);
uint8_t crc8(const uint8_t *buf, size_t size);
