/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

/* The code used here was taken from
 * https://web.mit.edu/freebsd/head/sys/libkern/crc32.c
 */
uint8_t crc8(const uint8_t *buf, uint32_t size);
uint32_t crc32(const uint8_t *buf, uint32_t size);
