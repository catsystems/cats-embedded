/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

#include <cstdint>

/* Random pattern to tell system to jump to bootloader */
constexpr uint32_t BOOTLOADER_MAGIC_PATTERN = 0xAA88BB77;
