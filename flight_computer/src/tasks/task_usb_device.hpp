/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "task.hpp"

namespace task {

class UsbDevice final : public Task<UsbDevice, 512> {
  [[noreturn]] void Run() noexcept override;
};

}  // namespace task
