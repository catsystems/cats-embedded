/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "task_usb_device.hpp"

#include "tusb.h"

namespace task {

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void UsbDevice::Run() noexcept {
  tud_init(0);
  while (true) {
    // put this thread to waiting state until there is new events
    tud_task();
  }
}

}  // namespace task
