/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "task_cdc.hpp"

#include <cstdint>

#include "comm/fifo.hpp"
#include "comm/stream_group.hpp"
#include "config/globals.hpp"
#include "tusb.h"

namespace task {

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
[[noreturn]] void Cdc::Run() noexcept {
  uint8_t buf[512]{};
  while (true) {
    while (tud_cdc_available() > 0) {
      global_usb_detection = true;
      const uint32_t count = tud_cdc_read(buf, sizeof(buf));
      stream_write(USB_SG.in, buf, count);
    }

    const uint32_t len = stream_length(USB_SG.out);
    if (len > 0 && stream_read(USB_SG.out, buf, len)) {
      tud_cdc_write(buf, len);
      tud_cdc_write_flush();
    }

    osDelay(1);
  }
}

}  // namespace task
