/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
