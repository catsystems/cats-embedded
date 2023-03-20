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

#include "tasks/task_cli.hpp"

#include "cli/cli.hpp"
#include "comm/stream_group.hpp"
#include "config/globals.hpp"
#include "util/log.h"

namespace task {

[[noreturn]] void Cli::Run() noexcept {
  log_raw("USB config started");
  log_raw("CATS is now ready to receive commands...");

  cli_enter();
  while (true) {
    if (stream_length(USB_SG.in) > 0) {
      cli_process();
    }

    osDelay(10);
  }
}

}  // namespace task
