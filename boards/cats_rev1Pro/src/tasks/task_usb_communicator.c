/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "util/log.h"
#include "cli/cli.h"
#include "config/globals.h"
#include "tasks/task_usb_communicator.h"

_Noreturn void task_usb_communicator(__attribute__((unused)) void *argument) {
  log_raw("USB config started");
  log_raw("CATS is now ready to receive commands...");

  fifo_flush(&usb_input_fifo);
  fifo_flush(&usb_output_fifo);
  cli_enter(&usb_input_fifo, &usb_output_fifo);
  while (1) {
    if (fifo_get_length(&usb_input_fifo)) {
      cli_process();
    }

    osDelay(10);
  }
}
