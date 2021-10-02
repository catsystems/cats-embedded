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

#pragma once

_Noreturn void task_usb_communicator(void *argument);

#define USB_COMMAND_NR  10
#define USB_VARIABLE_NR 6
#define USB_FILTER_NR   7

typedef enum {
  CATS_USB_CMD_SAVE = 0,
  CATS_USB_CMD_EXIT,
  CATS_USB_CMD_STATUS,
  CATS_USB_CMD_VERSION,
  CATS_USB_CMD_DUMP,
  CATS_USB_CMD_FLASH_ERASE,
  CATS_USB_CMD_SET,
  CATS_USB_CMD_GET,
  CATS_USB_CMD_READ,
  CATS_USB_CMD_HELP,
  CATS_USB_CMD_UNKNOWN
} cats_usb_commands;

typedef enum {
  CATS_USB_VAR_LIFTOFF_ACC_THRESH = 0,
  CATS_USB_VAR_FILTER_CONF,
  CATS_USB_VAR_APOGEE_TIMER1,
  CATS_USB_VAR_APOGEE_TIMER2,
  CATS_USB_VAR_STAGES,
  CATS_USB_VAR_BOOT_STATE,
  CATS_USB_VAR_UNKNOWN
} cats_usb_variables;
