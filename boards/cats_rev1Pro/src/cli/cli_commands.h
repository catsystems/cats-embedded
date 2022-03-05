/*
 * This file was part of Cleanflight and Betaflight.
 * https://github.com/betaflight/betaflight
 * It is modified for the CATS Flight Software.
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

typedef void cli_command_fn(const char *cmd_name, char *args);

typedef struct {
  const char *name;
  const char *description;
  const char *args;
  cli_command_fn *cli_command;
} clicmd_t;

#define CLI_COMMAND_DEF(name, description, args, cli_command) \
  { name, description, args, cli_command }

extern const clicmd_t cmd_table[];
extern size_t const NUM_CLI_COMMANDS;
