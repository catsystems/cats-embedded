/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Betaflight (https://github.com/betaflight/betaflight),
/// released under GNU General Public License v3.0.

#pragma once

#include <cstddef>

using cli_command_fn = void(const char *, char *);

struct clicmd_t {
  const char *name;
  const char *description;
  const char *args;
  cli_command_fn *cli_command;
};

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define CLI_COMMAND_DEF(name, description, args, cli_command) \
  { name, description, args, cli_command }

extern const clicmd_t cmd_table[];
extern size_t const NUM_CLI_COMMANDS;
