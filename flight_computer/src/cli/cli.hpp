/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from Betaflight (https://github.com/betaflight/betaflight),
/// released under GNU General Public License v3.0.

#pragma once

#include "cli/cli.hpp"
#include "cli/settings.hpp"
#include "comm/fifo.hpp"

void cli_process();
void cli_enter();

void cli_print(const char *str);
void cli_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

void cli_print_linefeed();
void cli_print_line(const char *str);

void cli_print_linef(const char *format, ...) __attribute__((format(printf, 1, 2)));
void cli_print_error_linef(const char *cmdName, const char *format, ...) __attribute__((format(printf, 2, 3)));

uint8_t get_word_length(char *bufBegin, char *bufEnd);
char *skip_space(char *buffer);
uint16_t cli_get_setting_index(char *name, uint8_t length);
void get_min_max(const cli_value_t *var, int *min, int *max);

void cli_print_var(const char *cmdName, const cats_config_t *cfg, const cli_value_t *var, bool full);
void cli_print_var_range(const cli_value_t *var);

const char *next_arg(const char *current_arg);

bool is_empty(const char *string);
