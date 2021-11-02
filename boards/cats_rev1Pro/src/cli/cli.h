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

#include "util/fifo.h"
#include "cli/cli.h"
#include "cli/settings.h"

#include <stdarg.h>

void cli_process(void);
void cli_enter(fifo_t *in, fifo_t *out);

void cli_print(const char *str);
void cli_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

void cli_print_linefeed(void);
void cli_print_line(const char *str);

void cli_print_linef(const char *format, ...) __attribute__((format(printf, 1, 2)));
void cli_print_error_linef(const char *cmdName, const char *format, ...) __attribute__((format(printf, 2, 3)));

uint8_t get_word_length(char *bufBegin, char *bufEnd);
char *skip_space(char *buffer);
uint16_t cli_get_setting_index(char *name, uint8_t length);
void get_min_max(const cli_value_t *var, int *min, int *max);

void cli_print_var(const char *cmdName, const cli_value_t *var, bool full);
void cli_print_var_range(const cli_value_t *var);

const char *next_arg(const char *current_arg);

bool is_empty(const char *string);