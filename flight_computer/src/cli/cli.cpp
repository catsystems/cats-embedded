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

#include "cli/cli.hpp"

#include <strings.h>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "cli/cli_commands.hpp"
#include "comm/stream_group.hpp"
#include "flash/lfs_custom.hpp"
#include "util/log.h"

#define CLI_IN_BUFFER_SIZE  128
#define CLI_OUT_BUFFER_SIZE 256

static uint32_t buffer_index = 0;

static char cli_buffer[CLI_IN_BUFFER_SIZE];
static char old_cli_buffer[CLI_IN_BUFFER_SIZE];

static void cli_print_error_va(const char *cmdName, const char *format, va_list va);
static void cli_print_error(const char *cmdName, const char *format, ...) __attribute__((format(printf, 2, 3)));

bool is_empty(const char *string) { return (string == nullptr || *string == '\0'); }

void get_min_max(const cli_value_t *var, int *min, int *max) {
  switch (var->type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
    case VAR_UINT16:
      *min = var->config.minmax_unsigned.min;
      *max = var->config.minmax_unsigned.max;

      break;
    default:
      *min = var->config.minmax.min;
      *max = var->config.minmax.max;

      break;
  }
}

void cli_print(const char *str) { stream_write(USB_SG.out, (uint8_t *)str, strlen(str)); }

static void cli_prompt() { cli_printf("\r\n^._.^:%s> ", cwd); }

void cli_print_linefeed() { cli_print("\r\n"); }

void cli_print_line(const char *str) {
  cli_print(str);
  cli_print_linefeed();
}

static void cli_printf_va(const char *format, va_list va) {
  static char buffer[CLI_OUT_BUFFER_SIZE];
  vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
  cli_print(buffer);
}

static void cli_write(uint8_t ch) { stream_write_byte(USB_SG.out, ch); }

void cli_printf(const char *format, ...) {
  va_list va;
  va_start(va, format);
  cli_printf_va(format, va);
  va_end(va);
}

void cli_print_linef(const char *format, ...) {
  va_list va;
  va_start(va, format);
  cli_printf_va(format, va);
  va_end(va);
  cli_print_linefeed();
}

static void cli_print_error_va(const char *cmdName, const char *format, va_list va) {
  cli_print("ERROR IN ");
  cli_print(cmdName);
  cli_print(": ");
  char buffer[CLI_OUT_BUFFER_SIZE];
  vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
  cli_print(buffer);
  cli_print(": ");
  va_end(va);
}

static void cli_print_error(const char *cmdName, const char *format, ...) {
  va_list va;
  va_start(va, format);
  cli_print_error_va(cmdName, format, va);
  va_end(va);
}

void cli_print_error_linef(const char *cmdName, const char *format, ...) {
  va_list va;
  va_start(va, format);
  cli_print_error_va(cmdName, format, va);
  cli_print("\r\n");
  va_end(va);
}

char *skip_space(char *buffer) {
  while (*(buffer) == ' ') {
    buffer++;
  }
  return buffer;
}

static char *check_command(char *cmdline, const char *command) {
  if (!strncasecmp(cmdline, command, strlen(command))  // command names match
      && (isspace((unsigned)cmdline[strlen(command)]) || cmdline[strlen(command)] == 0)) {
    return skip_space(cmdline + strlen(command) + 1);
  } else {
    return nullptr;
  }
}

static void process_character(const char c) {
  if (buffer_index && (c == '\n' || c == '\r')) {
    // enter pressed
    cli_print_linefeed();

    // Strip comment starting with # from line
    char *p = cli_buffer;
    p = strchr(p, '#');
    if (nullptr != p) {
      buffer_index = (uint32_t)(p - cli_buffer);
    }
    // Strip trailing whitespace
    while (buffer_index > 0 && cli_buffer[buffer_index - 1] == ' ') {
      buffer_index--;
    }

    // Process non-empty lines
    if (buffer_index > 0) {
      cli_buffer[buffer_index] = 0;  // null terminate

      const clicmd_t *cmd;
      char *options = nullptr;
      for (cmd = cmd_table; cmd < cmd_table + NUM_CLI_COMMANDS; cmd++) {
        options = check_command(cli_buffer, cmd->name);
        if (options) break;
      }
      if (cmd < cmd_table + NUM_CLI_COMMANDS) {
        cmd->cli_command(cmd->name, options);
      } else {
        cli_print_line("UNKNOWN COMMAND, TRY 'HELP'");
      }
      buffer_index = 0;
    }
    strncpy(old_cli_buffer, cli_buffer, sizeof(cli_buffer));
    memset(cli_buffer, 0, sizeof(cli_buffer));
    cli_prompt();

    // 'exit' will reset this flag, so we don't need to print prompt again

  } else if (buffer_index < sizeof(cli_buffer) && c >= 32 && c <= 126) {
    if (!buffer_index && c == ' ') return;  // Ignore leading spaces
    cli_buffer[buffer_index++] = c;
    cli_write(c);
  }
}

static void process_character_interactive(const char c) {
  // We ignore a few characters, this is only used for the up arrow
  static uint16_t ignore = 0;
  if (ignore) {
    ignore--;
    return;
  }
  if (c == '\t' || c == '?') {
    // do tab completion
    const clicmd_t *cmd, *pstart = nullptr, *pend = nullptr;
    uint32_t i = buffer_index;
    for (cmd = cmd_table; cmd < cmd_table + NUM_CLI_COMMANDS; cmd++) {
      if (buffer_index && (strncasecmp(cli_buffer, cmd->name, buffer_index) != 0)) {
        continue;
      }
      if (!pstart) {
        pstart = cmd;
      }
      pend = cmd;
    }
    if (pstart) { /* Buffer matches one or more commands */
      for (;; buffer_index++) {
        if (pstart->name[buffer_index] != pend->name[buffer_index]) break;
        if (!pstart->name[buffer_index] && buffer_index < sizeof(cli_buffer) - 2) {
          /* Unambiguous -- append a space */
          cli_buffer[buffer_index++] = ' ';
          cli_buffer[buffer_index] = '\0';
          break;
        }
        cli_buffer[buffer_index] = pstart->name[buffer_index];
      }
    }
    if (!buffer_index || pstart != pend) {
      /* Print list of ambiguous matches */
      cli_print("\r\n\033[K");
      for (cmd = pstart; cmd <= pend && cmd != nullptr; cmd++) {
        cli_print(cmd->name);
        cli_write('\t');
      }
      cli_prompt();
      i = 0; /* Redraw prompt */
    }
    for (; i < buffer_index; i++) cli_write(cli_buffer[i]);
  } else if (c == 4) {
    // CTRL-D - clear screen
    cli_print("\033[2J\033[1;1H");
    cli_prompt();
  } else if (c == 12) {  // CTRL-L - toggle logging
    if (log_is_enabled()) {
      log_disable();
      cli_prompt();
    } else {
      log_enable();
    }
  } else if (c == '\b') {
    // backspace
    if (buffer_index) {
      cli_buffer[--buffer_index] = 0;
      cli_print("\010 \010");
    }
  } else if (c == 27) {  // ESC character is called from the up arrow, we only look at the first of 3 characters
    // up arrow
    while (buffer_index) {
      cli_buffer[--buffer_index] = 0;
      cli_print("\010 \010");
    }
    for (uint32_t i = 0; i < sizeof(old_cli_buffer); i++) {
      if (old_cli_buffer[i] == 0) break;
      process_character(old_cli_buffer[i]);
    }
    // Ignore the following characters
    ignore = 2;
  } else {
    process_character(c);
  }
}

void cli_process(void) {
  while (stream_length(USB_SG.in) > 0) {
    uint8_t ch = 0;
    if (stream_read_byte(USB_SG.in, &ch)) {
      process_character_interactive(ch);
    }
  }
}

void cli_enter() { cli_prompt(); }

static void print_value_pointer(const char *cmdName, const cli_value_t *var, const void *valuePointer, bool full) {
  if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
    for (int i = 0; i < var->config.array.length; i++) {
      switch (var->type & VALUE_TYPE_MASK) {
        default:
        case VAR_UINT8:
          // uint8_t array
          cli_printf("%d", ((uint8_t *)valuePointer)[i]);
          break;

        case VAR_INT8:
          // int8_t array
          cli_printf("%d", ((int8_t *)valuePointer)[i]);
          break;

        case VAR_UINT16:
          // uin16_t array
          cli_printf("%d", ((uint16_t *)valuePointer)[i]);
          break;

        case VAR_INT16:
          // int16_t array
          cli_printf("%d", ((int16_t *)valuePointer)[i]);
          break;

        case VAR_UINT32:
          // uin32_t array
          cli_printf("%lu", ((uint32_t *)valuePointer)[i]);
          break;
      }

      if (i < var->config.array.length - 1) {
        cli_print(",");
      }
    }
  } else {
    int value = 0;

    switch (var->type & VALUE_TYPE_MASK) {
      case VAR_UINT8:
        value = *(uint8_t *)valuePointer;

        break;
      case VAR_INT8:
        value = *(int8_t *)valuePointer;

        break;
      case VAR_UINT16:
        value = *(uint16_t *)valuePointer;

        break;
      case VAR_INT16:
        value = *(int16_t *)valuePointer;

        break;
      case VAR_UINT32:
        value = *(uint32_t *)valuePointer;

        break;
    }

    bool value_is_corrupted = false;
    switch (var->type & VALUE_MODE_MASK) {
      case MODE_DIRECT:
        if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32) {
          cli_printf("%lu", (uint32_t)value);
          if ((uint32_t)value > var->config.u32_max) {
            value_is_corrupted = true;
          } else if (full) {
            cli_printf(" 0 %lu", var->config.u32_max);
          }
        } else {
          int min;
          int max;
          get_min_max(var, &min, &max);

          cli_printf("%d", value);
          if ((value < min) || (value > max)) {
            value_is_corrupted = true;
          } else if (full) {
            cli_printf(" %d %d", min, max);
          }
        }
        break;
      case MODE_LOOKUP:
        if (static_cast<size_t>(value) < lookup_tables[var->config.lookup.table_index].size()) {
          cli_print(lookup_tables[var->config.lookup.table_index][value]);
        } else {
          value_is_corrupted = true;
        }
        break;
      case MODE_BITSET:
        if (value & 1 << var->config.bitpos) {
          cli_printf("ON");
        } else {
          cli_printf("OFF");
        }
        break;
      case MODE_STRING:
        cli_printf("%s", (strlen((char *)valuePointer) == 0) ? "-" : (char *)valuePointer);
        break;
    }

    if (value_is_corrupted) {
      cli_print_linefeed();
      cli_print_error(cmdName, "CORRUPTED CONFIG: %s = %d", var->name, value);
    }
  }
}

void cli_print_var(const char *cmdName, const cats_config_t *cfg, const cli_value_t *var, bool full) {
  const void *ptr = get_cats_config_member_ptr(cfg, var);

  print_value_pointer(cmdName, var, ptr, full);
}

uint8_t get_word_length(char *bufBegin, char *bufEnd) {
  while (*(bufEnd - 1) == ' ') {
    bufEnd--;
  }

  return bufEnd - bufBegin;
}

uint16_t cli_get_setting_index(char *name, uint8_t length) {
  for (uint32_t i = 0; i < value_table_entry_count; i++) {
    const char *setting_name = value_table[i].name;

    // ensure exact match when setting to prevent setting variables with shorter names
    if (strncasecmp(name, setting_name, strlen(setting_name)) == 0 && length == strlen(setting_name)) {
      return i;
    }
  }
  return value_table_entry_count;
}

const char *next_arg(const char *current_arg) {
  const char *ptr = strchr(current_arg, ' ');
  while (ptr && *ptr == ' ') {
    ptr++;
  }
  return ptr;
}

void cli_print_var_range(const cli_value_t *var) {
  switch (var->type & VALUE_MODE_MASK) {
    case (MODE_DIRECT): {
      switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT32:
          cli_print_linef("Allowed range: 0 - %lu", var->config.u32_max);
          break;
        case VAR_UINT8:
        case VAR_UINT16:
          cli_print_linef("Allowed range: %d - %d", var->config.minmax_unsigned.min, var->config.minmax_unsigned.max);
          break;
        default:
          cli_print_linef("Allowed range: %d - %d", var->config.minmax.min, var->config.minmax.max);
          break;
      }
    } break;
    case (MODE_LOOKUP): {
      const EnumToStrMap &tableEntry = lookup_tables[var->config.lookup.table_index];
      cli_print("Allowed values: ");
      bool first_entry = true;
      for (uint32_t i = 0; i < tableEntry.size(); i++) {
        if (tableEntry[i]) {
          if (!first_entry) {
            cli_print(", ");
          }
          cli_printf("%s", tableEntry[i]);
          first_entry = false;
        }
      }
      cli_print_linefeed();
    } break;
    case (MODE_ARRAY): {
      cli_print_linef("Array length: %d", var->config.array.length);
    } break;
    case (MODE_STRING): {
      cli_print_linef("String length: %d - %d", var->config.string.min_length, var->config.string.max_length);
    } break;
    case (MODE_BITSET): {
      cli_print_linef("Allowed values: OFF, ON");
    } break;
  }
}