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

#include "drivers/w25q.h"
#include "cli/cli.h"
#include "cli/cli_commands.h"
#include "util/log.h"
#include "util/reader.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "util/actions.h"
#include "util/battery.h"
#include "lfs/lfs_custom.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/** CLI command function declarations **/

static void cli_cmd_help(const char *cmd_name, char *args);

static void cli_cmd_reboot(const char *cmd_name, char *args);
static void cli_cmd_save(const char *cmd_name, char *args);

static void cli_cmd_get(const char *cmd_name, char *args);
static void cli_cmd_set(const char *cmd_name, char *args);
static void cli_cmd_config(const char *cmd_name, char *args);
static void cli_cmd_defaults(const char *cmd_name, char *args);

static void cli_cmd_status(const char *cmd_name, char *args);
static void cli_cmd_version(const char *cmd_name, char *args);

static void cli_cmd_log_enable(const char *cmd_name, char *args);

static void cli_cmd_ls(const char *cmd_name, char *args);
static void cli_cmd_cd(const char *cmd_name, char *args);
static void cli_cmd_rm(const char *cmd_name, char *args);

static void cli_cmd_dump_flight(const char *cmd_name, char *args);
static void cli_cmd_parse_flight(const char *cmd_name, char *args);
static void cli_cmd_parse_stats(const char *cmd_name, char *args);

static void cli_cmd_lfs_format(const char *cmd_name, char *args);
static void cli_cmd_erase_flash(const char *cmd_name, char *args);
static void cli_cmd_erase_recordings(const char *cmd_name, char *args);

static void cli_cmd_flash_write(const char *cmd_name, char *args);
static void cli_cmd_flash_stop(const char *cmd_name, char *args);
static void cli_cmd_flash_test(const char *cmd_name, char *args);

/* List of CLI commands; should be sorted in alphabetical order. */
const clicmd_t cmd_table[] = {
    CLI_COMMAND_DEF("cd", "change current working directory", NULL, cli_cmd_cd),
    CLI_COMMAND_DEF("config", "print the flight config", NULL, cli_cmd_config),
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cli_cmd_defaults),
    CLI_COMMAND_DEF("flash_erase", "erase the flash", NULL, cli_cmd_erase_flash),
    CLI_COMMAND_DEF("flash_test", "test the flash", NULL, cli_cmd_flash_test),
    CLI_COMMAND_DEF("flash_start_write", "set recorder state to REC_WRITE_TO_FLASH", NULL, cli_cmd_flash_write),
    CLI_COMMAND_DEF("flash_stop_write", "set recorder state to REC_FILL_QUEUE", NULL, cli_cmd_flash_stop),
    CLI_COMMAND_DEF("flight_dump", "print a specific flight", "<flight_number>", cli_cmd_dump_flight),
    CLI_COMMAND_DEF("flight_parse", "print a specific flight", "<flight_number>", cli_cmd_parse_flight),
    CLI_COMMAND_DEF("get", "get variable value", "[cmd_name]", cli_cmd_get),
    CLI_COMMAND_DEF("help", "display command help", "[search string]", cli_cmd_help),
    CLI_COMMAND_DEF("lfs_format", "reformat lfs", NULL, cli_cmd_lfs_format),
    CLI_COMMAND_DEF("log_enable", "enable the logging output", NULL, cli_cmd_log_enable),
    CLI_COMMAND_DEF("ls", "list all files in current working directory", NULL, cli_cmd_ls),
    CLI_COMMAND_DEF("reboot", "reboot without saving", NULL, cli_cmd_reboot),
    CLI_COMMAND_DEF("rec_erase", "erase the recordings", NULL, cli_cmd_erase_recordings),
    CLI_COMMAND_DEF("rm", "remove a file", "<file_name>", cli_cmd_rm),
    CLI_COMMAND_DEF("save", "save configuration", NULL, cli_cmd_save),
    CLI_COMMAND_DEF("set", "change setting", "[<cmd_name>=<value>]", cli_cmd_set),
    CLI_COMMAND_DEF("stats", "print flight stats", "<flight_number>", cli_cmd_parse_stats),
    CLI_COMMAND_DEF("status", "show status", NULL, cli_cmd_status),
    CLI_COMMAND_DEF("version", "show version", NULL, cli_cmd_version),
};

const size_t NUM_CLI_COMMANDS = sizeof cmd_table / sizeof cmd_table[0];

/** Helper function declarations **/

static void print_action_config();

static void print_timer_config();

static void cli_set_var(const cli_value_t *var, uint32_t value);

static void fill_buf(uint8_t *buf, size_t buf_sz);

/** CLI command function definitions **/

static void cli_cmd_help(const char *cmd_name, char *args) {
  bool any_matches = false;

  for (uint32_t i = 0; i < ARRAYLEN(cmd_table); i++) {
    bool print_entry = false;
    if (is_empty(args)) {
      print_entry = true;
    } else {
      if (strstr(cmd_table[i].name, args) || strstr(cmd_table[i].description, args)) {
        print_entry = true;
      }
    }

    if (print_entry) {
      any_matches = true;
      cli_print(cmd_table[i].name);
      if (cmd_table[i].description) {
        cli_printf(" - %s", cmd_table[i].description);
      }
      if (cmd_table[i].args) {
        cli_printf("\r\n\t%s", cmd_table[i].args);
      }
      cli_print_linefeed();
    }
  }
  if (!is_empty(args) && !any_matches) {
    cli_print_error_linef(cmd_name, "NO MATCHES FOR '%s'", args);
  }
}

static void cli_cmd_reboot(const char *cmd_name, char *args) { NVIC_SystemReset(); }

static void cli_cmd_save(const char *cmd_name, char *args) {
  if (cc_save() == false) {
    cli_print_line("Saving unsuccessful, trying force save...");
    if (cc_format_save() == false) {
      cli_print_line("Force save failed!");
      return;
    }
  }
  cli_print_line("Successfully written to flash");
}

static void cli_cmd_get(const char *cmd_name, char *args) {
  const cli_value_t *val;
  int matched_commands = 0;

  for (uint32_t i = 0; i < value_table_entry_count; i++) {
    if (strstr(value_table[i].name, args)) {
      val = &value_table[i];
      if (matched_commands > 0) {
        cli_print_linefeed();
      }
      cli_printf("%s = ", value_table[i].name);
      cli_print_var(cmd_name, val, 0);
      cli_print_linefeed();
      cli_print_var_range(val);
      // cliPrintVarDefault(cmd_name, val);

      matched_commands++;
    }
  }

  if (!matched_commands) {
    cli_print_error_linef(cmd_name, "INVALID NAME");
  }
}

static void cli_cmd_set(const char *cmd_name, char *args) {
  const uint32_t len = strlen(args);
  char *eqptr;

  if (len == 0 || (len == 1 && args[0] == '*')) {
    cli_print_line("Current settings: ");

    for (uint32_t i = 0; i < value_table_entry_count; i++) {
      const cli_value_t *val = &value_table[i];
      cli_printf("%s = ", value_table[i].name);
      // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
      cli_print_var(cmd_name, val, len);
      cli_print_linefeed();
    }
  } else if ((eqptr = strstr(args, "=")) != NULL) {
    // has equals

    uint8_t variable_name_length = get_word_length(args, eqptr);

    // skip the '=' and any ' ' characters
    eqptr++;
    eqptr = skip_space(eqptr);

    const uint16_t index = cli_get_setting_index(args, variable_name_length);
    if (index >= value_table_entry_count) {
      cli_print_error_linef(cmd_name, "INVALID NAME");
      return;
    }
    const cli_value_t *val = &value_table[index];

    bool value_changed = false;

    switch (val->type & VALUE_MODE_MASK) {
      case MODE_DIRECT: {
        if ((val->type & VALUE_TYPE_MASK) == VAR_UINT32) {
          uint32_t value = strtoul(eqptr, NULL, 10);

          if (value <= val->config.u32_max) {
            cli_set_var(val, value);
            value_changed = true;
          }
        } else {
          int value = atoi(eqptr);

          int min;
          int max;
          get_min_max(val, &min, &max);

          if (value >= min && value <= max) {
            cli_set_var(val, value);
            value_changed = true;
          }
        }
      }

      break;
      case MODE_LOOKUP:
      case MODE_BITSET: {
        int tableIndex;
        if ((val->type & VALUE_MODE_MASK) == MODE_BITSET) {
          tableIndex = TABLE_BOOTSTATE;
        } else {
          tableIndex = val->config.lookup.table_index;
        }
        const lookup_table_entry_t *tableEntry = &lookup_tables[tableIndex];
        bool matched = false;
        for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->value_count && !matched; tableValueIndex++) {
          matched = tableEntry->values[tableValueIndex] && strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

          if (matched) {
            cli_set_var(val, tableValueIndex);
            value_changed = true;
          }
        }
      } break;
      case MODE_ARRAY: {
        const uint8_t array_length = val->config.array.length;
        char *valPtr = eqptr;

        int i = 0;
        while (i < array_length && valPtr != NULL) {
          // skip spaces
          valPtr = skip_space(valPtr);

          // process substring starting at valPtr
          // note: no need to copy substrings for atoi()
          //       it stops at the first character that cannot be converted...
          switch (val->type & VALUE_TYPE_MASK) {
            default:
            case VAR_UINT8: {
              // fetch data pointer
              uint8_t *data = (uint8_t *)val->pdata + i;
              // store value
              *data = (uint8_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_INT8: {
              // fetch data pointer
              int8_t *data = (int8_t *)val->pdata + i;
              // store value
              *data = (int8_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_UINT16: {
              // fetch data pointer
              uint16_t *data = (uint16_t *)val->pdata + i;
              // store value
              *data = (uint16_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_INT16: {
              // fetch data pointer
              int16_t *data = (int16_t *)val->pdata + i;
              // store value
              *data = (int16_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_UINT32: {
              // fetch data pointer
              uint32_t *data = (uint32_t *)val->pdata + i;
              // store value
              *data = (uint32_t)strtoul((const char *)valPtr, NULL, 10);
            }

            break;
          }

          // find next comma (or end of string)
          valPtr = strchr(valPtr, ',') + 1;

          i++;
        }
      }
        // mark as changed
        value_changed = true;

        break;
    }

    if (value_changed) {
      cli_printf("%s set to ", val->name);
      cli_print_var(cmd_name, val, 0);
    } else {
      cli_print_error_linef(cmd_name, "INVALID VALUE");
      cli_print_var_range(val);
    }

    return;
  } else {
    // no equals, check for matching variables.
    cli_cmd_get(cmd_name, args);
  }
}

static void cli_cmd_config(const char *cmd_name, char *args) {
  print_action_config();
  print_timer_config();
}

static void cli_cmd_defaults(const char *cmd_name, char *args) {
  cc_defaults();
  cli_print_line("Reset to default values");
}

static void cli_cmd_status(const char *cmd_name, char *args) {
  const lookup_table_entry_t *p_boot_table = &lookup_tables[TABLE_BOOTSTATE];
  const lookup_table_entry_t *p_event_table = &lookup_tables[TABLE_EVENTS];
  cli_printf("Mode:\t%s\n", p_boot_table->values[global_cats_config.config.boot_state]);
  cli_printf("State:\t%s\n", p_event_table->values[global_flight_state.flight_state - 1]);
  cli_printf("Voltage: %.2fV\n", (double)battery_voltage());
  cli_printf("h: %.2fm, v: %.2fm/s, a: %.2fm/s^2", (double)global_estimation_data.height, (double)global_estimation_data.velocity,
             (double)global_estimation_data.acceleration);
}

static void cli_cmd_version(const char *cmd_name, char *args) {
  /* TODO: Store the board name somewhere else. */
  cli_printf("Board: %s\n", "CATS v2");
  cli_printf("CPU ID: 0x%lx, Revision: 0x%lx\n", HAL_GetDEVID(), HAL_GetREVID());
  cli_printf("Code version: %s\n", code_version);
}

static void cli_cmd_log_enable(const char *cmd_name, char *args) { log_enable(); }

static void cli_cmd_ls(const char *cmd_name, char *args) { lfs_ls(cwd); }

static void cli_cmd_cd(const char *cmd_name, char *args) {
  /* TODO - check if a directory actually exists */
  if (args == NULL || strcmp(args, "/") == 0) {
    strncpy(cwd, "/", sizeof(cwd));
  } else if (strcmp(args, "..") == 0) {
    /* return one lvl back */
  } else if (strcmp(args, ".") != 0) {
    if (args[0] == '/') {
      /* absolute path */
      strncpy(cwd, args, sizeof(cwd));
    } else {
      /* relative path */
      strncat(cwd, args, sizeof(cwd) - strlen(cwd) - 1);
    }
  }
}

static void cli_cmd_rm(const char *cmd_name, char *args) {
  if (args != NULL) {
    if (strlen(args) > LFS_NAME_MAX) {
      cli_print_line("File cmd_name too long!");
      return;
    }
    /* first +1 for the path separator (/), second +1 for the null terminator */
    char *full_path = malloc(strlen(cwd) + 1 + strlen(args) + 1);
    strcpy(full_path, cwd);
    strcat(full_path, "/");
    strcat(full_path, args);
    struct lfs_info info;
    int32_t stat_err = lfs_stat(&lfs, full_path, &info);
    if (stat_err < 0) {
      cli_print_linef("lfs_stat failed with %ld", stat_err);
      free(full_path);
      return;
    }
    if (info.type != LFS_TYPE_REG) {
      cli_print_line("This is not a file!");
      free(full_path);
      return;
    }
    int32_t rm_err = lfs_remove(&lfs, full_path);
    if (rm_err < 0) {
      cli_print_linef("File removal failed with %ld", rm_err);
    }
    cli_printf("File %s removed!", args);
    free(full_path);
  } else {
    cli_print_line("Argument not provided!");
  }
}

static void cli_cmd_dump_flight(const char *cmd_name, char *args) {
  /* TODO - count how many files in a directory here */
  char *endptr;
  uint32_t flight_idx = strtoul(args, &endptr, 10);

  if (args != endptr) {
    // A number was found
    if (flight_idx > flight_counter) {
      cli_print_linef("\nFlight %lu doesn't exist", flight_idx);
      cli_print_linef("Number of recorded flights: %lu", flight_counter);
    } else {
      cli_print_linefeed();
      dump_recording(flight_idx);
    }
  } else {
    cli_print_line("\nArgument not provided!");
  }
}

static void cli_cmd_parse_flight(const char *cmd_name, char *args) {
  /* TODO - count how many files in a directory here */
  char *endptr;
  uint32_t flight_idx = strtoul(args, &endptr, 10);

  if (args != endptr) {
    // A number was found
    if (flight_idx > flight_counter) {
      cli_print_linef("\nFlight %lu doesn't exist", flight_idx);
      cli_print_linef("Number of recorded flights: %lu", flight_counter);
    } else {
      cli_print_linefeed();
      parse_recording(flight_idx);
    }
  } else {
    cli_print_line("\nArgument not provided!");
  }
}
static void cli_cmd_parse_stats(const char *cmd_name, char *args) {
  /* TODO - count how many files in a directory here */
  char *endptr;
  uint32_t flight_idx = strtoul(args, &endptr, 10);

  if (args != endptr) {
    // A number was found
    if (flight_idx > flight_counter) {
      cli_print_linef("\nFlight %lu doesn't exist", flight_idx);
      cli_print_linef("Number of recorded flights: %lu", flight_counter);
    } else {
      cli_print_linefeed();
      parse_stats(flight_idx);
    }
  } else {
    cli_print_line("\nArgument not provided!");
  }
}

static void cli_cmd_lfs_format(const char *cmd_name, char *args) {
  cli_print_line("\nTrying LFS format");
  lfs_format(&lfs, &lfs_cfg);
  int err = lfs_mount(&lfs, &lfs_cfg);
  if (err != 0) {
    cli_print_linef("LFS mounting failed with error %d!", err);
  } else {
    cli_print_line("Mounting successful!");
    /* create the flights directory */
    lfs_mkdir(&lfs, "flights");
    lfs_mkdir(&lfs, "stats");

    strncpy(cwd, "/", sizeof(cwd));
  }
}

static void cli_cmd_erase_flash(const char *cmd_name, char *args) {
  cli_print_line("\nErasing the flash, this might take a while...");
  w25q_chip_erase();
  cli_print_line("Flash erased!");
  cli_print_line("Mounting LFS");

  int err = lfs_mount(&lfs, &lfs_cfg);
  if (err == 0) {
    cli_print_line("LFS mounted successfully!");
  } else {
    cli_print_linef("LFS mounting failed with error %d!", err);
    cli_print_line("Trying LFS format");
    lfs_format(&lfs, &lfs_cfg);
    int err2 = lfs_mount(&lfs, &lfs_cfg);
    if (err2 != 0) {
      cli_print_linef("LFS mounting failed again with error %d!", err2);
      return;
    } else {
      cli_print_line("Mounting successful!");
    }
  }
  flight_counter = 0;
  /* create the flights directory */
  lfs_mkdir(&lfs, "flights");
  lfs_mkdir(&lfs, "stats");

  strncpy(cwd, "/", sizeof(cwd));
}

static void cli_cmd_erase_recordings(const char *cmd_name, char *args) {
  cli_print_line("\nErasing the flight recordings, this might not take much...");
  erase_recordings();
  cli_print_line("Recordings erased!");
}

static void cli_cmd_flash_write(const char *cmd_name, char *args) {
  cli_print_line("\nSetting recorder state to REC_WRITE_TO_FLASH");
  set_recorder_state(REC_WRITE_TO_FLASH);
}

static void cli_cmd_flash_stop(const char *cmd_name, char *args) {
  cli_print_line("\nSetting recorder state to REC_FILL_QUEUE");
  set_recorder_state(REC_FILL_QUEUE);
}

static void cli_cmd_flash_test(const char *cmd_name, char *args) {
  uint8_t write_buf[256] = {0};
  uint8_t read_buf[256] = {0};
  fill_buf(write_buf, 256);
  cli_print_line("\nStep 1: Erasing the chip sector by sector...");
  w25q_chip_erase();
  for (uint32_t i = 0; i < w25q.sector_count; ++i) {
    if (i % 100 == 0) {
      cli_print_linef("%lu / %lu sectors erased...", i, w25q.sector_count);
    }
    w25q_status_e sector_erase_status = w25q_sector_erase(i);
    if (sector_erase_status != W25Q_OK) {
      cli_print_linef("Sector erase error encountered at sector %lu; status %d", i, sector_erase_status);
      osDelay(5000);
    }
  }
  cli_print_line("Step 2: Sequential write test");
  for (uint32_t i = 0; i < w25q.page_count; ++i) {
    if (i % 100 == 0) {
      cli_print_linef("%lu / %lu pages written...", i, w25q.page_count);
    }
    w25q_status_e write_status = w25q_write_buffer(write_buf, i * w25q.page_size, 256);
    if (write_status != W25Q_OK) {
      cli_print_linef("Write error encountered at page %lu; status %d", i, write_status);
      osDelay(5000);
    }
  }
  cli_print_line("Step 3: Sequential read test");
  for (uint32_t i = 0; i < w25q.page_count; ++i) {
    memset(read_buf, 0, 256);
    if (i % 100 == 0) {
      cli_print_linef("%lu / %lu pages read...", i, w25q.page_count);
    }
    w25q_status_e read_status = w25q_read_buffer(read_buf, i * w25q.page_size, 256);
    if (read_status != W25Q_OK) {
      cli_print_linef("Read error encountered at page %lu; status %d", i, read_status);
      osDelay(5000);
    }
    if (memcmp(write_buf, read_buf, 256) != 0) {
      cli_print_linef("Buffer mismatch at page %lu", i);
      osDelay(5000);
    }
  }
  cli_print_line("Test complete!");
}

/**  Helper function definitions **/

static void print_action_config() {
  const lookup_table_entry_t *p_event_table = &lookup_tables[TABLE_EVENTS];
  const lookup_table_entry_t *p_action_table = &lookup_tables[TABLE_ACTIONS];

  cli_printf("\n * ACTION CONFIGURATION *\n");
  config_action_t action;
  for (int i = 0; i < NUM_EVENTS; i++) {
    int nr_actions = cc_get_num_actions(i);
    if (nr_actions > 0) {
      cli_printf("\n%s\n", p_event_table->values[i]);
      cli_printf("   Number of Actions: %d\n", nr_actions);
      for (int j = 0; j < nr_actions; j++) {
        cc_get_action(i, j, &action);
        cli_printf("     %s - %d\n", p_action_table->values[action.action_idx], action.arg);
      }
    }
  }
}

static void print_timer_config() {
  const lookup_table_entry_t *p_event_table = &lookup_tables[TABLE_EVENTS];

  cli_printf("\n\n * TIMER CONFIGURATION *\n");
  for (int i = 0; i < NUM_TIMERS; i++) {
    if (global_cats_config.config.timers[i].duration > 0) {
      cli_printf("\nTIMER %d\n", i + 1);
      cli_printf("  Start: %s\n", p_event_table->values[global_cats_config.config.timers[i].start_event]);
      cli_printf("  End: %s\n", p_event_table->values[global_cats_config.config.timers[i].end_event]);
      cli_printf("  Duration: %lu ms\n", global_cats_config.config.timers[i].duration);
    }
  }
}

static void cli_set_var(const cli_value_t *var, const uint32_t value) {
  void *ptr = var->pdata;
  uint32_t work_value;
  uint32_t mask;

  if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
    switch (var->type & VALUE_TYPE_MASK) {
      case VAR_UINT8:
        mask = (1 << var->config.bitpos) & 0xff;
        if (value) {
          work_value = *(uint8_t *)ptr | mask;
        } else {
          work_value = *(uint8_t *)ptr & ~mask;
        }
        *(uint8_t *)ptr = work_value;
        break;

      case VAR_UINT16:
        mask = (1 << var->config.bitpos) & 0xffff;
        if (value) {
          work_value = *(uint16_t *)ptr | mask;
        } else {
          work_value = *(uint16_t *)ptr & ~mask;
        }
        *(uint16_t *)ptr = work_value;
        break;

      case VAR_UINT32:
        mask = 1 << var->config.bitpos;
        if (value) {
          work_value = *(uint32_t *)ptr | mask;
        } else {
          work_value = *(uint32_t *)ptr & ~mask;
        }
        *(uint32_t *)ptr = work_value;
        break;
    }
  } else {
    switch (var->type & VALUE_TYPE_MASK) {
      case VAR_UINT8:
        *(uint8_t *)ptr = value;
        break;

      case VAR_INT8:
        *(int8_t *)ptr = value;
        break;

      case VAR_UINT16:
        *(uint16_t *)ptr = value;
        break;

      case VAR_INT16:
        *(int16_t *)ptr = value;
        break;

      case VAR_UINT32:
        *(uint32_t *)ptr = value;
        break;
    }
  }
}

static void fill_buf(uint8_t *buf, size_t buf_sz) {
  for (uint32_t i = 0; i < buf_sz / 2; ++i) {
    buf[i] = i * 2;
    buf[buf_sz - i - 1] = i * 2 + 1;
  }
}
