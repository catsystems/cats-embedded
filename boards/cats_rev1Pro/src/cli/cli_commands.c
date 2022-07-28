/*
 * This file was adapted from Cleanflight and Betaflight.
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

#include "cli/cli_commands.h"

#include "cli/cli.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "drivers/w25q.h"
#include "flash/lfs_custom.h"
#include "flash/reader.h"
#include "util/actions.h"
#include "util/battery.h"
#include "util/log.h"

#include <stdlib.h>
#include <string.h>
#ifdef CATS_DEBUG
#include "tasks/task_simulator.h"
#endif

/** CLI command function declarations **/
static void cli_cmd_help(const char *cmd_name, char *args);

static void cli_cmd_reboot(const char *cmd_name, char *args);
static void cli_cmd_bl(const char *cmd_name, char *args);
static void cli_cmd_save(const char *cmd_name, char *args);

static void cli_cmd_get(const char *cmd_name, char *args);
static void cli_cmd_set(const char *cmd_name, char *args);
static void cli_cmd_config(const char *cmd_name, char *args);
static void cli_cmd_defaults(const char *cmd_name, char *args);
static void cli_cmd_dump(const char *cmd_name, char *args);

static void cli_cmd_status(const char *cmd_name, char *args);
static void cli_cmd_version(const char *cmd_name, char *args);

static void cli_cmd_log_enable(const char *cmd_name, char *args);

static void cli_cmd_ls(const char *cmd_name, char *args);
static void cli_cmd_cd(const char *cmd_name, char *args);
static void cli_cmd_rm(const char *cmd_name, char *args);
static void cli_cmd_rec_info(const char *cmd_name, char *args);

static void cli_cmd_dump_flight(const char *cmd_name, char *args);
static void cli_cmd_parse_flight(const char *cmd_name, char *args);
static void cli_cmd_parse_stats(const char *cmd_name, char *args);

static void cli_cmd_lfs_format(const char *cmd_name, char *args);
static void cli_cmd_erase_flash(const char *cmd_name, char *args);

static void cli_cmd_flash_write(const char *cmd_name, char *args);
static void cli_cmd_flash_stop(const char *cmd_name, char *args);
static void cli_cmd_flash_test(const char *cmd_name, char *args);

#ifdef CATS_DEBUG
static void cli_cmd_start_simulation(const char *cmd_name, char *args);
#endif

/* List of CLI commands; should be sorted in alphabetical order. */
const clicmd_t cmd_table[] = {
    CLI_COMMAND_DEF("bl", "reset into bootloader", NULL, cli_cmd_bl),
    CLI_COMMAND_DEF("cd", "change current working directory", NULL, cli_cmd_cd),
    CLI_COMMAND_DEF("config", "print the flight config", NULL, cli_cmd_config),
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cli_cmd_defaults),
    CLI_COMMAND_DEF("dump", "Dump configuration", NULL, cli_cmd_dump),
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
    CLI_COMMAND_DEF("rec_info", "get the info about flash", NULL, cli_cmd_rec_info),
    CLI_COMMAND_DEF("rm", "remove a file", "<file_name>", cli_cmd_rm),
    CLI_COMMAND_DEF("save", "save configuration", NULL, cli_cmd_save),
    CLI_COMMAND_DEF("set", "change setting", "[<cmd_name>=<value>]", cli_cmd_set),
#ifdef CATS_DEBUG
    CLI_COMMAND_DEF("sim", "start a simulation flight", "<sim_tag>", cli_cmd_start_simulation),
#endif
    CLI_COMMAND_DEF("stats", "print flight stats", "<flight_number>", cli_cmd_parse_stats),
    CLI_COMMAND_DEF("status", "show status", NULL, cli_cmd_status),
    CLI_COMMAND_DEF("version", "show version", NULL, cli_cmd_version),
};

const size_t NUM_CLI_COMMANDS = sizeof cmd_table / sizeof cmd_table[0];

static const char *const emptyName = "-";

/** Helper function declarations **/

static void print_control_config();

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

extern RTC_HandleTypeDef hrtc;
static void cli_cmd_bl(const char *cmd_name, char *args) {
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, BOOTLOADER_MAGIC_PATTERN);
  __disable_irq();
  NVIC_SystemReset();
}

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
      cli_print_var(cmd_name, &global_cats_config, val, 0);
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

    // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
    print_cats_config(cmd_name, &global_cats_config, len);

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

          const void *var_ptr = get_cats_config_member_ptr(&global_cats_config, val);

          // process substring starting at valPtr
          // note: no need to copy substrings for atoi()
          //       it stops at the first character that cannot be converted...
          switch (val->type & VALUE_TYPE_MASK) {
            default:
            case VAR_UINT8: {
              // fetch data pointer
              uint8_t *data = (uint8_t *)var_ptr + i;
              // store value
              *data = (uint8_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_INT8: {
              // fetch data pointer
              int8_t *data = (int8_t *)var_ptr + i;
              // store value
              *data = (int8_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_UINT16: {
              // fetch data pointer
              uint16_t *data = (uint16_t *)var_ptr + i;
              // store value
              *data = (uint16_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_INT16: {
              // fetch data pointer
              int16_t *data = (int16_t *)var_ptr + i;
              // store value
              *data = (int16_t)atoi((const char *)valPtr);
            }

            break;
            case VAR_UINT32: {
              // fetch data pointer
              uint32_t *data = (uint32_t *)var_ptr + i;
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
      case MODE_STRING: {
        char *val_ptr = eqptr;
        val_ptr = skip_space(val_ptr);
        const void *var_ptr = get_cats_config_member_ptr(&global_cats_config, val);
        const unsigned int len = strlen(val_ptr);
        const uint8_t min = val->config.string.min_length;
        const uint8_t max = val->config.string.max_length;
        const bool updatable = ((val->config.string.flags & STRING_FLAGS_WRITEONCE) == 0 || strlen(var_ptr) == 0 ||
                                strncmp(val_ptr, var_ptr, len) == 0);

        if (updatable && len > 0 && len <= max) {
          memset(var_ptr, 0, max);
          if (len >= min && strncmp(val_ptr, emptyName, len)) {
            strncpy(var_ptr, val_ptr, len);
          }
          value_changed = true;
        } else {
          cli_print_error_linef(cmd_name, "STRING MUST BE 1..%d CHARACTERS OR '-' FOR EMPTY", max);
        }
      } break;
    }

    if (value_changed) {
      cli_printf("%s set to ", val->name);
      cli_print_var(cmd_name, &global_cats_config, val, 0);
      if (val->cb != NULL) {
        val->cb(val);
      }
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
  print_control_config();
  print_action_config();
  print_timer_config();
}

static void cli_cmd_defaults(const char *cmd_name, char *args) {
  bool use_default_outputs = true;
  if (!strcmp(args, "--no-outputs")) {
    use_default_outputs = false;
  }
  cc_defaults(use_default_outputs);
  cli_print_linef("Reset to default values%s", use_default_outputs ? "" : " [no outputs]");
}

static void cli_cmd_dump(const char *cmd_name, char *args) {
  const uint32_t len = strlen(args);
  cli_print_linef("#Configuration dump");

  print_cats_config(cmd_name, &global_cats_config, len);

  cli_printf("#End of configuration dump");
}

static void cli_cmd_status(const char *cmd_name, char *args) {
  const lookup_table_entry_t *p_boot_table = &lookup_tables[TABLE_BOOTSTATE];
  const lookup_table_entry_t *p_event_table = &lookup_tables[TABLE_EVENTS];
  cli_printf("System time: %lu ticks\n", osKernelGetTickCount());
  cli_printf("Mode:        %s\n", p_boot_table->values[global_cats_config.config.boot_state]);
  cli_printf("State:       %s\n", p_event_table->values[global_flight_state.flight_state - 1]);
  cli_printf("Voltage:     %.2fV\n", (double)battery_voltage());
  cli_printf("h: %.2fm, v: %.2fm/s, a: %.2fm/s^2", (double)global_estimation_data.height,
             (double)global_estimation_data.velocity, (double)global_estimation_data.acceleration);

#ifdef CATS_DEBUG
  if (!strcmp(args, "--heap")) {
    HeapStats_t heap_stats = {};
    vPortGetHeapStats(&heap_stats);
    cli_print_linef("\nHeap stats");
    cli_print_linef("  Available heap space: %u B", heap_stats.xAvailableHeapSpaceInBytes);
    cli_print_linef("  Largest free block size: %u B", heap_stats.xSizeOfLargestFreeBlockInBytes);
    cli_print_linef("  Smallest free block size: %u B", heap_stats.xSizeOfSmallestFreeBlockInBytes);
    cli_print_linef("  Number of free blocks: %u", heap_stats.xNumberOfFreeBlocks);
    cli_print_linef("  Minimum free bytes remaining during program lifetime: %u B",
                    heap_stats.xMinimumEverFreeBytesRemaining);
    cli_print_linef("  Number of successful allocations: %u", heap_stats.xNumberOfSuccessfulAllocations);
    cli_print_linef("  Number of successful frees: %u", heap_stats.xNumberOfSuccessfulFrees);
  }
#endif
}

static void cli_cmd_version(const char *cmd_name, char *args) {
  /* TODO: Store the board name somewhere else. */
  cli_printf("Board: %s\n", "CATS v2");
  cli_printf("CPU ID: 0x%lx, Revision: 0x%lx\n", HAL_GetDEVID(), HAL_GetREVID());
  cli_printf("Code version: %s\n", code_version);
}

static void cli_cmd_log_enable(const char *cmd_name, char *args) { log_enable(); }

static void cli_cmd_ls(const char *cmd_name, char *args) {
  if (args == NULL) {
    lfs_ls(cwd);
  } else {
    uint32_t full_path_len = strlen(cwd) + 1 + strlen(args);
    if (full_path_len > LFS_NAME_MAX) {
      cli_print_line("File path too long!");
      return;
    }
    char *full_path = (char *)(pvPortMalloc(full_path_len + 1));
    strcpy(full_path, cwd);
    strcat(full_path, "/");
    strcat(full_path, args);
    lfs_ls(full_path);
    vPortFree(full_path);
  }
}

static void cli_cmd_cd(const char *cmd_name, char *args) {
  /* TODO - check if a directory actually exists */
  if (args == NULL || strcmp(args, "/") == 0) {
    strncpy(cwd, "/", sizeof(cwd));
  } else if (strcmp(args, "..") == 0) {
    /* Return one lvl back by clearing everything after the last path separator. */
    const char *last_path_sep = strrchr(cwd, '/');
    if (last_path_sep != NULL) {
      uint32_t last_path_sep_loc = last_path_sep - cwd;
      cwd[last_path_sep_loc + 1] = '\0';
    }
  } else if (strcmp(args, ".") != 0) {
    if (args[0] == '/') {
      /* absolute path */
      uint32_t full_path_len = strlen(args);
      if (full_path_len > LFS_NAME_MAX) {
        cli_print_line("Path too long!");
        return;
      }
      char *tmp_path = (char *)(pvPortMalloc(full_path_len + 1));
      strcpy(tmp_path, args);
      if (lfs_obj_type(tmp_path) != LFS_TYPE_DIR) {
        cli_print_linef("Cannot go to '%s': not a directory!", tmp_path);
        vPortFree(tmp_path);
        return;
      }
      strncpy(cwd, args, sizeof(cwd));
      vPortFree(tmp_path);
    } else {
      /* relative path */
      uint32_t full_path_len = strlen(cwd) + 1 + strlen(args);
      if (full_path_len > LFS_NAME_MAX) {
        cli_print_line("Path too long!");
        return;
      }
      char *tmp_path = (char *)(pvPortMalloc(full_path_len + 1));
      strcpy(tmp_path, args);
      if (lfs_obj_type(tmp_path) != LFS_TYPE_DIR) {
        cli_print_linef("Cannot go to '%s': not a directory!", tmp_path);
        vPortFree(tmp_path);
        return;
      }
      strncat(cwd, args, sizeof(cwd) - strlen(cwd) - 1);
      vPortFree(tmp_path);
    }
  }
}

static void cli_cmd_rm(const char *cmd_name, char *args) {
  if (args != NULL) {
    /* +1 for the path separator (/) */
    uint32_t full_path_len = strlen(cwd) + 1 + strlen(args);
    if (full_path_len > LFS_NAME_MAX) {
      cli_print_line("File path too long!");
      return;
    }
    /* +1 for the null terminator */
    char *full_path = (char *)(pvPortMalloc(full_path_len + 1));
    strcpy(full_path, cwd);
    strcat(full_path, "/");
    strcat(full_path, args);

    if (lfs_obj_type(full_path) != LFS_TYPE_REG) {
      cli_print_linef("Cannot remove '%s': not a file!", full_path);
      vPortFree(full_path);
      return;
    }

    int32_t rm_err = lfs_remove(&lfs, full_path);
    if (rm_err < 0) {
      cli_print_linef("Removal of file '%s' failed with %ld", full_path, rm_err);
    }
    cli_printf("File '%s' removed!", args);
    vPortFree(full_path);
  } else {
    cli_print_line("Argument not provided!");
  }
}

static void cli_cmd_rec_info(const char *cmd_name, char *args) {
  const lfs_ssize_t curr_sz_blocks = lfs_fs_size(&lfs);
  const int32_t num_flights = lfs_cnt("/flights", LFS_TYPE_REG);
  const int32_t num_stats = lfs_cnt("/stats", LFS_TYPE_REG);

  if ((curr_sz_blocks < 0) || (num_flights < 0) || (num_stats < 0)) {
    cli_print_line("Error while accessing recorder info.");
    return;
  }

  const lfs_size_t block_size_kb = get_lfs_cfg()->block_size / 1024;
  const lfs_size_t curr_sz_kb = curr_sz_blocks * block_size_kb;
  const lfs_size_t total_sz_kb = block_size_kb * get_lfs_cfg()->block_count;
  const double percentage_used = (double)curr_sz_kb / total_sz_kb * 100;
  cli_print_linef("Space:\n  Total: %lu KB\n   Used: %lu KB (%.2f%%)\n   Free: %lu KB (%.2f%%)", total_sz_kb,
                  curr_sz_kb, percentage_used, total_sz_kb - curr_sz_kb, 100 - percentage_used);

  cli_print_linef("Number of flight logs: %ld", num_flights);
  cli_print_linef("Number of stats logs: %ld", num_stats);
}

/**
 * Parse the log index argument string and return it as a number.
 *
 * The function supports tail indexing: -1, -2, -3..., where -1 is the last log, -2 the one before it, etc.
 *
 *
 * @param log_idx_arg
 * @return
 */
static int32_t get_flight_idx(const char *log_idx_arg) {
  if (log_idx_arg == NULL) {
    cli_print_line("\nArgument not provided!");
    return -1;
  }

  char *endptr;
  int32_t flight_idx = strtol(log_idx_arg, &endptr, 10);

  if (log_idx_arg == endptr) {
    cli_print_linef("\nInvalid argument: %s.", log_idx_arg);
    return -1;
  }

  /* Check for tail indexing */
  if (flight_idx < 0) {
    /* Convert to "normal" index */
    flight_idx = flight_counter + 1 + flight_idx;
  }

  if (flight_idx <= 0) {
    cli_print_linef("\nInvalid flight: %s.", log_idx_arg);
    return -1;
  }

  if (flight_idx > flight_counter) {
    cli_print_linef("\nFlight %lu doesn't exist", flight_idx);
    cli_print_linef("Number of recorded flights: %lu", flight_counter);
    return -1;
  }

  return flight_idx;
}

static void cli_cmd_dump_flight(const char *cmd_name, char *args) {
  int32_t flight_idx_or_err = get_flight_idx(args);

  if (flight_idx_or_err > 0) {
    cli_print_linefeed();
    dump_recording(flight_idx_or_err);
  }
}

/* flight_parse <flight_idx> [--filter <RECORDER TYPE>...] */
static void cli_cmd_parse_flight(const char *cmd_name, char *args) {
  char *ptr = strtok(args, " ");

  int32_t flight_idx_or_err = get_flight_idx(ptr);
  rec_entry_type_e filter_mask = 0;

  if (flight_idx_or_err < 0) {
    return;
  }

  /* Read filter command */
  ptr = strtok(NULL, " ");
  if (ptr != NULL) {
    if (!strcmp(ptr, "--filter")) {
      /*Read filter types */
      while (ptr != NULL) {
        if (!strcmp(ptr, "IMU")) filter_mask |= IMU;
        if (!strcmp(ptr, "BARO")) filter_mask |= BARO;
        if (!strcmp(ptr, "MAGNETO")) filter_mask |= MAGNETO;
        if (!strcmp(ptr, "ACCELEROMETER")) filter_mask |= ACCELEROMETER;
        if (!strcmp(ptr, "FLIGHT_INFO")) filter_mask |= FLIGHT_INFO;
        if (!strcmp(ptr, "ORIENTATION_INFO")) filter_mask |= ORIENTATION_INFO;
        if (!strcmp(ptr, "FILTERED_DATA_INFO")) filter_mask |= FILTERED_DATA_INFO;
        if (!strcmp(ptr, "FLIGHT_STATE")) filter_mask |= FLIGHT_STATE;
        if (!strcmp(ptr, "EVENT_INFO")) filter_mask |= EVENT_INFO;
        if (!strcmp(ptr, "ERROR_INFO")) filter_mask |= ERROR_INFO;
        ptr = strtok(NULL, " ");
      }
    } else {
      cli_print_linef("\nBad option: %s!", ptr);
    }
  } else {
    filter_mask = UINT32_MAX;
  }

  parse_recording(flight_idx_or_err, filter_mask);
}

static void cli_cmd_parse_stats(const char *cmd_name, char *args) {
  int32_t flight_idx_or_err = get_flight_idx(args);

  if (flight_idx_or_err > 0) {
    cli_print_linefeed();
    parse_stats(flight_idx_or_err);
  }
}

static void cli_cmd_lfs_format(const char *cmd_name, char *args) {
  cli_print_line("\nTrying LFS format");
  lfs_format(&lfs, get_lfs_cfg());
  int err = lfs_mount(&lfs, get_lfs_cfg());
  if (err != 0) {
    cli_print_linef("LFS mounting failed with error %d!", err);
  } else {
    cli_print_line("Mounting successful!");
    flight_counter = 0;
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

  int err = lfs_mount(&lfs, get_lfs_cfg());
  if (err == 0) {
    cli_print_line("LFS mounted successfully!");
  } else {
    cli_print_linef("LFS mounting failed with error %d!", err);
    cli_print_line("Trying LFS format");
    lfs_format(&lfs, get_lfs_cfg());
    int err2 = lfs_mount(&lfs, get_lfs_cfg());
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
  // w25q_chip_erase();
  if (!strcmp(args, "full")) {
    cli_print_line("\nStep 1: Erasing the chip sector by sector...");
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
#if defined(CATS_ORION)
      w25q_status_e write_status = w25q_write_buffer(write_buf, i * w25q.page_size, 256);
#elif defined(CATS_VEGA)
      w25q_status_e write_status = w25qxx_write_page(write_buf, i, 0, 256);
#endif
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
#if defined(CATS_ORION)
      w25q_status_e read_status = w25q_read_buffer(read_buf, i * w25q.page_size, 256);
#elif defined(CATS_VEGA)

      w25q_status_e read_status = w25qxx_read_page(read_buf, i, 0, 256);
#endif
      if (read_status != W25Q_OK) {
        cli_print_linef("Read error encountered at page %lu; status %d", i, read_status);
        osDelay(1);
      }
      if (memcmp(write_buf, read_buf, 256) != 0) {
        cli_print_linef("Buffer mismatch at page %lu", i);
        osDelay(1);
      }
    }

    cli_print_line("\nStep 4: Erasing the chip sector by sector...");
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
  } else {
    char *endptr;
    uint32_t sector_idx = strtoul(args, &endptr, 10);
    if (args != endptr) {
      if (sector_idx >= w25q.sector_count) {
        cli_print_linef("Sector %lu not found!", sector_idx);
        return;
      }

      cli_print_linef("\nStep 1: Erasing sector %lu", sector_idx);
      w25q_status_e sector_erase_status = w25q_sector_erase(sector_idx);
      if (sector_erase_status != W25Q_OK) {
        cli_print_linef("Sector erase error encountered at sector %lu; status %d", sector_idx, sector_erase_status);
        osDelay(5000);
      }

      const uint32_t start_page_idx = w25q_sector_to_page(sector_idx);
      const uint32_t pages_per_sector = w25q.sector_size / w25q.page_size;
      const uint32_t end_page_idx = start_page_idx + pages_per_sector - 1;
      cli_print_linef("Step 2: Sequential write test (start_page: %lu, end_page: %lu)", start_page_idx, end_page_idx);
      for (uint32_t i = start_page_idx; i <= end_page_idx; ++i) {
        if (i % 4 == 0) {
          cli_print_linef("%lu / %lu pages written...", i - start_page_idx, pages_per_sector);
        }
#if defined(CATS_ORION)
        w25q_status_e write_status = w25q_write_buffer(write_buf, i * w25q.page_size, 256);
#elif defined(CATS_VEGA)
        w25q_status_e write_status = w25qxx_write_page(write_buf, i, 0, 256);
#endif
        if (write_status != W25Q_OK) {
          cli_print_linef("Write error encountered at page %lu; status %d", i, write_status);
          osDelay(5000);
        }
      }

      cli_print_linef("Step 3: Sequential read test (start_page: %lu, end_page: %lu)", start_page_idx, end_page_idx);
      for (uint32_t i = start_page_idx; i <= end_page_idx; ++i) {
        memset(read_buf, 0, 256);
        if (i % 4 == 0) {
          cli_print_linef("%lu / %lu pages read...", i - start_page_idx, pages_per_sector);
        }
#if defined(CATS_ORION)
        w25q_status_e read_status = w25q_read_buffer(read_buf, i * w25q.page_size, 256);
#elif defined(CATS_VEGA)

        w25q_status_e read_status = w25qxx_read_page(read_buf, i, 0, 256);
#endif
        if (read_status != W25Q_OK) {
          cli_print_linef("Read error encountered at page %lu; status %d", i, read_status);
          osDelay(1);
        }
        if (memcmp(write_buf, read_buf, 256) != 0) {
          cli_print_linef("Buffer mismatch at page %lu", i);
          osDelay(1);
        }
      }

      cli_print_linef("\nStep 4: Erasing sector %lu...", sector_idx);
      sector_erase_status = w25q_sector_erase(sector_idx);
      if (sector_erase_status != W25Q_OK) {
        cli_print_linef("Sector erase error encountered at sector %lu; status %d", sector_idx, sector_erase_status);
        osDelay(5000);
      }
    }
  }
  cli_print_line("Test complete!");
}

#ifdef CATS_DEBUG
static void cli_cmd_start_simulation(const char *cmd_name, char *args) { start_simulation(args); }
#endif

/**  Helper function definitions **/

static void print_control_config() {
  cli_print_line("\n * CONTROL SETTINGS *\n");

  cli_printf("  Liftoff Acc. Threshold: %u m/s^2\n", global_cats_config.config.control_settings.liftoff_acc_threshold);
  cli_printf("  Main Altitude:          %u m\n", global_cats_config.config.control_settings.main_altitude);
  cli_printf("  Mach Timer Duration:    %u ms\n", global_cats_config.config.control_settings.mach_timer_duration);
}

static void print_action_config() {
  const lookup_table_entry_t *p_event_table = &lookup_tables[TABLE_EVENTS];
  const lookup_table_entry_t *p_action_table = &lookup_tables[TABLE_ACTIONS];

  cli_printf("\n * ACTION CONFIGURATION *\n");
  config_action_t action;
  for (int i = 0; i < NUM_EVENTS; i++) {
    int nr_actions = cc_get_num_actions((cats_event_e)(i));
    if (nr_actions > 0) {
      cli_printf("\n%s\n", p_event_table->values[i]);
      cli_printf("   Number of Actions: %d\n", nr_actions);
      for (int j = 0; j < nr_actions; j++) {
        cc_get_action((cats_event_e)(i), j, &action);
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
      cli_printf("  Start:    %s\n", p_event_table->values[global_cats_config.config.timers[i].start_event]);
      cli_printf("  Trigger:  %s\n", p_event_table->values[global_cats_config.config.timers[i].trigger_event]);
      cli_printf("  Duration: %lu ms\n", global_cats_config.config.timers[i].duration);
    }
  }
}

static void cli_set_var(const cli_value_t *var, const uint32_t value) {
  const void *ptr = get_cats_config_member_ptr(&global_cats_config, var);

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
