///*
// * cli.c
// *
// *  Created on: 3 May 2021
// *      Author: Luca
// */
//

#include "cli/cli.h"
#include "cli/settings.h"
#include "util/log.h"
#include "util/reader.h"
#include "config/cats_config.h"
#include "config/globals.h"
#include "lfs/lfs_custom.h"
#include "util/actions.h"
#include "util/battery.h"
#include "drivers/w25q.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#define CLI_IN_BUFFER_SIZE  256
#define CLI_OUT_BUFFER_SIZE 256

static uint32_t bufferIndex = 0;

static char cliBuffer[CLI_IN_BUFFER_SIZE];
static char oldcliBuffer[CLI_IN_BUFFER_SIZE];

static fifo_t *cli_in;
static fifo_t *cli_out;

typedef void cliCommandFn(const char *name, char *cmdline);

typedef struct {
  const char *name;
  const char *description;
  const char *args;
  cliCommandFn *cliCommand;
} clicmd_t;

#define CLI_COMMAND_DEF(name, description, args, cliCommand) \
  { name, description, args, cliCommand }

static bool isEmpty(const char *string) { return (string == NULL || *string == '\0') ? true : false; }

static void getMinMax(const clivalue_t *var, int *min, int *max) {
  switch (var->type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
    case VAR_UINT16:
      *min = var->config.minmaxUnsigned.min;
      *max = var->config.minmaxUnsigned.max;

      break;
    default:
      *min = var->config.minmax.min;
      *max = var->config.minmax.max;

      break;
  }
}

static void cliDefaults(const char *cmdName, char *cmdline);
static void cliHelp(const char *cmdName, char *cmdline);
static void cliSave(const char *cmdName, char *cmdline);
static void cliDump(const char *cmdName, char *cmdline);
static void cliExit(const char *cmdName, char *cmdline);
static void cliGet(const char *cmdName, char *cmdline);
static void cliSet(const char *cmdName, char *cmdline);
static void cliStatus(const char *cmdName, char *cmdline);
static void cliVersion(const char *cmdName, char *cmdline);
static void cliConfig(const char *cmdName, char *cmdline);
static void cliEraseFlash(const char *cmdName, char *cmdline);
static void cliEraseRecordings(const char *cmdName, char *cmdline);
static void cliRecInfo(const char *cmdName, char *cmdline);
static void cliPrintFlight(const char *cmdName, char *cmdline);
static void cliFlashWrite(const char *cmdName, char *cmdline);
static void cliFlashStop(const char *cmdName, char *cmdline);
static void cliLfsFormat(const char *cmdName, char *cmdline);
static void cliLs(const char *cmdName, char *cmdline);
static void cliCd(const char *cmdName, char *cmdline);

void cliPrint(const char *str);
void cliPrintLinefeed(void);
void cliPrintLine(const char *str);

static void cliPrintHashLine(const char *str);
static bool cliDumpPrintLinef(bool equalsDefault, const char *format, ...) __attribute__((format(printf, 2, 3)));
static void cliWrite(uint8_t ch);

void cliPrintf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void cliPrintLinef(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void cliPrintErrorVa(const char *cmdName, const char *format, va_list va);
static void cliPrintError(const char *cmdName, const char *format, ...) __attribute__((format(printf, 2, 3)));
static void cliPrintErrorLinef(const char *cmdName, const char *format, ...) __attribute__((format(printf, 2, 3)));
static void cliRead(const char *cmdName, char *cmdline);
static void cliEnable(const char *cmdName, char *cmdline);

const clicmd_t cmdTable[] = {
    // CLI_COMMAND_DEF("bl", "reboot into bootloader", "[rom]", cliBootloader),
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", "[nosave|show]", cliDefaults),
    CLI_COMMAND_DEF("dump", "dump configuration", "[master|profile|rates|hardware|all] {defaults|bare}", cliDump),
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
    CLI_COMMAND_DEF("help", "display command help", "[search string]", cliHelp),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
    CLI_COMMAND_DEF("read", "readout the flash", NULL, cliRead),
    CLI_COMMAND_DEF("flash_erase", "erase the flash", NULL, cliEraseFlash),
    CLI_COMMAND_DEF("rec_erase", "erase the recordings", NULL, cliEraseRecordings),
    CLI_COMMAND_DEF("rec_info", "get the flight recorder info", NULL, cliRecInfo),
    CLI_COMMAND_DEF("rec_print_flight", "print a specific flight", "[flight_number]", cliPrintFlight),
    CLI_COMMAND_DEF("log_enable", "enable the logging output", NULL, cliEnable),
    CLI_COMMAND_DEF("flash_start_write", "set recorder state to REC_WRITE_TO_FLASH", NULL, cliFlashWrite),
    CLI_COMMAND_DEF("flash_stop_write", "set recorder state to REC_FILL_QUEUE", NULL, cliFlashStop),
    CLI_COMMAND_DEF("ls", "list all files in current working directory", NULL, cliLs),
    CLI_COMMAND_DEF("cd", "change current working directory", NULL, cliCd),
    CLI_COMMAND_DEF("lfs_format", "reformat lfs", NULL, cliLfsFormat),
    CLI_COMMAND_DEF("config", "print and edit the flight config", "[print|reset|add] {event} {action} {arg}",
                    cliConfig),
};

static void cliLfsFormat(const char *cmdName, char *cmdline) {
  log_raw("\nTrying LFS format");
  lfs_format(&lfs, &lfs_cfg);
  int err = lfs_mount(&lfs, &lfs_cfg);
  if (err != 0) {
    log_raw("LFS mounting failed with error %d!", err);
  } else {
    log_raw("Mounting successful!");
    /* create the flights directory */
    lfs_mkdir(&lfs, "flights");

    strncpy(cwd, "/", sizeof(cwd));
  }
}
static void cliLs(const char *cmdName, char *cmdline) { lfs_ls(cwd); }
static void cliCd(const char *cmdName, char *cmdline) {
  /* TODO - check if a directory actually exists */
  if (cmdline == NULL || strcmp(cmdline, "/") == 0) {
    strncpy(cwd, "/", sizeof(cwd));
  } else if (strcmp(cmdline, "..") == 0) {
    /* return one lvl back */
  } else if (strcmp(cmdline, ".") != 0) {
    if (cmdline[0] == '/') {
      /* absolute path */
      strncpy(cwd, cmdline, sizeof(cwd));
    } else {
      /* relative path */
      strncat(cwd, cmdline, sizeof(cwd) - strlen(cwd) - 1);
    }
  }
}

static void cliEnable(const char *cmdName, char *cmdline) { log_enable(); }

static void cliEraseFlash(const char *cmdName, char *cmdline) {
  log_raw("\nErasing the flash, this might take a while...");
  w25q_chip_erase();
  log_raw("Flash erased!");
  log_raw("Mounting LFS");

  int err = lfs_mount(&lfs, &lfs_cfg);
  if (err == 0) {
    log_raw("LFS mounted successfully!");
  } else {
    log_raw("LFS mounting failed with error %d!", err);
    log_raw("Trying LFS format");
    lfs_format(&lfs, &lfs_cfg);
    int err2 = lfs_mount(&lfs, &lfs_cfg);
    if (err2 != 0) {
      log_raw("LFS mounting failed again with error %d!", err2);
      return;
    } else {
      log_raw("Mounting successful!");
    }
  }
  flight_counter = 0;
  /* create the flights directory */
  lfs_mkdir(&lfs, "flights");

  strncpy(cwd, "/", sizeof(cwd));
}

static void cliEraseRecordings(const char *cmdName, char *cmdline) {
  log_raw("\nErasing the flight recordings, this might not take much...");
  erase_recordings();
  log_raw("Recordings erased!");
}

static void cliRecInfo(const char *cmdName, char *cmdline) {
  // implement a "ls" here
  log_raw("\nNumber of recorded flights: %lu", flight_counter);
  lfs_ls("flights/");
}

static void cliPrintFlight(const char *cmdName, char *cmdline) {
  /* TODO - count how many files in a directory here */
  char *endptr;
  uint32_t flight_idx = strtoul(cmdline, &endptr, 10);

  if (cmdline != endptr) {
    // A number was found
    if (flight_idx > flight_counter) {
      log_raw("\nFlight %lu doesn't exist", flight_idx);
      log_raw("Number of recorded flights: %lu", flight_counter);
    } else {
      dump_recording(flight_idx);
    }
  } else {
    log_raw("\nArgument not provided!");
  }
}

static void cliFlashWrite(const char *cmdName, char *cmdline) {
  log_raw("\nSetting recorder state to REC_WRITE_TO_FLASH");
  set_recorder_state(REC_WRITE_TO_FLASH);
}

static void cliFlashStop(const char *cmdName, char *cmdline) {
  log_raw("\nSetting recorder state to REC_FILL_QUEUE");
  set_recorder_state(REC_FILL_QUEUE);
}

static void cliRead(const char *cmdName, char *cmdline) {
  osDelay(2000);
  /* Remember the current logging state so that we can revert back to it after we read out the flash */
  bool log_was_enabled = log_is_enabled();
  log_disable();
  /* iterate through all recordings in the /flights dir */
  uint16_t num_recorded_flights = 0;
  if (num_recorded_flights == 0)
    log_raw("No recordings found");
  else
    log_raw("Number of recorded flights: %hu", num_recorded_flights);
  for (int i = 0; i < num_recorded_flights; i++) {
    dump_recording(i);
  }
  if (log_was_enabled) log_enable();
}

static void cliDefaults(const char *cmdName, char *cmdline) {
  cc_defaults();
  cliPrint("Reset to default values");
}

static void cliDump(const char *cmdName, char *cmdline) {}

static void cliExit(const char *cmdName, char *cmdline) {
  NVIC_SystemReset();
}

static void cliSave(const char *cmdName, char *cmdline) {
  if (cc_save() == false){
    cliPrintLine("Saving unsuccessful, trying force save...");
    if (cc_format_save() == false){
      cliPrintLine("Force save failed!");
    }
  }
  cliPrintLine("Successfully written to flash");
}

static char *skipSpace(char *buffer) {
  while (*(buffer) == ' ') {
    buffer++;
  }
  return buffer;
}

static void cliSetVar(const clivalue_t *var, const uint32_t value) {
  void *ptr = var->pdata;
  uint32_t workValue;
  uint32_t mask;

  if ((var->type & VALUE_MODE_MASK) == MODE_BITSET) {
    switch (var->type & VALUE_TYPE_MASK) {
      case VAR_UINT8:
        mask = (1 << var->config.bitpos) & 0xff;
        if (value) {
          workValue = *(uint8_t *)ptr | mask;
        } else {
          workValue = *(uint8_t *)ptr & ~mask;
        }
        *(uint8_t *)ptr = workValue;
        break;

      case VAR_UINT16:
        mask = (1 << var->config.bitpos) & 0xffff;
        if (value) {
          workValue = *(uint16_t *)ptr | mask;
        } else {
          workValue = *(uint16_t *)ptr & ~mask;
        }
        *(uint16_t *)ptr = workValue;
        break;

      case VAR_UINT32:
        mask = 1 << var->config.bitpos;
        if (value) {
          workValue = *(uint32_t *)ptr | mask;
        } else {
          workValue = *(uint32_t *)ptr & ~mask;
        }
        *(uint32_t *)ptr = workValue;
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

static void print_sensor_state(){
  const lookupTableEntry_t *p_boot_table = &lookupTables[TABLE_BOOTSTATE];
  const lookupTableEntry_t *p_event_table = &lookupTables[TABLE_EVENTS];
  cliPrintf("Mode:\t%s\n", p_boot_table->values[global_cats_config.config.boot_state]);
  cliPrintf("State:\t%s\n", p_event_table->values[global_flight_state.flight_state-1]);
  cliPrintf("Voltage: %d.%02dV\n",(int)battery_voltage(),(int)(battery_voltage()*100)%100);
  cliPrintf("h: %dm, v: %dm/s, a: %dm/s^2", (int)global_kf_data.height, (int)global_kf_data.velocity, (int)global_kf_data.acceleration);
}

static void print_action_config() {
  const lookupTableEntry_t *p_event_table = &lookupTables[TABLE_EVENTS];
  const lookupTableEntry_t *p_action_table = &lookupTables[TABLE_ACTIONS];

  cliPrintf("\n * ACTION CONFIGURATION *\n");
  config_action_t action;
  for (int i = 0; i < NUM_EVENTS; i++) {
    int nr_actions = cc_get_action_number(i);
    if (nr_actions > 0) {
      cliPrintf("\n%s\n", p_event_table->values[i]);
      cliPrintf("   Number of Actions: %d\n", nr_actions);
      for (int j = 0; j < nr_actions; j++) {
        cc_get_action(i, (int16_t)j, &action);
        cliPrintf("     %s - %d\n", p_action_table->values[action.action_pointer], action.arg);
      }
    }
  }
}

static void print_timer_config() {
  const lookupTableEntry_t *p_event_table = &lookupTables[TABLE_EVENTS];

  cliPrintf("\n\n * TIMER CONFIGURATION *\n");
  for (int i = 0; i < num_timers; i++) {
    if (global_cats_config.config.timers[i].duration > 0) {
      cliPrintf("\nTIMER %d\n", i + 1);
      cliPrintf("  Start: %s\n", p_event_table->values[global_cats_config.config.timers[i].start_event]);
      cliPrintf("  End: %s\n", p_event_table->values[global_cats_config.config.timers[i].end_event]);
      cliPrintf("  Duration: %d ms\n", global_cats_config.config.timers[i].duration);
    }
  }
}

static void printValuePointer(const char *cmdName, const clivalue_t *var, const void *valuePointer, bool full) {
  if ((var->type & VALUE_MODE_MASK) == MODE_ARRAY) {
    for (int i = 0; i < var->config.array.length; i++) {
      switch (var->type & VALUE_TYPE_MASK) {
        default:
        case VAR_UINT8:
          // uint8_t array
          cliPrintf("%d", ((uint8_t *)valuePointer)[i]);
          break;

        case VAR_INT8:
          // int8_t array
          cliPrintf("%d", ((int8_t *)valuePointer)[i]);
          break;

        case VAR_UINT16:
          // uin16_t array
          cliPrintf("%d", ((uint16_t *)valuePointer)[i]);
          break;

        case VAR_INT16:
          // int16_t array
          cliPrintf("%d", ((int16_t *)valuePointer)[i]);
          break;

        case VAR_UINT32:
          // uin32_t array
          cliPrintf("%lu", ((uint32_t *)valuePointer)[i]);
          break;
      }

      if (i < var->config.array.length - 1) {
        cliPrint(",");
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

    bool valueIsCorrupted = false;
    switch (var->type & VALUE_MODE_MASK) {
      case MODE_DIRECT:
        if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32) {
          cliPrintf("%lu", (uint32_t)value);
          if ((uint32_t)value > var->config.u32Max) {
            valueIsCorrupted = true;
          } else if (full) {
            cliPrintf(" 0 %lu", var->config.u32Max);
          }
        } else {
          int min;
          int max;
          getMinMax(var, &min, &max);

          cliPrintf("%d", value);
          if ((value < min) || (value > max)) {
            valueIsCorrupted = true;
          } else if (full) {
            cliPrintf(" %d %d", min, max);
          }
        }
        break;
      case MODE_LOOKUP:
        if (value < lookupTables[var->config.lookup.tableIndex].valueCount) {
          cliPrint(lookupTables[var->config.lookup.tableIndex].values[value]);
        } else {
          valueIsCorrupted = true;
        }
        break;
      case MODE_BITSET:
        if (value & 1 << var->config.bitpos) {
          cliPrintf("ON");
        } else {
          cliPrintf("OFF");
        }
        break;
      case MODE_STRING:
        cliPrintf("%s", (strlen((char *)valuePointer) == 0) ? "-" : (char *)valuePointer);
        break;
    }

    if (valueIsCorrupted) {
      cliPrintLinefeed();
      cliPrintError(cmdName, "CORRUPTED CONFIG: %s = %d", var->name, value);
    }
  }
}

static void cliPrintVar(const char *cmdName, const clivalue_t *var, bool full) {
  const void *ptr = var->pdata;

  printValuePointer(cmdName, var, ptr, full);
}

static uint8_t getWordLength(char *bufBegin, char *bufEnd) {
  while (*(bufEnd - 1) == ' ') {
    bufEnd--;
  }

  return bufEnd - bufBegin;
}

uint16_t cli_get_event_index(char *name, uint8_t length) {
  const lookupTableEntry_t *p_event_table = &lookupTables[TABLE_EVENTS];
  for (uint32_t i = 0; i < 9; i++) {
    const char *settingName = p_event_table->values[i];

    // ensure exact match when setting to prevent setting variables with shorter names
    if (strncasecmp(name, settingName, strlen(settingName)) == 0 && length == strlen(settingName)) {
      return i;
    }
  }
  return valueTableEntryCount;
}

uint16_t cli_get_action_index(char *name, uint8_t length) {
  const lookupTableEntry_t *p_event_table = &lookupTables[TABLE_ACTIONS];
  for (uint32_t i = 0; i < 17; i++) {
    const char *settingName = p_event_table->values[i];

    // ensure exact match when setting to prevent setting variables with shorter names
    if (strncasecmp(name, settingName, strlen(settingName)) == 0 && length == strlen(settingName)) {
      return i;
    }
  }
  return valueTableEntryCount;
}

uint16_t cliGetSettingIndex(char *name, uint8_t length) {
  for (uint32_t i = 0; i < valueTableEntryCount; i++) {
    const char *settingName = valueTable[i].name;

    // ensure exact match when setting to prevent setting variables with shorter names
    if (strncasecmp(name, settingName, strlen(settingName)) == 0 && length == strlen(settingName)) {
      return i;
    }
  }
  return valueTableEntryCount;
}

static const char *nextArg(const char *currentArg) {
  const char *ptr = strchr(currentArg, ' ');
  while (ptr && *ptr == ' ') {
    ptr++;
  }

  return ptr;
}

static void cliPrintVarRange(const clivalue_t *var) {
  switch (var->type & VALUE_MODE_MASK) {
    case (MODE_DIRECT): {
      switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT32:
          cliPrintLinef("Allowed range: 0 - %lu", var->config.u32Max);

          break;
        case VAR_UINT8:
        case VAR_UINT16:
          cliPrintLinef("Allowed range: %d - %d", var->config.minmaxUnsigned.min, var->config.minmaxUnsigned.max);

          break;
        default:
          cliPrintLinef("Allowed range: %d - %d", var->config.minmax.min, var->config.minmax.max);

          break;
      }
    } break;
    case (MODE_LOOKUP): {
      const lookupTableEntry_t *tableEntry = &lookupTables[var->config.lookup.tableIndex];
      cliPrint("Allowed values: ");
      bool firstEntry = true;
      for (unsigned i = 0; i < tableEntry->valueCount; i++) {
        if (tableEntry->values[i]) {
          if (!firstEntry) {
            cliPrint(", ");
          }
          cliPrintf("%s", tableEntry->values[i]);
          firstEntry = false;
        }
      }
      cliPrintLinefeed();
    } break;
    case (MODE_ARRAY): {
      cliPrintLinef("Array length: %d", var->config.array.length);
    } break;
    case (MODE_STRING): {
      cliPrintLinef("String length: %d - %d", var->config.string.minlength, var->config.string.maxlength);
    } break;
    case (MODE_BITSET): {
      cliPrintLinef("Allowed values: OFF, ON");
    } break;
  }
}

static void cliGet(const char *cmdName, char *cmdline) {
  const clivalue_t *val;
  int matchedCommands = 0;

  for (uint32_t i = 0; i < valueTableEntryCount; i++) {
    if (strstr(valueTable[i].name, cmdline)) {
      val = &valueTable[i];
      if (matchedCommands > 0) {
        cliPrintLinefeed();
      }
      cliPrintf("%s = ", valueTable[i].name);
      cliPrintVar(cmdName, val, 0);
      cliPrintLinefeed();
      cliPrintVarRange(val);
      // cliPrintVarDefault(cmdName, val);

      matchedCommands++;
    }
  }

  if (!matchedCommands) {
    cliPrintErrorLinef(cmdName, "INVALID NAME");
  }
}

static void cliConfig(const char *cmdName, char *cmdline) {
  print_action_config();
  print_timer_config();
}

static void cliSet(const char *cmdName, char *cmdline) {
  const uint32_t len = strlen(cmdline);
  char *eqptr;

  if (len == 0 || (len == 1 && cmdline[0] == '*')) {
    cliPrintLine("Current settings: ");

    for (uint32_t i = 0; i < valueTableEntryCount; i++) {
      const clivalue_t *val = &valueTable[i];
      cliPrintf("%s = ", valueTable[i].name);
      cliPrintVar(cmdName, val,
                  len);  // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
      cliPrintLinefeed();
    }
  } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
    // has equals

    uint8_t variableNameLength = getWordLength(cmdline, eqptr);

    // skip the '=' and any ' ' characters
    eqptr++;
    eqptr = skipSpace(eqptr);

    const uint16_t index = cliGetSettingIndex(cmdline, variableNameLength);
    if (index >= valueTableEntryCount) {
      cliPrintErrorLinef(cmdName, "INVALID NAME");
      return;
    }
    const clivalue_t *val = &valueTable[index];

    bool valueChanged = false;

    switch (val->type & VALUE_MODE_MASK) {
      case MODE_DIRECT: {
        if ((val->type & VALUE_TYPE_MASK) == VAR_UINT32) {
          uint32_t value = strtoul(eqptr, NULL, 10);

          if (value <= val->config.u32Max) {
            cliSetVar(val, value);
            valueChanged = true;
          }
        } else {
          int value = atoi(eqptr);

          int min;
          int max;
          getMinMax(val, &min, &max);

          if (value >= min && value <= max) {
            cliSetVar(val, value);
            valueChanged = true;
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
          tableIndex = val->config.lookup.tableIndex;
        }
        const lookupTableEntry_t *tableEntry = &lookupTables[tableIndex];
        bool matched = false;
        for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
          matched = tableEntry->values[tableValueIndex] && strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

          if (matched) {
            cliSetVar(val, tableValueIndex);
            valueChanged = true;
          }
        }
      } break;
      case MODE_ARRAY: {
        const uint8_t arrayLength = val->config.array.length;
        char *valPtr = eqptr;

        int i = 0;
        while (i < arrayLength && valPtr != NULL) {
          // skip spaces
          valPtr = skipSpace(valPtr);

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
        valueChanged = true;

        break;
    }

    if (valueChanged) {
      cliPrintf("%s set to ", val->name);
      cliPrintVar(cmdName, val, 0);
    } else {
      cliPrintErrorLinef(cmdName, "INVALID VALUE");
      cliPrintVarRange(val);
    }

    return;
  } else {
    // no equals, check for matching variables.
    cliGet(cmdName, cmdline);
  }
}

static void cliStatus(const char *cmdName, char *cmdline) {
  print_sensor_state();
}

static void cliVersion(const char *cmdName, char *cmdline) {}

static void cliHelp(const char *cmdName, char *cmdline) {
  bool anyMatches = false;

  for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
    bool printEntry = false;
    if (isEmpty(cmdline)) {
      printEntry = true;
    } else {
      if (strstr(cmdTable[i].name, cmdline) || strstr(cmdTable[i].description, cmdline)) {
        printEntry = true;
      }
    }

    if (printEntry) {
      anyMatches = true;
      cliPrint(cmdTable[i].name);
      if (cmdTable[i].description) {
        cliPrintf(" - %s", cmdTable[i].description);
      }
      if (cmdTable[i].args) {
        cliPrintf("\r\n\t%s", cmdTable[i].args);
      }
      cliPrintLinefeed();
    }
  }
  if (!isEmpty(cmdline) && !anyMatches) {
    cliPrintErrorLinef(cmdName, "NO MATCHES FOR '%s'", cmdline);
  }
}

void cliPrint(const char *str) {
  while (*str) {
    while (fifo_write(cli_out, *str++) == false) {
      osDelay(1);
      str--;
    }
  }
}

static void cliPrompt(void) { cliPrintf("\r\n^._.^:%s> ", cwd); }

void cliPrintLinefeed(void) { cliPrint("\r\n"); }

void cliPrintLine(const char *str) {
  cliPrint(str);
  cliPrintLinefeed();
}

static void cliPrintHashLine(const char *str) {
  cliPrint("\r\n# ");
  cliPrintLine(str);
}

static void cliPrintfva(const char *format, va_list va) {
  static char buffer[CLI_OUT_BUFFER_SIZE];
  vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
  cliPrint(buffer);
}

static bool cliDumpPrintLinef(bool equalsDefault, const char *format, ...) {
  va_list va;
  va_start(va, format);
  cliPrintfva(format, va);
  va_end(va);
  cliPrintLinefeed();
  return true;
}

static void cliWrite(uint8_t ch) {
  while (fifo_write(cli_out, ch) == false) osDelay(10);
}

static bool cliDefaultPrintLinef(bool equalsDefault, const char *format, ...) {
  cliWrite('#');

  va_list va;
  va_start(va, format);
  cliPrintfva(format, va);
  va_end(va);
  cliPrintLinefeed();
  return true;
}

void cliPrintf(const char *format, ...) {
  va_list va;
  va_start(va, format);
  cliPrintfva(format, va);
  va_end(va);
}

void cliPrintLinef(const char *format, ...) {
  va_list va;
  va_start(va, format);
  cliPrintfva(format, va);
  va_end(va);
  cliPrintLinefeed();
}

static void cliPrintErrorVa(const char *cmdName, const char *format, va_list va) {
  cliPrint("ERROR IN ");
  cliPrint(cmdName);
  cliPrint(": ");
  char buffer[CLI_OUT_BUFFER_SIZE];
  vsnprintf(buffer, CLI_OUT_BUFFER_SIZE, format, va);
  cliPrint(buffer);
  cliPrint(": ");
  va_end(va);
}

static void cliPrintError(const char *cmdName, const char *format, ...) {
  va_list va;
  va_start(va, format);
  cliPrintErrorVa(cmdName, format, va);
}

static void cliPrintErrorLinef(const char *cmdName, const char *format, ...) {
  va_list va;
  va_start(va, format);
  cliPrintErrorVa(cmdName, format, va);
  cliPrint("\r\n");
}

static char *checkCommand(char *cmdline, const char *command) {
  if (!strncasecmp(cmdline, command, strlen(command))  // command names match
      && (isspace((unsigned)cmdline[strlen(command)]) || cmdline[strlen(command)] == 0)) {
    return skipSpace(cmdline + strlen(command) + 1);
  } else {
    return NULL;
  }
}

static void processCharacter(const char c) {
  if (bufferIndex && (c == '\n' || c == '\r')) {
    // enter pressed
    cliPrintLinefeed();

    // Strip comment starting with # from line
    char *p = cliBuffer;
    p = strchr(p, '#');
    if (NULL != p) {
      bufferIndex = (uint32_t)(p - cliBuffer);
    }
    // Strip trailing whitespace
    while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
      bufferIndex--;
    }

    // Process non-empty lines
    if (bufferIndex > 0) {
      cliBuffer[bufferIndex] = 0;  // null terminate

      const clicmd_t *cmd;
      char *options = NULL;
      for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
        options = checkCommand(cliBuffer, cmd->name);
        if (options) break;
      }
      if (cmd < cmdTable + ARRAYLEN(cmdTable)) {
        cmd->cliCommand(cmd->name, options);
      } else {
        cliPrintLine("UNKNOWN COMMAND, TRY 'HELP'");
      }
      bufferIndex = 0;
    }
    memcpy(oldcliBuffer, cliBuffer, sizeof(cliBuffer));
    memset(cliBuffer, 0, sizeof(cliBuffer));
    cliPrompt();

    // 'exit' will reset this flag, so we don't need to print prompt again

  } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
    if (!bufferIndex && c == ' ') return;  // Ignore leading spaces
    cliBuffer[bufferIndex++] = c;
    cliWrite(c);
  }
}

static void processCharacterInteractive(const char c) {
  static uint16_t ignore = 0;
  if (ignore) {
    ignore--;
    return;
  }
  if (c == '\t' || c == '?') {
    // do tab completion
    const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
    uint32_t i = bufferIndex;
    for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
      if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0)) {
        continue;
      }
      if (!pstart) {
        pstart = cmd;
      }
      pend = cmd;
    }
    if (pstart) { /* Buffer matches one or more commands */
      for (;; bufferIndex++) {
        if (pstart->name[bufferIndex] != pend->name[bufferIndex]) break;
        if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
          /* Unambiguous -- append a space */
          cliBuffer[bufferIndex++] = ' ';
          cliBuffer[bufferIndex] = '\0';
          break;
        }
        cliBuffer[bufferIndex] = pstart->name[bufferIndex];
      }
    }
    if (!bufferIndex || pstart != pend) {
      /* Print list of ambiguous matches */
      cliPrint("\r\n\033[K");
      for (cmd = pstart; cmd <= pend; cmd++) {
        cliPrint(cmd->name);
        cliWrite('\t');
      }
      cliPrompt();
      i = 0; /* Redraw prompt */
    }
    for (; i < bufferIndex; i++) cliWrite(cliBuffer[i]);
  } else if (c == 4) {  // CTRL-D - clear screen
    // clear screen
    cliPrint("\033[2J\033[1;1H");
    cliPrompt();
  } else if (c == 12) {  // CTRL-L - toggle logging
    if (log_is_enabled()) {
      log_disable();
      cliPrompt();
    } else {
      log_enable();
    }
  } else if (c == '\b') {
    // backspace
    if (bufferIndex) {
      cliBuffer[--bufferIndex] = 0;
      cliPrint("\010 \010");
    }
  } else if (c == 27) {
    // up arrow
    while (bufferIndex) {
      cliBuffer[--bufferIndex] = 0;
      cliPrint("\010 \010");
    }
    for (int i = 0; i < sizeof(oldcliBuffer); i++) {
      if (oldcliBuffer[i] == 0) break;
      processCharacter(oldcliBuffer[i]);
    }
    ignore = 2;
  } else {
    processCharacter(c);
  }
}

void cli_process(void) {
  while (fifo_get_length(cli_in) > 0) {
    processCharacterInteractive(fifo_read(cli_in));
  }
}

void cli_enter(fifo_t *in, fifo_t *out) {
  cli_in = in;
  cli_out = out;
  cliPrompt();
}
