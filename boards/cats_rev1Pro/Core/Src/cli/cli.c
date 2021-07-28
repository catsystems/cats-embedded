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
#include "drivers/w25q256.h"
#include "util/actions.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#define CLI_IN_BUFFER_SIZE  256
#define CLI_OUT_BUFFER_SIZE 256

static uint32_t bufferIndex = 0;

static char cliBuffer[CLI_IN_BUFFER_SIZE];

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
static void cliMcuId(const char *cmdName, char *cmdline);
static void cliSet(const char *cmdName, char *cmdline);
static void cliStatus(const char *cmdName, char *cmdline);
static void cliVersion(const char *cmdName, char *cmdline);
static void cliEraseFlash(const char *cmdName, char *cmdline);
static void cliEraseRecordings(const char *cmdName, char *cmdline);
static void cliRecInfo(const char *cmdName, char *cmdline);
static void cliPrintFlight(const char *cmdName, char *cmdline);
static void cliFlashWrite(const char *cmdName, char *cmdline);
static void cliFlashStop(const char *cmdName, char *cmdline);

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
    CLI_COMMAND_DEF("mcu_id", "id of the microcontroller", NULL, cliMcuId),
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
};

static void cliEnable(const char *cmdName, char *cmdline) { log_enable(); }

static void cliEraseFlash(const char *cmdName, char *cmdline) {
  log_raw("\nErasing the flash, this might take a while...");
  w25q_chip_erase();
  cs_init(CATS_STATUS_SECTOR, 0);
  cs_save();
  log_raw("Flash erased!");
}

static void cliEraseRecordings(const char *cmdName, char *cmdline) {
  log_raw("\nErasing the flight recordings, this might not take much...");
  erase_recordings();
  cs_init(CATS_STATUS_SECTOR, 0);
  cs_save();
  log_raw("Recordings erased!");
}

static void cliRecInfo(const char *cmdName, char *cmdline) {
  uint16_t num_flights = cs_get_num_recorded_flights();
  log_raw("\nNumber of recorded flights: %hu", num_flights);
  for (uint16_t i = 0; i < num_flights; i++) {
    log_raw("Last sectors of flight %hu: %lu", i, cs_get_last_sector_of_flight(i));
  }
}

static void cliPrintFlight(const char *cmdName, char *cmdline) {
  uint16_t num_flights = cs_get_num_recorded_flights();
  char *endptr;
  uint32_t flight_idx = strtoul(cmdline, &endptr, 10);

  if (cmdline != endptr) {
    // A number was found
    if (flight_idx >= num_flights) {
      log_raw("\nFlight %lu doesn't exist", flight_idx);
      log_raw("Number of recorded flights: %hu", num_flights);
    } else {
      log_raw("");
      print_recording(flight_idx);
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
  uint16_t num_recorded_flights = cs_get_num_recorded_flights();
  if (num_recorded_flights == 0)
    log_raw("No recordings found");
  else
    log_raw("Number of recorded flights: %hu", num_recorded_flights);
  for (int i = 0; i < num_recorded_flights; i++) {
    print_recording(i);
  }
  if (log_was_enabled) log_enable();
}

static void cliDefaults(const char *cmdName, char *cmdline) {
  cc_defaults();
  cliPrint("Reset to default values");
}

static void cliDump(const char *cmdName, char *cmdline) {}

static void cliExit(const char *cmdName, char *cmdline) {}

static void cliMcuId(const char *cmdName, char *cmdline) {}

static void cliSave(const char *cmdName, char *cmdline) {
  cc_save();
  NVIC_SystemReset();
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
    int16_t value = 0;
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
            value = tableValueIndex;

            cliSetVar(val, value);
            valueChanged = true;
          }
        }
      } break;
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

static void cliStatus(const char *cmdName, char *cmdline) {}

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
    while (fifo_write(cli_out, *str++) == false){
    	osDelay(10);
    	*str--;
    }
  }
}

static void cliPrompt(void) { cliPrint("\r\n> "); }

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
