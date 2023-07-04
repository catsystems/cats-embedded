#pragma once

#include "config.hpp"

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

typedef enum {
  STRING = 0,
  TOGGLE = 1,
  NUMBER = 2,
} settings_type_e;

typedef struct {
  int16_t min;
  int16_t max;
} settings_min_max_t;

typedef union {
  uint32_t stringLength;
  uint32_t lookup;
  settings_min_max_t minmax;
} settings_limits_u;

typedef struct {
  const char* name;
  const char* description1;
  const char* description2;
  settings_type_e type;
  settings_limits_u config;

  void* dataPtr;

} device_settings_t;

typedef enum {
  TABLE_MODE = 0,
  TABLE_UNIT,
  TABLE_LOGGING,
} lookup_table_index_e;

const char* const mode_map[2] = {
    "SINGLE",
    "DUAL",
};

const char* const unit_map[2] = {
    "METRIC",
    "RETARDED",
};

const char* const logging_map[2] = {
    "DOWN",
    "NEVER",
};

typedef struct {
  const char* const* values;
  const uint8_t value_count;
} lookup_table_entry_t;

#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookup_table_entry_t lookup_tables[] = {
    LOOKUP_TABLE_ENTRY(mode_map),
    LOOKUP_TABLE_ENTRY(unit_map),
    LOOKUP_TABLE_ENTRY(logging_map),
};

const char* const settingPageName[2] = {
    "General",
    "Telemetry",
};

const device_settings_t settingsTable[][4] = {
    {
        {"Time Zone",
         "Set the time offset",
         "",
         NUMBER,
         {.minmax = {.min = -12, .max = 12}},
         &systemConfig.config.timeZoneOffset},
        {"Stop Logging",
         "Down: Stop the log at touchdown",
         "Never: Never stop logging after liftoff",
         TOGGLE,
         {.lookup = TABLE_LOGGING},
         &systemConfig.config.neverStopLogging},
    },
    {
        {"Mode",
         "Single: Use both receiver to track one rocket",
         "Dual: Use both receivers individually",
         TOGGLE,
         {.lookup = TABLE_MODE},
         &systemConfig.config.receiverMode},
        {"Link Phrase 1",
         "Single Mode: Set phrase for both receivers",
         "Dual Mode: Set phrase for the left receiver",
         STRING,
         {.stringLength = 16},
         systemConfig.config.linkPhrase1},
        {"Link Phrase 2",
         "Single Mode: No functionality",
         "Dual Mode: Set phrase for the right receiver",
         STRING,
         {.stringLength = 16},
         systemConfig.config.linkPhrase2},
        {"Testing Phrase",
         "Set the phrase for the testing mode",
         "",
         STRING,
         {.stringLength = 16},
         systemConfig.config.testingPhrase},
    },
};

const uint16_t settingsTableValueCount[2] = {2, 4};
