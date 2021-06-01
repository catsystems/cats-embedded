/*
 * settings.h
 *
 *  Created on: 1 Jun 2021
 *      Author: Luca
 */

#pragma once
#include <stdbool.h>

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

typedef enum {
    TABLE_BOOTSTATE = 0,
	TABLE_EVENTS
} lookupTableIndex_e;

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 3
#define VALUE_MODE_OFFSET 5

typedef enum {
    // value type, bits 0-2
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),

    // value mode, bits 5-7
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET),
    MODE_ARRAY = (2 << VALUE_MODE_OFFSET),
    MODE_BITSET = (3 << VALUE_MODE_OFFSET),
    MODE_STRING = (4 << VALUE_MODE_OFFSET),
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x07)
#define VALUE_SECTION_MASK (0x18)
#define VALUE_MODE_MASK (0xE0)

typedef struct cliMinMaxConfig_s {
    const int16_t min;
    const int16_t max;
} cliMinMaxConfig_t;

typedef struct cliMinMaxUnsignedConfig_s {
    const uint16_t min;
    const uint16_t max;
} cliMinMaxUnsignedConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef struct cliArrayLengthConfig_s {
    const uint8_t length;
} cliArrayLengthConfig_t;

typedef struct cliStringLengthConfig_s {
    const uint8_t minlength;
    const uint8_t maxlength;
    const uint8_t flags;
} cliStringLengthConfig_t;

#define STRING_FLAGS_NONE      (0)
#define STRING_FLAGS_WRITEONCE (1 << 0)

typedef union {
    cliLookupTableConfig_t lookup;            // used for MODE_LOOKUP excl. VAR_UINT32
    cliMinMaxConfig_t minmax;                 // used for MODE_DIRECT with signed parameters
    cliMinMaxUnsignedConfig_t minmaxUnsigned; // used for MODE_DIRECT with unsigned parameters
    cliArrayLengthConfig_t array;             // used for MODE_ARRAY
    cliStringLengthConfig_t string;           // used for MODE_STRING
    uint8_t bitpos;                           // used for MODE_BITSET
    uint32_t u32Max;                          // used for MODE_DIRECT with VAR_UINT32
} cliValueConfig_t;

typedef struct clivalue_s {
    const char *name;
    const uint8_t type;                       // see cliValueFlag_e
    const cliValueConfig_t config;

    void* pdata;
} __attribute__((packed)) clivalue_t;

extern const lookupTableEntry_t lookupTables[];
extern const uint16_t valueTableEntryCount;

extern const clivalue_t valueTable[];
