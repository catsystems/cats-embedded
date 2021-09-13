/*
 * settings.c
 *
 *  Created on: 1 Jun 2021
 *      Author: Luca
 */

#include "config/cats_config.h"
#include "cli/settings.h"

const char* const lookupTableBootState[] = {
    "CATS_INVALID", "CATS_IDLE", "CATS_CONFIG", "CATS_TIMER", "CATS_DROP", "CATS_FLIGHT",
};

const char* const lookupTableEvents[] = {
    "MOVING", "READY", "LIFTOFF", "MAX_V", "APOGEE", "POST_APOGEE", "TOUCHDOWN", "CUSTOM_1", "CUSTOM_2",
};

const char* const lookupTableActions[] = {
    "NONE",   "DELAY",   "HC_ONE",  "HC_TWO",    "HC_THREE",  "HC_FOUR",     "HC_FIVE",    "HC_SIX",   "LL_ONE",
    "LL_TWO", "LL_TREE", "LL_FOUR", "SERVO_ONE", "SERVO_TWO", "SERVO_THREE", "SERVO_FOUR", "RECORDER",
};

#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookupTableEntry_t lookupTables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableBootState),
    LOOKUP_TABLE_ENTRY(lookupTableEvents),
    LOOKUP_TABLE_ENTRY(lookupTableActions),
};

#undef LOOKUP_TABLE_ENTRY

const clivalue_t valueTable[] = {
    {"boot_state", VAR_UINT32 | MODE_LOOKUP, .config.lookup = {TABLE_BOOTSTATE}, &global_cats_config.config.boot_state},

    // Control
    {"main_altitude", VAR_UINT16, .config.minmaxUnsigned = {10, 65535},
     &global_cats_config.config.control_settings.main_altitude},
    {"acc_threshhold", VAR_UINT16, .config.minmaxUnsigned = {1500, 8000},
     &global_cats_config.config.control_settings.liftoff_acc_threshold},

    // Timers
    {"timer1_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[0].start_event},
    {"timer1_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[0].end_event},
    {"timer1_duration", VAR_UINT16, .config.minmaxUnsigned = {0, 60000},
     &global_cats_config.config.timers[0].duration},

    {"timer2_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].start_event},
    {"timer2_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].end_event},
    {"timer2_duration", VAR_UINT16, .config.minmaxUnsigned = {0, 60000},
     &global_cats_config.config.timers[1].duration},

    {"timer3_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].start_event},
    {"timer3_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].end_event},
    {"timer3_duration", VAR_UINT16, .config.minmaxUnsigned = {0, 60000},
     &global_cats_config.config.timers[2].duration},

    {"timer4_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].start_event},
    {"timer4_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].end_event},
    {"timer4_duration", VAR_UINT16, .config.minmaxUnsigned = {0, 60000},
     &global_cats_config.config.timers[3].duration},
     {"ev_moving", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_MOVING]},
     {"ev_ready", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_IDLE]},
    {"ev_liftoff", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_LIFTOFF]},
    {"ev_burnout", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_MAX_V]},
    {"ev_apogee", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_APOGEE]},
    {"ev_lowalt", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_POST_APOGEE]},
    {"ev_touchdown", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_TOUCHDOWN]},
    {"ev_custom1", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_CUSTOM_1]},
    {"ev_custom2", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_CUSTOM_2]},
    {"servo1_init_pos", VAR_INT16, .config.minmaxUnsigned = {0, 180},
     &global_cats_config.config.initial_servo_position[0]},
    {"servo2_init_pos", VAR_INT16, .config.minmaxUnsigned = {0, 180},
      &global_cats_config.config.initial_servo_position[1]},
};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
