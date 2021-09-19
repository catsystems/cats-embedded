/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "config/cats_config.h"
#include "cli/settings.h"

const char* const lookupTableBootState[] = {
    "CATS_INVALID", "CATS_IDLE", "CATS_CONFIG", "CATS_TIMER", "CATS_DROP", "CATS_FLIGHT",
};

const char* const lookupTabeEvents[] = {
    "UNUSED", "IDLE", "MOVING", "LIFTOFF", "MAX_V", "APOGEE", "POST_APOGEE", "TOUCHDOWN", "TIMER_1_END", "TIMER_2_END",
};

#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookupTableEntry_t lookupTables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableBootState),
    LOOKUP_TABLE_ENTRY(lookupTabeEvents),
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
    {"timer1_duration", VAR_UINT16, .config.minmaxUnsigned = {1000, 60000},
     &global_cats_config.config.timers[0].duration},

    {"timer2_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].start_event},
    {"timer2_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].end_event},
    {"timer2_duration", VAR_UINT16, .config.minmaxUnsigned = {1000, 60000},
     &global_cats_config.config.timers[1].duration},

    {"timer3_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].start_event},
    {"timer3_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].end_event},
    {"timer3_duration", VAR_UINT16, .config.minmaxUnsigned = {1000, 60000},
     &global_cats_config.config.timers[2].duration},

    {"timer4_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].start_event},
    {"timer4_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].end_event},
    {"timer4_duration", VAR_UINT16, .config.minmaxUnsigned = {1000, 60000},
     &global_cats_config.config.timers[3].duration},

};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);
