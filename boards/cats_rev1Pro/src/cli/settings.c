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

const lookup_table_entry_t lookup_tables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableBootState),
    LOOKUP_TABLE_ENTRY(lookupTableEvents),
    LOOKUP_TABLE_ENTRY(lookupTableActions),
};

#undef LOOKUP_TABLE_ENTRY

const cli_value_t value_table[] = {
    {"boot_state", VAR_UINT32 | MODE_LOOKUP, .config.lookup = {TABLE_BOOTSTATE}, &global_cats_config.config.boot_state},

    // Control
    {"main_altitude", VAR_UINT16, .config.minmax_unsigned = {10, 65535},
     &global_cats_config.config.control_settings.main_altitude},
    {"acc_threshhold", VAR_UINT16, .config.minmax_unsigned = {1500, 8000},
     &global_cats_config.config.control_settings.liftoff_acc_threshold},
    {"mach_timer_duration", VAR_UINT16, .config.minmax_unsigned = {0, 60000},
     &global_cats_config.config.control_settings.mach_timer_duration},

    // Timers
    {"timer1_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[0].start_event},
    {"timer1_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[0].end_event},
    {"timer1_duration", VAR_UINT32, .config.u32_max = 1200000, &global_cats_config.config.timers[0].duration},
    {"timer2_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].start_event},
    {"timer2_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[1].end_event},
    {"timer2_duration", VAR_UINT32, .config.u32_max = 1200000, &global_cats_config.config.timers[1].duration},
    {"timer3_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].start_event},
    {"timer3_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[2].end_event},
    {"timer3_duration", VAR_UINT32, .config.u32_max = 1200000, &global_cats_config.config.timers[2].duration},
    {"timer4_start", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].start_event},
    {"timer4_end", VAR_UINT8 | MODE_LOOKUP, .config.lookup = {TABLE_EVENTS},
     &global_cats_config.config.timers[3].end_event},
    {"timer4_duration", VAR_UINT32, .config.u32_max = 1200000, &global_cats_config.config.timers[3].duration},

    // Events
    {"ev_moving", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_MOVING]},
    {"ev_ready", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_READY]},
    {"ev_liftoff", VAR_INT16 | MODE_ARRAY, .config.array.length = 16,
     global_cats_config.config.action_array[EV_LIFTOFF]},
    {"ev_burnout", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_MAX_V]},
    {"ev_apogee", VAR_INT16 | MODE_ARRAY, .config.array.length = 16, global_cats_config.config.action_array[EV_APOGEE]},
    {"ev_lowalt", VAR_INT16 | MODE_ARRAY, .config.array.length = 16,
     global_cats_config.config.action_array[EV_POST_APOGEE]},
    {"ev_touchdown", VAR_INT16 | MODE_ARRAY, .config.array.length = 16,
     global_cats_config.config.action_array[EV_TOUCHDOWN]},
    {"ev_custom1", VAR_INT16 | MODE_ARRAY, .config.array.length = 16,
     global_cats_config.config.action_array[EV_CUSTOM_1]},
    {"ev_custom2", VAR_INT16 | MODE_ARRAY, .config.array.length = 16,
     global_cats_config.config.action_array[EV_CUSTOM_2]},

    // Servo position
    {"servo1_init_pos", VAR_INT16, .config.minmax_unsigned = {0, 180},
     &global_cats_config.config.initial_servo_position[0]},
    {"servo2_init_pos", VAR_INT16, .config.minmax_unsigned = {0, 180},
     &global_cats_config.config.initial_servo_position[1]},
};

const uint16_t value_table_entry_count = ARRAYLEN(value_table);
