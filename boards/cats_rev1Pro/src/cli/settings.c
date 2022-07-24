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

#include "cli/settings.h"
#include "util/enum_str_maps.h"

#include <stddef.h>

#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookup_table_entry_t lookup_tables[] = {
    LOOKUP_TABLE_ENTRY(boot_state_map),
    LOOKUP_TABLE_ENTRY(event_map),
    LOOKUP_TABLE_ENTRY(action_map),
    {(const char *const *)recorder_speed_map, NUM_REC_SPEEDS},
};

#undef LOOKUP_TABLE_ENTRY

const cli_value_t value_table[] = {
    {"boot_state", VAR_UINT32 | MODE_LOOKUP, {.lookup = {TABLE_BOOTSTATE}}, offsetof(cats_config_u, config.boot_state)},

    // Control
    {"main_altitude",
     VAR_UINT16,
     {.minmax_unsigned = {10, 65535}},
     offsetof(cats_config_u, config.control_settings.main_altitude)},
    {"acc_threshold",
     VAR_UINT16,
     {.minmax_unsigned = {15, 80}},
     offsetof(cats_config_u, config.control_settings.liftoff_acc_threshold)},
    {"mach_timer_duration",
     VAR_UINT16,
     {.minmax_unsigned = {0, 60000}},
     offsetof(cats_config_u, config.control_settings.mach_timer_duration)},

    // Timers
    {"timer1_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[0].start_event)},
    {"timer1_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[0].trigger_event)},
    {"timer1_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_u, config.timers[0].duration)},
    {"timer2_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[1].start_event)},
    {"timer2_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[1].trigger_event)},
    {"timer2_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_u, config.timers[1].duration)},
    {"timer3_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[2].start_event)},
    {"timer3_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[2].trigger_event)},
    {"timer3_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_u, config.timers[2].duration)},
    {"timer4_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[3].start_event)},
    {"timer4_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_u, config.timers[3].trigger_event)},
    {"timer4_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_u, config.timers[3].duration)},

    // Events
    {"ev_moving",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_MOVING])},
    {"ev_ready",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_READY])},
    {"ev_liftoff",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_LIFTOFF])},
    {"ev_burnout",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_MAX_V])},
    {"ev_apogee",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_APOGEE])},
    {"ev_lowalt",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_POST_APOGEE])},
    {"ev_touchdown",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_TOUCHDOWN])},
    {"ev_custom1",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_CUSTOM_1])},
    {"ev_custom2",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_u, config.action_array[EV_CUSTOM_2])},

    // Servo position
    {"servo1_init_pos",
     VAR_INT16,
     {.minmax_unsigned = {0, 180}},
     offsetof(cats_config_u, config.initial_servo_position[0])},
    {"servo2_init_pos",
     VAR_INT16,
     {.minmax_unsigned = {0, 180}},
     offsetof(cats_config_u, config.initial_servo_position[1])},

    {"rec_elements", VAR_UINT32, {.u32_max = UINT32_MAX}, offsetof(cats_config_u, config.rec_mask)},
    {"rec_speed", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_SPEEDS}}, offsetof(cats_config_u, config.rec_speed_idx)}};

const uint16_t value_table_entry_count = ARRAYLEN(value_table);

void *get_cats_config_member_ptr(const cats_config_u *cfg, const cli_value_t *var) {
  return ((uint8_t *)cfg) + var->member_offset;
}
