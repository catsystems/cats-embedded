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

#include "cli/settings.hpp"
#include "cli/cli.hpp"
#include "util/enum_str_maps.hpp"

#include <cstddef>

#define LOOKUP_TABLE_ENTRY(name) \
  { name, ARRAYLEN(name) }

const lookup_table_entry_t lookup_tables[] = {
    LOOKUP_TABLE_ENTRY(event_map),   LOOKUP_TABLE_ENTRY(action_map),
    LOOKUP_TABLE_ENTRY(on_off_map),  {(const char *const *)recorder_speed_map, NUM_REC_SPEEDS},
    LOOKUP_TABLE_ENTRY(battery_map),
};

#undef LOOKUP_TABLE_ENTRY

const cli_value_t value_table[] = {
    // Control
    {"acc_threshold",
     VAR_UINT16,
     {.minmax_unsigned = {30, 80}},
     offsetof(cats_config_t, control_settings.liftoff_acc_threshold)},
    {"liftoff_detection_agl",
     VAR_UINT16,
     {.minmax_unsigned = {50, 100}},
     offsetof(cats_config_t, control_settings.liftoff_detection_agl)},
    {"main_altitude",
     VAR_UINT16,
     {.minmax_unsigned = {10, 65535}},
     offsetof(cats_config_t, control_settings.main_altitude)},

    // Timers
    {"timer1_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[0].start_event)},
    {"timer1_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[0].trigger_event)},
    {"timer1_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[0].duration)},
    {"timer2_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[1].start_event)},
    {"timer2_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[1].trigger_event)},
    {"timer2_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[1].duration)},
    {"timer3_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[2].start_event)},
    {"timer3_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[2].trigger_event)},
    {"timer3_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[2].duration)},
    {"timer4_start",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[3].start_event)},
    {"timer4_trigger",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_EVENTS}},
     offsetof(cats_config_t, timers[3].trigger_event)},
    {"timer4_duration", VAR_UINT32, {.u32_max = 1200000}, offsetof(cats_config_t, timers[3].duration)},

    // Events
    {"ev_calibrate",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CALIBRATE])},
    {"ev_ready", VAR_INT16 | MODE_ARRAY, {.array = {.length = 16}}, offsetof(cats_config_t, action_array[EV_READY])},
    {"ev_liftoff",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_LIFTOFF])},
    {"ev_burnout", VAR_INT16 | MODE_ARRAY, {.array = {.length = 16}}, offsetof(cats_config_t, action_array[EV_MAX_V])},
    {"ev_apogee", VAR_INT16 | MODE_ARRAY, {.array = {.length = 16}}, offsetof(cats_config_t, action_array[EV_APOGEE])},
    {"ev_main_deployment",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_MAIN_DEPLOYMENT])},
    {"ev_touchdown",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_TOUCHDOWN])},
    {"ev_custom1",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CUSTOM_1])},
    {"ev_custom2",
     VAR_INT16 | MODE_ARRAY,
     {.array = {.length = 16}},
     offsetof(cats_config_t, action_array[EV_CUSTOM_2])},

    // Servo position
    {"servo1_init_pos", VAR_INT16, {.minmax_unsigned = {0, 1000}}, offsetof(cats_config_t, initial_servo_position[0])},
    {"servo2_init_pos", VAR_INT16, {.minmax_unsigned = {0, 1000}}, offsetof(cats_config_t, initial_servo_position[1])},

    {"tele_link_phrase",
     VAR_UINT8 | MODE_STRING,
     {.string = {4, 8}},
     offsetof(cats_config_t, telemetry_settings.link_phrase)},
    {"tele_test_phrase",
     VAR_UINT8 | MODE_STRING,
     {.string = {4, 8}},
     offsetof(cats_config_t, telemetry_settings.test_phrase)},
    {"tele_power_level",
     VAR_UINT8,
     {.minmax_unsigned = {16, 30}},
     offsetof(cats_config_t, telemetry_settings.power_level)},
    {"tele_enable",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_POWER}},
     offsetof(cats_config_t, telemetry_settings.enable_telemetry)},
    {"tele_adaptive_power",
     VAR_UINT8 | MODE_LOOKUP,
     {.lookup = {TABLE_POWER}},
     offsetof(cats_config_t, telemetry_settings.adaptive_power)},

    {"buzzer_volume", VAR_UINT8, {.minmax_unsigned = {0, 100}}, offsetof(cats_config_t, buzzer_volume)},
    {"battery_type", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_BATTERY}}, offsetof(cats_config_t, battery_type)},

    {"rec_elements", VAR_UINT32, {.u32_max = UINT32_MAX}, offsetof(cats_config_t, rec_mask)},
    {"rec_speed", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_SPEEDS}}, offsetof(cats_config_t, rec_speed_idx)},
    {"test_mode", VAR_UINT8 | MODE_LOOKUP, {.lookup = {TABLE_POWER}}, offsetof(cats_config_t, enable_testing_mode)},
};

const uint16_t value_table_entry_count = ARRAYLEN(value_table);

void *get_cats_config_member_ptr(const cats_config_t *cfg, const cli_value_t *var) {
  return ((uint8_t *)cfg) + var->member_offset;
}

void print_cats_config(const char *cmd_name, const cats_config_t *cfg, bool print_limits) {
  const char *prefix = "";
  if (!strcmp(cmd_name, "dump")) {
    prefix = "set ";
  }

  for (uint32_t i = 0; i < value_table_entry_count; i++) {
    const cli_value_t *val = &value_table[i];
    cli_printf("%s%s = ", prefix, value_table[i].name);
    cli_print_var(cmd_name, cfg, val, print_limits);
    cli_print_linefeed();
  }

  /* Print out the 'set_by_user' flag when requested by commands other than 'dump'. */
  if (strcmp(cmd_name, "dump") != 0) {
    cli_printf("set_by_user: %s", cfg->is_set_by_user ? "TRUE" : "FALSE");
  }
}
