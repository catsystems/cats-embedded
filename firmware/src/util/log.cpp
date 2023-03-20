/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#include <cstdarg>
#include <cstdio>

#include "comm/stream_group.hpp"
#include "util/log.h"

#ifdef CATS_DEBUG
#include "cmsis_os.h"

#define CATS_RAINBOW_LOG

static struct {
  int level;
  log_mode_e log_mode;
} L;

static const char *level_strings[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
#ifdef CATS_RAINBOW_LOG
static const char *level_colors[] = {"\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};
#endif

#define PRINT_BUFFER_LEN 420
static char print_buffer[PRINT_BUFFER_LEN];
#endif

void log_set_mode(log_mode_e mode) {
#ifdef CATS_DEBUG
  L.log_mode = mode;
#endif
}
log_mode_e log_get_mode() {
#ifdef CATS_DEBUG
  return L.log_mode;
#else
  return LOG_MODE_NONE;
#endif
}

// Only has impact on LOG_MODE_DEFAULT
void log_set_level(int level) {
#ifdef CATS_DEBUG
  L.level = level;
#endif
}

void log_enable() {
#ifdef CATS_DEBUG
  L.log_mode = LOG_MODE_DEFAULT;
#endif
}

void log_disable() {
#ifdef CATS_DEBUG
  L.log_mode = LOG_MODE_NONE;
#endif
}

bool log_is_enabled() {
#ifdef CATS_DEBUG
  return L.log_mode == LOG_MODE_DEFAULT;
#else
  return false;
#endif
}

void log_log(int level, const char *file, int line, const char *format, ...) {
#ifdef CATS_DEBUG
  if ((L.log_mode == LOG_MODE_DEFAULT) && level >= L.level) {
    /* fill buffer with metadata */
    static char buf_ts[16];
    buf_ts[snprintf(buf_ts, sizeof(buf_ts), "%lu", osKernelGetTickCount())] = '\0';
    static char buf_loc[30];
    buf_loc[snprintf(buf_loc, sizeof(buf_loc), "%s:%d:", file, line)] = '\0';
    int len;
#ifdef CATS_RAINBOW_LOG
    len = snprintf(print_buffer, PRINT_BUFFER_LEN, "%6s %s%5s\x1b[0m \x1b[90m%30s\x1b[0m ", buf_ts, level_colors[level],
                   level_strings[level], buf_loc);
#else
    len = snprintf(print_buffer, PRINT_BUFFER_LEN, "%6s %5s %30s ", buf_ts, level_strings[level], buf_loc);
#endif
    va_list argptr;
    va_start(argptr, format);
    len += vsnprintf(print_buffer + len, PRINT_BUFFER_LEN, format, argptr);
    va_end(argptr);
    len += snprintf(print_buffer + len, PRINT_BUFFER_LEN, "\n");
    stream_write(USB_SG.out, (uint8_t *)print_buffer, len);
  }
#endif
}

void log_raw(const char *format, ...) {
#ifdef CATS_DEBUG
  va_list argptr;
  va_start(argptr, format);
  int len = vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
  va_end(argptr);
  len += snprintf(print_buffer + len, PRINT_BUFFER_LEN, "\n");
  stream_write(USB_SG.out, (uint8_t *)print_buffer, len);
#endif
}

void log_sim(const char *format, ...) {
#ifdef CATS_DEBUG
  if (L.log_mode == LOG_MODE_SIM) {
    va_list argptr;
    va_start(argptr, format);
    int len = vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
    va_end(argptr);
    len += snprintf(print_buffer + len, PRINT_BUFFER_LEN, "\n");
    stream_write(USB_SG.out, (uint8_t *)print_buffer, len);
  }
#endif
}

void log_rawr(const char *format, ...) {
#ifdef CATS_DEBUG
  va_list argptr;
  va_start(argptr, format);
  int len = vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
  va_end(argptr);
  stream_write(USB_SG.out, (uint8_t *)print_buffer, len);
#endif
}
