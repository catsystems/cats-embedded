/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include <cstdarg>
#include <cstdio>

#include "comm/stream_group.hpp"
#include "util/log.h"

#ifdef CATS_DEBUG
#include "cmsis_os.h"

#define CATS_RAINBOW_LOG

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static struct {
  int level;
  log_mode_e log_mode;
} L;

static const char *level_strings[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
#ifdef CATS_RAINBOW_LOG
static const char *level_colors[] = {"\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"};
#endif

constexpr uint16_t PRINT_BUFFER_LEN = 420;
static char print_buffer[PRINT_BUFFER_LEN];
#endif
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

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
    int len = 0;
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
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream_write(USB_SG.out, reinterpret_cast<uint8_t *>(print_buffer), len);
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
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream_write(USB_SG.out, reinterpret_cast<uint8_t *>(print_buffer), len);
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
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream_write(USB_SG.out, reinterpret_cast<uint8_t *>(print_buffer), len);
  }
#endif
}

void log_rawr(const char *format, ...) {
#ifdef CATS_DEBUG
  va_list argptr;
  va_start(argptr, format);
  const int len = vsnprintf(print_buffer, PRINT_BUFFER_LEN, format, argptr);
  va_end(argptr);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream_write(USB_SG.out, reinterpret_cast<uint8_t *>(print_buffer), len);
#endif
}
