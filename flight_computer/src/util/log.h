/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

/* Logging inspired by https://github.com/rxi/log.c */

#pragma once

#include "cmsis_os.h"

/** LOGGING SECTION **/

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };
typedef enum { LOG_MODE_NONE, LOG_MODE_DEFAULT, LOG_MODE_SIM } log_mode_e;

#ifdef __cplusplus
extern "C" {
#endif

void log_set_level(int level);
void log_set_mode(log_mode_e mode);
log_mode_e log_get_mode();

void log_enable();
void log_disable();
bool log_is_enabled();

void log_log(int level, const char *file, int line, const char *format, ...) __attribute__((format(printf, 4, 5)));

void log_raw(const char *format, ...) __attribute__((format(printf, 1, 2)));

void log_sim(const char *format, ...) __attribute__((format(printf, 1, 2)));

/* just like log_raw, but without \n */
void log_rawr(const char *format, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#ifdef CATS_DEBUG

#if defined(_WIN32)
#define PATH_SEPARATOR '\\'
#else
#define DIR_SEPARATOR '/'
#endif

#define GET_FILENAME (strrchr(__FILE__, DIR_SEPARATOR) ? strrchr(__FILE__, DIR_SEPARATOR) + 1 : __FILE__)

#define log_trace(...) log_log(LOG_TRACE, GET_FILENAME, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(LOG_DEBUG, GET_FILENAME, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(LOG_INFO, GET_FILENAME, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(LOG_WARN, GET_FILENAME, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(LOG_ERROR, GET_FILENAME, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(LOG_FATAL, GET_FILENAME, __LINE__, __VA_ARGS__)
#else
/* to avoid running the GET_FILENAME macro while not debugging */
#define log_trace(...) log_raw(__VA_ARGS__)
#define log_debug(...) log_raw(__VA_ARGS__)
#define log_info(...)  log_raw(__VA_ARGS__)
#define log_warn(...)  log_raw(__VA_ARGS__)
#define log_error(...) log_raw(__VA_ARGS__)
#define log_fatal(...) log_raw(__VA_ARGS__)
#endif
