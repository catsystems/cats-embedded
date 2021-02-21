/*
 * util.h
 *
 *  Created on: Feb 24, 2020
 *      Author: stoja
 *
 *      Logging inspired by https://github.com/rxi/log.c
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <string.h>
#include "FreeRTOSConfig.h"
#include "cmsis_os.h"

/** TRACING SECTION **/
#if (configUSE_TRACE_FACILITY == 1)
#define trace_print(ch, str)       vTracePrint(ch, str);
#define trace_printf(ch, str, ...) vTracePrintF(ch, str, __VA_ARGS__);
#else
#define trace_print(ch, str) \
  do {                       \
  } while (0)
#define trace_printf(ch, str, ...) \
  do {                             \
  } while (0)
#endif

/** LOGGING SECTION **/

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

void log_set_level(int level);
void log_enable();
void log_disable();

/* Read In Fake Sensor Data */
#define USB_DATA_ENABLE 0

/* Debug flag */
#ifdef CATS_DEBUG
#undef CATS_DEBUG
#endif
/* Comment the next line in order to disable debug mode */
#define CATS_DEBUG
#define CATS_RAINBOW_LOG

#if (configUSE_TRACE_FACILITY == 1) && defined(CATS_DEBUG)
#undef CATS_DEBUG
#endif

/* remove the rainbow flag if CATS_DEBUG isn't active */
#ifndef CATS_DEBUG
#undef CATS_RAINBOW_LOG
#endif

#ifdef CATS_DEBUG
#define PRINT_BUFFER_LEN 420
extern osMutexId_t print_mutex;
#endif

#ifdef CATS_DEBUG
#define GET_FILENAME \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
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

void log_log(int level, const char *file, int line, const char *format, ...)
    __attribute__((format(printf, 4, 5)));

void log_raw(const char *format, ...) __attribute__((format(printf, 1, 2)));

/* just like log_raw, but without \n */
void log_rawr(const char *format, ...) __attribute__((format(printf, 1, 2)));

#endif /* INC_UTIL_H_ */
