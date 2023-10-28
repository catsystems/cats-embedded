/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "comm/stream.hpp"

#include "comm/fifo.hpp"

#include "util/task_util.hpp"

void stream_init(stream_t *stream, fifo_t *fifo, uint32_t timeout_msec) {
  stream->fifo = fifo;
  stream->timeout_msec = timeout_msec;
}

void stream_reset_fifo(stream_t *stream) { fifo_reset(stream->fifo); }

uint32_t stream_length(const stream_t *stream) { return fifo_get_length(stream->fifo); }

bool stream_read_byte(const stream_t *stream, uint8_t *byte_ptr) {
  bool ret = false;
  uint32_t timeout = 0;

  do {
    ret |= fifo_read_byte(stream->fifo, byte_ptr);
    if (ret) {
      break;
    }
    sysDelay(1);
  } while ((++timeout < stream->timeout_msec));
  return ret;
}

bool stream_write_byte(const stream_t *stream, uint8_t byte) {
  bool ret = false;
  uint32_t timeout = 0;

  do {
    ret |= fifo_write_byte(stream->fifo, byte);
    if (ret) {
      break;
    }
    sysDelay(1);
  } while ((++timeout < stream->timeout_msec));
  return ret;
}

bool stream_read(const stream_t *stream, uint8_t *data, uint32_t len) {
  bool ret = false;
  uint32_t timeout = 0;

  if (len == 0) {
    return true;
  }

  do {
    ret |= fifo_read(stream->fifo, data, len);
    if (ret) {
      break;
    }
    sysDelay(1);
  } while ((++timeout < stream->timeout_msec));
  return ret;
}

bool stream_write(const stream_t *stream, const uint8_t *data, uint32_t len) {
  bool ret = false;
  uint32_t timeout = 0;

  if (len == 0) {
    return true;
  }

  do {
    ret |= fifo_write(stream->fifo, data, len);
    if (ret) {
      break;
    }
    sysDelay(1);
  } while ((++timeout < stream->timeout_msec));
  return ret;
}
