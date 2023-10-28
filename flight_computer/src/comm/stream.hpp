/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include "comm/fifo.hpp"

/**
 * A wrapper for a fifo with a retry timeout.
 */
struct stream_t {
  /* Pointer to the fifo. */
  fifo_t *fifo;
  /* Timeout after which the stream will stop trying to read from the fifo. */
  uint32_t timeout_msec;
};

/**
 * Initialize a stream with a fifo and timeout.
 */
void stream_init(stream_t *stream, fifo_t *fifo, uint32_t timeout_msec);

/**
 * Reset the stream's underlying fifo.
 */
void stream_reset_fifo(stream_t *stream);

/**
 * Get the occupied amount of bytes in the stream's underlying fifo.
 * @return
 */
uint32_t stream_length(const stream_t *stream);

/**
 * Read a single byte from a stream.
 * @param stream
 * @param byte_ptr pointer to where the byte will be written
 * @return
 */
bool stream_read_byte(const stream_t *stream, uint8_t *byte_ptr);

/**
 * Write a single byte to a stream.
 */
bool stream_write_byte(const stream_t *stream, uint8_t byte);

/**
 * Read multiple bytes from a stream.
 * @param stream
 * @param data pointer to where the data will be written
 * @param count number of bytes to read
 * @return
 */
bool stream_read(const stream_t *stream, uint8_t *data, uint32_t len);

/**
 * Write multiple bytes to a stream.
 * @param stream
 * @param data bytes to be written
 * @param count number of bytes to write
 * @return
 */
bool stream_write(const stream_t *stream, const uint8_t *data, uint32_t len);
