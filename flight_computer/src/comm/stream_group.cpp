/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "comm/stream_group.hpp"

#include "comm/fifo.hpp"
#include "comm/stream.hpp"

/** USB STREAM GROUP **/
constexpr uint16_t USB_OUT_BUF_SIZE = 512;
constexpr uint16_t USB_IN_BUF_SIZE = 1024;

constexpr uint8_t USB_TIMEOUT_MSEC = 10;

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static uint8_t usb_fifo_out_buf[USB_OUT_BUF_SIZE];
static uint8_t usb_fifo_in_buf[USB_IN_BUF_SIZE];

static fifo_t usb_fifo_in = {
    .head = 0, .tail = 0, .used = 0, .size = USB_IN_BUF_SIZE, .buf = usb_fifo_in_buf, .mutex = false};
static fifo_t usb_fifo_out = {
    .head = 0, .tail = 0, .used = 0, .size = USB_OUT_BUF_SIZE, .buf = usb_fifo_out_buf, .mutex = false};

static stream_t usb_stream_in = {.fifo = &usb_fifo_in, .timeout_msec = USB_TIMEOUT_MSEC};
static stream_t usb_stream_out = {.fifo = &usb_fifo_out, .timeout_msec = USB_TIMEOUT_MSEC};

const stream_group_t USB_SG = {.in = &usb_stream_in, .out = &usb_stream_out};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)
