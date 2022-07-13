/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
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

#include "comm/stream_group.h"

#include "comm/fifo.h"
#include "comm/stream.h"

/** USB STREAM GROUP **/
#define USB_OUT_BUF_SIZE 512
#define USB_IN_BUF_SIZE  256

#define USB_TIMEOUT_MSEC 10

static uint8_t usb_fifo_out_buf[USB_OUT_BUF_SIZE];
static uint8_t usb_fifo_in_buf[USB_IN_BUF_SIZE];

static fifo_t usb_fifo_in = {
    .head = 0, .tail = 0, .used = 0, .size = USB_IN_BUF_SIZE, .buf = usb_fifo_in_buf, .mutex = false};
static fifo_t usb_fifo_out = {
    .head = 0, .tail = 0, .used = 0, .size = USB_OUT_BUF_SIZE, .buf = usb_fifo_out_buf, .mutex = false};

static stream_t usb_stream_in = {.fifo = &usb_fifo_in, .timeout_msec = USB_TIMEOUT_MSEC};
static stream_t usb_stream_out = {.fifo = &usb_fifo_out, .timeout_msec = USB_TIMEOUT_MSEC};

const stream_group_t USB_SG = {.in = &usb_stream_in, .out = &usb_stream_out};
