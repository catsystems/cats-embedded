/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from tinyusb (https://github.com/hathach/tinyusb),
/// released under MIT License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by board.mk
#define CFG_TUSB_MCU              OPT_MCU_STM32F4
#define CFG_TUSB_OS               OPT_OS_FREERTOS
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#define BOARD_DEVICE_RHPORT_NUM   0
#define CFG_TUSB_RHPORT_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_DEVICE_RHPORT_NUM
#define BOARD_DEVICE_RHPORT_NUM 0
#endif

// RHPort max operational speed can defined by board.mk
// Default to Highspeed for MCU with internal HighSpeed PHY (can be port specific), otherwise FullSpeed
#ifndef BOARD_DEVICE_RHPORT_SPEED
#if TU_CHECK_MCU(OPT_MCU_LPC18XX, OPT_MCU_LPC43XX, OPT_MCU_MIMXRT10XX, OPT_MCU_NUC505) || \
    TU_CHECK_MCU(OPT_MCU_CXD56, OPT_MCU_SAMX7X, OPT_MCU_BCM2711) || TU_CHECK_MCU(OPT_MCU_FT90X, OPT_MCU_FT93X)
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#else
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#endif
#endif

// Device mode with rhport and speed defined by board.mk
#if BOARD_DEVICE_RHPORT_NUM == 0
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
#define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
#error "Incorrect RHPort configuration"
#endif

// This example doesn't use an RTOS
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

// CFG_TUSB_DEBUG is defined by compiler in DEBUG build
// #define CFG_TUSB_DEBUG           0

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC    1
#define CFG_TUD_MSC    1
#define CFG_TUD_HID    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_VENDOR 0

// #define TUD_OPT_HIGH_SPEED

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512

// CDC Endpoint transfer buffer size, more is faster
#define CFG_TUD_CDC_EP_BUFSIZE 512

// MSC Buffer size of Device Mass storage
#define CFG_TUD_MSC_EP_BUFSIZE 512

#ifdef __cplusplus
}
#endif
