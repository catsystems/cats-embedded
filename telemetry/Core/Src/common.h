#pragma once

#include "SX1280Driver/SX1280Driver.h"
#include "Transmission/Transmission.h"
#include "util/ringbuffer.h"

extern Transmission Link;

extern RingBuffer<uint8_t, 128> Uart1Buffer;
extern RingBuffer<uint8_t, 128> Uart2Buffer;
