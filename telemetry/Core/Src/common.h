#pragma once


#include "SX1280Driver/SX1280Driver.h"

typedef enum
{
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7

} expresslrs_tlm_ratio_e;

typedef enum {
	TX = 1,
	RX = 2,
} transmission_direction_e;

typedef enum {
	UNIDIRECTIONAL = 1,
	BIDIRECTIONAL = 2,
} transmission_mode_e;

typedef enum {
    connected,
    tentative,
    disconnected,
} connectionState_e;

typedef struct modulation_settings_s {
    uint8_t bw;
    uint8_t sf;
    uint8_t cr;
    uint32_t interval;
    uint8_t PreambleLen;
    uint8_t PayloadLength;
} modulation_settings_t;



