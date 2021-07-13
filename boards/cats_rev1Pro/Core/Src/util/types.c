//
// Created by stoja on 28.12.20.
//

#include "util/types.h"

/* as suggested by
 * https://stackoverflow.com/questions/23699719/inline-vs-static-inline-in-header-file
 */
extern inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low);
extern inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low);

const char *flight_fsm_map[14] = {"INVALID",   "MOVING",      "IDLE",       "THRUSTING_1", "THRUSTING_2",
                                  "COASTING",  "TRANSONIC_1", "SUPERSONIC", "TRANSONIC_2", "APOGEE",
                                  "PARACHUTE", "BALLISTIC",   "TOUCHDOWN",  "HEHE"};

const char *drop_test_fsm_map[7] = {"DT_INVALID", "DT_IDLE",      "DT_WAITING", "DT_DROGUE",
                                    "DT_MAIN",    "DT_TOUCHDOWN", "HEHE"};
