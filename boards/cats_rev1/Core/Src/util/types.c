//
// Created by stoja on 28.12.20.
//

#include "util/types.h"

/* as suggested by
 * https://stackoverflow.com/questions/23699719/inline-vs-static-inline-in-header-file
 */
extern inline uint16_t uint8_to_uint16(uint8_t src_high, uint8_t src_low);
extern inline int16_t uint8_to_int16(uint8_t src_high, uint8_t src_low);