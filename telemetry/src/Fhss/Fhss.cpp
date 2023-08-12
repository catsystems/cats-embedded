/// CATS Flight Software
/// Copyright (C) 2022 Control and Telemetry Systems
///
/// This program is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// This program is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Fhss.hpp"
#include <cstring>

#define RADIO_SX128X
#define Regulatory_Domain_ISM_2400

const uint32_t FHSSfreqs[] = {FreqHzToRegVal(2400400000), FreqHzToRegVal(2401400000), FreqHzToRegVal(2402400000),
                              FreqHzToRegVal(2403400000), FreqHzToRegVal(2404400000),

                              FreqHzToRegVal(2405400000), FreqHzToRegVal(2406400000), FreqHzToRegVal(2407400000),
                              FreqHzToRegVal(2408400000), FreqHzToRegVal(2409400000),

                              FreqHzToRegVal(2410400000), FreqHzToRegVal(2411400000), FreqHzToRegVal(2412400000),
                              FreqHzToRegVal(2413400000), FreqHzToRegVal(2414400000),

                              FreqHzToRegVal(2415400000), FreqHzToRegVal(2416400000), FreqHzToRegVal(2417400000),
                              FreqHzToRegVal(2418400000), FreqHzToRegVal(2419400000),

                              FreqHzToRegVal(2420400000), FreqHzToRegVal(2421400000), FreqHzToRegVal(2422400000),
                              FreqHzToRegVal(2423400000), FreqHzToRegVal(2424400000),

                              FreqHzToRegVal(2425400000), FreqHzToRegVal(2426400000), FreqHzToRegVal(2427400000),
                              FreqHzToRegVal(2428400000), FreqHzToRegVal(2429400000),

                              FreqHzToRegVal(2430400000), FreqHzToRegVal(2431400000), FreqHzToRegVal(2432400000),
                              FreqHzToRegVal(2433400000), FreqHzToRegVal(2434400000),

                              FreqHzToRegVal(2435400000), FreqHzToRegVal(2436400000), FreqHzToRegVal(2437400000),
                              FreqHzToRegVal(2438400000), FreqHzToRegVal(2439400000),

                              FreqHzToRegVal(2440400000), FreqHzToRegVal(2441400000), FreqHzToRegVal(2442400000),
                              FreqHzToRegVal(2443400000), FreqHzToRegVal(2444400000),

                              FreqHzToRegVal(2445400000), FreqHzToRegVal(2446400000), FreqHzToRegVal(2447400000),
                              FreqHzToRegVal(2448400000), FreqHzToRegVal(2449400000),

                              FreqHzToRegVal(2450400000), FreqHzToRegVal(2451400000), FreqHzToRegVal(2452400000),
                              FreqHzToRegVal(2453400000), FreqHzToRegVal(2454400000),

                              FreqHzToRegVal(2455400000), FreqHzToRegVal(2456400000), FreqHzToRegVal(2457400000),
                              FreqHzToRegVal(2458400000), FreqHzToRegVal(2459400000),

                              FreqHzToRegVal(2460400000), FreqHzToRegVal(2461400000), FreqHzToRegVal(2462400000),
                              FreqHzToRegVal(2463400000), FreqHzToRegVal(2464400000),

                              FreqHzToRegVal(2465400000), FreqHzToRegVal(2466400000), FreqHzToRegVal(2467400000),
                              FreqHzToRegVal(2468400000), FreqHzToRegVal(2469400000),

                              FreqHzToRegVal(2470400000), FreqHzToRegVal(2471400000), FreqHzToRegVal(2472400000),
                              FreqHzToRegVal(2473400000), FreqHzToRegVal(2474400000),

                              FreqHzToRegVal(2475400000), FreqHzToRegVal(2476400000), FreqHzToRegVal(2477400000),
                              FreqHzToRegVal(2478400000), FreqHzToRegVal(2479400000)};

// Number of FHSS frequencies in the table
constexpr uint32_t FHSS_FREQ_CNT = (sizeof(FHSSfreqs) / sizeof(uint32_t));
// Number of hops in the FHSSsequence list before circling back around, even
// multiple of the number of frequencies

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
// Actual sequence of hops as indexes into the frequency list
uint8_t FHSSsequence[FHSS_SEQUENCE_CNT];
// Which entry in the sequence we currently are on
uint8_t volatile FHSSptr;
// Channel for sync packets and initial connection establishment
uint_fast8_t sync_channel;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pseudorandom

Approach:
  Fill the sequence array with the sync channel every FHSS_FREQ_CNT
  Iterate through the array, and for each block, swap each entry in it with
  another random entry, excluding the sync channel.

*/

uint16_t crc16(uint8_t const *data, int32_t size);

uint8_t contains(int current_pos, uint8_t value) {
  for (int i = 0; i < current_pos; i++) {
    if (FHSSsequence[i] == value) {
      return 1;
    }
  }
  return 0;
}

void FHSSrandomiseFHSSsequence(uint32_t crc) {
  rngSeed(crc);
  int i = 0;
  while (i < FHSS_SEQUENCE_CNT) {
    const uint8_t next_freq = rngN(FHSS_FREQ_CNT);
    if (contains(i, next_freq) == 0) {
      FHSSsequence[i] = next_freq;
      i++;
    }
  }
  sync_channel = FHSSsequence[0];
}

uint32_t FHSSgetChannelCount() { return FHSS_FREQ_CNT; }
