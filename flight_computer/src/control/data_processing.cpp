/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "control/data_processing.hpp"

#include "config/control_config.hpp"

#include <cstring>

/* a temporary variable "tmp" must be defined beforehand to use these macros */
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SWAP(a, b, tmp) \
  {                     \
    (tmp) = (a);        \
    (a) = (b);          \
    (b) = (tmp);        \
  }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define SORT_TWO(a, b, tmp)              \
  {                                      \
    if ((a) > (b)) SWAP((a), (b), (tmp)) \
  }

/* this function returns the median of an array of size MEDIAN_FILTER_SIZE
 * it assumes that the size of "input_array" is equal to MEDIAN_FILTER_SIZE */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
float32_t median(float32_t input_array[]) {
  float32_t array[MEDIAN_FILTER_SIZE];
  memcpy(array, input_array, MEDIAN_FILTER_SIZE * sizeof(float32_t));

  if constexpr (MEDIAN_FILTER_SIZE == 9) {
    /* hardwired algorithm - see
     * https://web.archive.org/web/20060613213236/https://www.xilinx.com/xcell/xl23/xl23_16.pdf
     */
    float tmp{0.0F};
    SORT_TWO(array[1], array[2], tmp);
    SORT_TWO(array[4], array[5], tmp);
    SORT_TWO(array[7], array[8], tmp);
    SORT_TWO(array[0], array[1], tmp);
    SORT_TWO(array[3], array[4], tmp);
    SORT_TWO(array[6], array[7], tmp);
    SORT_TWO(array[1], array[2], tmp);
    SORT_TWO(array[4], array[5], tmp);
    SORT_TWO(array[7], array[8], tmp);
    SORT_TWO(array[0], array[3], tmp);
    SORT_TWO(array[5], array[8], tmp);
    SORT_TWO(array[4], array[7], tmp);
    SORT_TWO(array[3], array[6], tmp);
    SORT_TWO(array[1], array[4], tmp);
    SORT_TWO(array[2], array[5], tmp);
    SORT_TWO(array[4], array[7], tmp);
    SORT_TWO(array[4], array[2], tmp);
    SORT_TWO(array[6], array[4], tmp);
    SORT_TWO(array[4], array[2], tmp);
    return array[4];
  } else {
    /* quickselect algorithm */
    float a = 0.0F;
    float tmp = 0.0F;
    int32_t i = 0;
    int32_t j = 0;
    int32_t m = 0;
    const int32_t k = MEDIAN_FILTER_SIZE >> 1U;
    int32_t l = 0;
    int32_t r = MEDIAN_FILTER_SIZE - 1;

    while (r > l + 1) { /* keep iterating until our partition is of size less than two */
      // NOLINTNEXTLINE(hicpp-signed-bitwise)
      m = (l + r) >> 1U;

      /* we want the inequalities array[l] <= array[l + 1] <= array[r] to hold */
      SWAP(array[m], array[l + 1], tmp);
      SORT_TWO(array[l], array[r], tmp);
      SORT_TWO(array[l + 1], array[r], tmp);
      SORT_TWO(array[l], array[l + 1], tmp);

      i = l + 1;
      j = r;
      a = array[l + 1];

      while (true) {
        i++;
        j--;
        while (array[i] < a) {
          i++;
        }
        while (array[j] > a) {
          j--;
        }
        if (j < i) {
          break;
        }
        SWAP(array[i], array[j], tmp);
      }

      array[l + 1] = array[j];
      array[j] = a;

      /* the partition for the next iteration [l, r] must contain the median */
      if (j >= k) {
        r = j - 1;
      }
      if (j <= k) {
        l = i;
      }
    }

    /* we have the final partition which is of size 1 or 2 */
    if (r == l + 1) {
      SORT_TWO(array[l], array[r], tmp);
    }

    return array[k];
  }
}

/* Todo: Huge Ass comment describing this formula */
float32_t calculate_height(float32_t pressure) {
  return (-(powf(pressure / P_INITIAL, (1 / 5.257F)) - 1) * (TEMPERATURE_0 + 273.15F) / 0.0065F);
}

/* Todo: Huge Ass comment describing Barometric Liftoff Detection */
float32_t approx_moving_average(float32_t data, bool is_transparent) {
  static float32_t avg = 0;
  if (is_transparent) {
    avg -= avg / BARO_LIFTOFF_FAST_MOV_AVG_SIZE;
    avg += data / BARO_LIFTOFF_FAST_MOV_AVG_SIZE;
  } else {
    avg -= avg / BARO_LIFTOFF_MOV_AVG_SIZE;
    avg += data / BARO_LIFTOFF_MOV_AVG_SIZE;
  }
  return avg;
}
