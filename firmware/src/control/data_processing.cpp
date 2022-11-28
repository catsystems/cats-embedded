/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
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

#include "control/data_processing.h"
#include "config/control_config.h"
#include <cstring>


/* a temporary variable "tmp" must be defined beforehand to use these macros */
#define SWAP(a, b, tmp) \
  {                     \
    tmp = (a);          \
    (a) = (b);          \
    (b) = tmp;          \
  }

#define SORT_TWO(a, b, tmp)              \
  {                                      \
    if ((a) > (b)) SWAP((a), (b), (tmp)) \
  }

/* this function returns the median of an array of size MEDIAN_FILTER_SIZE
 * it assumes that the size of "input_array" is equal to MEDIAN_FILTER_SIZE */
float32_t median(float32_t input_array[]) {
    float32_t array[MEDIAN_FILTER_SIZE];
    memcpy(array, input_array, MEDIAN_FILTER_SIZE * sizeof(float32_t));

#if MEDIAN_FILTER_SIZE == 9
    /* hardwired algorithm - see https://web.archive.org/web/20060613213236/https://www.xilinx.com/xcell/xl23/xl23_16.pdf
     */
    float tmp;
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
#else
    /* quickselect algorithm */
    float a, tmp;
    int32_t i, j, m, k = (MEDIAN_FILTER_SIZE >> 1);
    int32_t l = 0, r = MEDIAN_FILTER_SIZE - 1;

    while (r > l + 1) { /* keep iterating until our partition is of size less than two */
      m = (l + r) >> 1;

      /* we want the inequalities array[l] <= array[l + 1] <= array[r] to hold */
      SWAP(array[m], array[l + 1], tmp);
      SORT_TWO(array[l], array[r], tmp);
      SORT_TWO(array[l + 1], array[r], tmp);
      SORT_TWO(array[l], array[l + 1], tmp);

      i = l + 1;
      j = r;
      a = array[l + 1];

      while (1) {
        i++;
        j--;
        while (array[i] < a) i++;
        while (array[j] > a) j--;
        if (j < i) break;
        SWAP(array[i], array[j], tmp);
      }

      array[l + 1] = array[j];
      array[j] = a;

      /* the partition for the next iteration [l, r] must contain the median */
      if (j >= k) r = j - 1;
      if (j <= k) l = i;
    }

    /* we have the final partition which is of size 1 or 2 */
    if (r == l + 1) {
      SORT_TWO(array[l], array[r], tmp);
    }

    return array[k];
#endif
}

/* Todo: Huge Ass comment describing this formula */
float32_t calculate_height(float32_t pressure) {
    return (-(powf(pressure / P_INITIAL, (1 / 5.257f)) - 1) * (TEMPERATURE_0 + 273.15f) / 0.0065f);
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
