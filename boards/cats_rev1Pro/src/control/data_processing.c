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
#include <string.h>

// Utility function to find minimum of two integers
int min(int x, int y) { return (x < y) ? x : y; }

/* Function to merge the two haves arr[l..m] and arr[m+1..r] of array arr[] */
void merge(float arr[], int l, int m, int r) {
  int i, j, k;
  int n1 = m - l + 1;
  int n2 = r - m;

  /* create temp arrays */
  float L[n1], R[n2];

  /* Copy data to temp arrays L[] and R[] */
  for (i = 0; i < n1; i++) L[i] = arr[l + i];
  for (j = 0; j < n2; j++) R[j] = arr[m + 1 + j];

  /* Merge the temp arrays back into arr[l..r]*/
  i = 0;
  j = 0;
  k = l;
  while (i < n1 && j < n2) {
    if (L[i] <= R[j]) {
      arr[k] = L[i];
      i++;
    } else {
      arr[k] = R[j];
      j++;
    }
    k++;
  }

  /* Copy the remaining elements of L[], if there are any */
  while (i < n1) {
    arr[k] = L[i];
    i++;
    k++;
  }

  /* Copy the remaining elements of R[], if there are any */
  while (j < n2) {
    arr[k] = R[j];
    j++;
    k++;
  }
}

/* Iterative mergesort function to sort arr[0...n-1] */
void mergeSort(float arr[], int n) {
  int curr_size;  // For current size of subarrays to be merged
  // curr_size varies from 1 to n/2
  int left_start;  // For picking starting index of left subarray
  // to be merged

  // Merge subarrays in bottom up manner.  First merge subarrays of
  // size 1 to create sorted subarrays of size 2, then merge subarrays
  // of size 2 to create sorted subarrays of size 4, and so on.
  for (curr_size = 1; curr_size <= n - 1; curr_size = 2 * curr_size) {
    // Pick starting point of different subarrays of current size
    for (left_start = 0; left_start < n - 1; left_start += 2 * curr_size) {
      // Find ending point of left subarray. mid+1 is starting
      // point of right
      int mid = min(left_start + curr_size - 1, n - 1);

      int right_end = min(left_start + 2 * curr_size - 1, n - 1);

      // Merge Subarrays arr[left_start...mid] & arr[mid+1...right_end]
      merge(arr, left_start, mid, right_end);
    }
  }
}

float median(float array[], int array_size) {
  float dummy_array[array_size];
  memcpy(dummy_array, array, array_size * sizeof(float));
  mergeSort(dummy_array, array_size);
  return dummy_array[array_size / 2];
}