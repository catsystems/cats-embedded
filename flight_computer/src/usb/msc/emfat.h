/// Copyright (C) 2020, 2024 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later
///
/// Additional notice:
/// This file was adapted from emfat (https://github.com/fetisov/emfat/blob/master/project/emfat.c),
/// released under MIT License.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

struct emfat_entry_s;
typedef void (*emfat_readcb_t)(uint8_t *dest, int size, uint32_t offset, struct emfat_entry_s *entry);
typedef void (*emfat_writecb_t)(const uint8_t *data, int size, uint32_t offset, struct emfat_entry_s *entry);

typedef struct emfat_entry_s {
  const char *name;
  bool dir;
  uint8_t attr;
  int level;
  uint32_t offset;
  int32_t number;
  uint32_t curr_size;
  uint32_t max_size;
  long user_data;
  uint32_t cma_time[3]; /**< create/mod/access time in unix format */
  emfat_readcb_t readcb;
  emfat_writecb_t writecb;
  struct {
    uint32_t first_clust;
    uint32_t last_clust;
    uint32_t last_reserved;
    uint32_t num_subentry;
    struct emfat_entry_s *top;
    struct emfat_entry_s *sub;
    struct emfat_entry_s *next;
  } priv;
  uint8_t lfs_flight_idx;  // custom - used for storing lfs flight log index
} emfat_entry_t;

typedef struct emfat_s {
  uint64_t vol_size;
  uint32_t disk_sectors;
  const char *vol_label;
  struct {
    uint32_t boot_lba;
    uint32_t fsinfo_lba;
    uint32_t fat1_lba;
    uint32_t fat2_lba;
    uint32_t root_lba;
    uint32_t num_clust;
    uint32_t free_clust;
    emfat_entry_t *entries;
    emfat_entry_t *last_entry;
    int num_entries;
  } priv;
} emfat_t;

bool emfat_init(emfat_t *emfat, const char *label, emfat_entry_t *entries);
void emfat_read(emfat_t *emfat, uint8_t *data, uint32_t sector, int num_sectors);
void emfat_write(emfat_t *emfat, const uint8_t *data, uint32_t sector, int num_sectors);

#define EMFAT_ENCODE_CMA_TIME(D, M, Y, h, m, s) \
  ((((((Y)-1980U) << 9U) | ((M) << 5U) | (D)) << 16U) | (((h) << 11U) | ((m) << 5U) | ((s) >> 1U)))

static inline uint32_t emfat_encode_cma_time(int D, int M, int Y, int h, int m, int s) {
  return EMFAT_ENCODE_CMA_TIME(D, M, Y, h, m, s);
}

uint32_t emfat_cma_time_from_unix(uint32_t unix_time);

#define ATTR_READ       0x01
#define ATTR_HIDDEN     0x02
#define ATTR_SYSTEM     0x04
#define ATTR_VOL_LABEL  0x08
#define ATTR_DIR        0x10
#define ATTR_ARCHIVE    0x20
#define ATTR_LONG_FNAME 0x0F

#ifdef __cplusplus
}
#endif
