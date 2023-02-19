/*
 * CATS Flight Software
 * Copyright (C) 2023 Control and Telemetry Systems
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

#pragma once

#include "lfs.h"

/* TODO: Wrap lfs functions where you always pass this lfs variable instead of making it visible globally */
extern lfs_t lfs;

#ifdef __cplusplus
extern "C" {
#endif
const struct lfs_config *get_lfs_cfg();
#ifdef __cplusplus
}
#endif

extern lfs_file_t fc_file;

extern char cwd[LFS_NAME_MAX];

extern uint32_t flight_counter;

/**
 * List the contents of the directory.
 *
 * @param path - path to the directory
 * @return 0 if no error
 */
int lfs_ls(const char *path);

/**
 * Return the type of an object in the file system.
 *
 * @param path - path to the LFS object
 * @return LFS_TYPE_REG if file, LFS_TYPE_DIR if directory, or -1 in case of error.
 */
int8_t lfs_obj_type(const char *path);

/**
 * Counts the number of elements of a given type on the provided path. Not recursive.
 *
 * @param path directory whose elements are to be counted
 * @param type type of the elements to count, either LFS_TYPE_REG (file) or LFS_TYPE_DIR
 * @return number of elements, negative number in case of error
 */

#ifdef __cplusplus
extern "C" {
#endif
int32_t lfs_cnt(const char *path, enum lfs_type type);
#ifdef __cplusplus
}
#endif
