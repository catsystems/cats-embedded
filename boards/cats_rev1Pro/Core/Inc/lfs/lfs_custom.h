//
// Created by Nemanja on 8/8/2021.
//

#pragma once

#include "lfs/lfs.h"

extern lfs_t lfs;
extern const struct lfs_config lfs_cfg;

extern lfs_file_t fc_file;
extern lfs_file_t current_flight_file;

extern char cwd[256];

extern uint32_t flight_counter;

/**
 * List the contents of the directory
 *
 * @param path - path to the directory
 * @return 0 if no error
 */
int lfs_ls(const char *path);