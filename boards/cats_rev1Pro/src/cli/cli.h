/*
 * cli.h
 *
 *  Created on: 21 May 2021
 *      Author: Luca
 */

#pragma once
#include "util/fifo.h"

void cli_process(void);
void cli_enter(fifo_t *in, fifo_t *out);

void cliPrint(const char *str);
void cliPrintf(const char *format, ...) __attribute__((format(printf, 1, 2)));