/*
 * transmisson.h
 *
 *  Created on: Mar 3, 2022
 *      Author: luca.jost
 */
#pragma once

typedef enum {
	TX = 1,
	RX = 2,
} mode_e;


void transmissionSetup(mode_e mode);

