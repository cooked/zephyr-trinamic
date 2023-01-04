/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC2130_H_
#define ZEPHYR_DRIVERS_TMC2130_H_

#include "tmc.h"
// from hal_trinamic
#include "TMC2130_Register.h"
#include "TMC2130_Mask_Shift.h"

// NOTE: keep it up to date according to tmcXXXX_map.c
#define NREG 28

#define DEFAULT_FCLK		12000000.0f	// [Hz]


#endif /* ZEPHYR_DRIVERS_TMC2130_H_ */
