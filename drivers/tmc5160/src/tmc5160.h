/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC5160_H_
#define ZEPHYR_DRIVERS_TMC5160_H_

#include "tmc.h"

#define DEFAULT_FCLK		12000000.0f	// [Hz]

#define TMC_T				((float)(1 << 23) / DEFAULT_FCLK)
#define TMC_TA2				((float)(1 << 41) / (DEFAULT_FCLK*DEFAULT_FCLK))

#define DEFAULT_AMAX 		500			// ???
#define DEFAULT_VMAX 		100000		//
#define DEFAULT_VSTOP		10			//

//self.accel_factor = float(1 << 41) / fCLK**2 * self._microsteps / self._full_step_dist
//self.accel_factor_t = float(1 << 17) / fCLK

// ramp
int32_t tmc_get_xtarget(const struct device *dev, uint8_t slave);
void 	tmc_set_xtarget(const struct device *dev, uint8_t slave, int32_t target);
void tmc_set_vstart(const struct device *dev, uint8_t slave, uint32_t vstart);
void tmc_set_a1(const struct device *dev, uint8_t slave, uint32_t a1);
void tmc_set_amax(const struct device *dev, uint8_t slave, uint32_t amax);
void tmc_set_v1(const struct device *dev, uint8_t slave, uint32_t v1);
void tmc_set_vmax(const struct device *dev, uint8_t slave, uint32_t vmax);
void tmc_set_dmax(const struct device *dev, uint8_t slave, uint32_t dmax);
void tmc_set_d1(const struct device *dev, uint8_t slave, uint32_t d1);
void tmc_set_vstop(const struct device *dev, uint8_t slave, uint32_t vstop);


#endif /* ZEPHYR_DRIVERS_TMC5160_H_ */
