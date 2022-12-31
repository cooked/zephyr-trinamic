/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC2209_H_
#define ZEPHYR_DRIVERS_TMC2209_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#define DEFAULT_FCLK		12000000.0f	// [Hz]
#define DEFAULT_USTEP		256			// [-]
#define DEFAULT_STEPS_TURN 	200 * DEFAULT_USTEP
#define RPM_TO_PPS			DEFAULT_STEPS_TURN / 60.0f
#define DEFAULT_ROT_DIS 	1000		// [mm]


// map
typedef struct attr {
    uint8_t 	reg;
	uint32_t	mask;
    uint8_t		shift;
} attr;

struct field {
    char *name;
    attr a;
};

struct reg {
    uint8_t reg;
    char 	*rwc;
	//char 	*fields[];
};

attr get_field(char *key);

//
uint8_t tmc_reg_read(const struct device *dev, uint8_t reg, uint32_t *data);
uint8_t tmc_reg_write(const struct device *dev, uint8_t reg, uint32_t value);

int32_t tmc_get(const struct device *dev, char *key);
void 	tmc_set(const struct device *dev, char *key, int32_t value);

int tmc_init(const struct device *dev);

// general
void tmc_set_irun_ihold(const struct device *dev, uint8_t irun, uint8_t ihold);

// velocity mode
void tmc_run(const struct device *dev, int32_t speed, int32_t acc);

int tmc_test(const struct device *dev);
int tmc_dump(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_TMC2209_H_ */
