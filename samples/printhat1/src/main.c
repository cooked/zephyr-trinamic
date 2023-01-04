/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "tmc2130.h"

#define SLEEP_TIME_MS   1000

#define S1 DT_NODELABEL(s1)
#define S2 DT_NODELABEL(s2)
#define S3 DT_NODELABEL(s3)
#define S4 DT_NODELABEL(s4)

#define H0_NODE DT_ALIAS(h0)
#define H1_NODE DT_ALIAS(h1)
#define F0_NODE DT_ALIAS(f0)
#define F1_NODE DT_ALIAS(f1)

bool toggle;

const struct device *tmc1 = DEVICE_DT_GET(S1);
const struct device *tmc2 = DEVICE_DT_GET(S2);
const struct device *tmc3 = DEVICE_DT_GET(S3);
const struct device *tmc4 = DEVICE_DT_GET(S4);

struct gpio_dt_spec h0 = GPIO_DT_SPEC_GET(H0_NODE, gpios);
struct gpio_dt_spec h1 = GPIO_DT_SPEC_GET(H1_NODE, gpios);
struct gpio_dt_spec f0 = GPIO_DT_SPEC_GET(F0_NODE, gpios);
struct gpio_dt_spec f1 = GPIO_DT_SPEC_GET(F1_NODE, gpios);

void main(void)
{
	if( !device_is_ready(tmc1) ||
		!device_is_ready(tmc2) ||
		!device_is_ready(tmc3) ||
		!device_is_ready(tmc4) ) {
		return;
	}

	if( !device_is_ready(h0.port) ) {
		return;
	}

	gpio_pin_configure_dt(&h0, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&h1, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&f0, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&f1, GPIO_OUTPUT_ACTIVE);

	toggle = 0;

	uint32_t count = 0, data;
	uint8_t reg = TMC2130_GSTAT;

	while (1) {

		printk(" Heeeellooo %d\n", toggle);

		if(toggle) {
			gpio_pin_toggle_dt(&h0);
			gpio_pin_toggle_dt(&h1);
			gpio_pin_toggle_dt(&f0);
			gpio_pin_toggle_dt(&f1);
		}
		toggle = !toggle;


		tmc_reg_write(tmc1, 0, reg, 1);
		tmc_reg_read(tmc1, 0, reg, &data);
		printk( "Count %u - Register value: 0x%08X \n", count, data);

		count++;

		k_msleep(SLEEP_TIME_MS);

	}
}
