/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/usb/usb_device.h>

#include "tmc2209.h"

#define SLEEP_TIME_MS   5000

#define S1 DT_NODELABEL(s1)
#define S2 DT_NODELABEL(s2)
#define S3 DT_NODELABEL(s3)

#define H0_NODE DT_ALIAS(h0)
#define H1_NODE DT_ALIAS(h1)
#define H2_NODE DT_ALIAS(h2)
#define F0_NODE DT_ALIAS(f0)
#define F1_NODE DT_ALIAS(f1)

bool toggle;

const struct device *tmc1 = DEVICE_DT_GET(S1);
const struct device *tmc2 = DEVICE_DT_GET(S2);
const struct device *tmc3 = DEVICE_DT_GET(S3);

struct gpio_dt_spec h0 = GPIO_DT_SPEC_GET(H0_NODE, gpios);
struct gpio_dt_spec h1 = GPIO_DT_SPEC_GET(H1_NODE, gpios);
struct gpio_dt_spec h2 = GPIO_DT_SPEC_GET(H2_NODE, gpios);
struct gpio_dt_spec f0 = GPIO_DT_SPEC_GET(F0_NODE, gpios);
struct gpio_dt_spec f1 = GPIO_DT_SPEC_GET(F1_NODE, gpios);

struct gpio_dt_spec gpio4 = GPIO_DT_SPEC_GET( DT_NODELABEL(gp4), gpios );

void main(void)
{
	const struct device *dev;
	int ret;

	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev)) {
		printk("CDC ACM device not ready");
		return;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		printk("Failed to enable USB");
		return;
	}


	if( !device_is_ready(tmc1) ||
		!device_is_ready(tmc2) ||
		!device_is_ready(tmc3)
	) {
		return;
	}

	if( !device_is_ready(h0.port) ) {
		return;
	}

	gpio_pin_configure_dt(&h0, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&h1, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&h2, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&f0, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&f1, GPIO_OUTPUT_ACTIVE);

	gpio_pin_configure_dt(&gpio4, GPIO_OUTPUT_ACTIVE);

	toggle = 0;

	uint32_t count = 0, data;
	uint8_t reg = TMC2209_GSTAT;

	while (1) {

		gpio_pin_set_dt(&gpio4, 1);

		if(toggle) {
			gpio_pin_toggle_dt(&h0);
			gpio_pin_toggle_dt(&h1);
			gpio_pin_toggle_dt(&h2);
			gpio_pin_toggle_dt(&f0);
			gpio_pin_toggle_dt(&f1);
		}
		toggle = !toggle;

		tmc_reg_write(tmc1, 0, TMC2209_VACTUAL, 1);
		tmc_reg_read(tmc1, 0, TMC2209_IFCNT, &data);
		printk( "Count %u - Register value: 0x%08X \n", count, data);
		//tmc_dump(tmc1, 0);

		count++;

		k_msleep(SLEEP_TIME_MS);

	}
}
