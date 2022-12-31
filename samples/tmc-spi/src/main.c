/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "tmc5160.h"

#define SLEEP_TIME_MS   1000

const struct device *tmc0 = DEVICE_DT_GET( DT_ALIAS(tmc0) );
struct gpio_dt_spec led = GPIO_DT_SPEC_GET( DT_ALIAS(led0), gpios);

bool toggle;

void main(void) {

	if( !device_is_ready(tmc0) ) {
		return;
	}

	if( !device_is_ready(led.port) ) {
		return;
	}

	if( gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0 ) {
		return;
	}

	toggle = 0;

	while (1) {

		if(toggle) {
			if( gpio_pin_toggle_dt(&led) < 0 ) {
				return;
			}
		}
		toggle = !toggle;

		k_msleep(SLEEP_TIME_MS);

	}
}
