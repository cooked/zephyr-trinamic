/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "tmc5160.h"

#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)

bool toggle;

const struct device *tmc;
struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void main(void)
{
	tmc = DEVICE_DT_GET_ANY(trinamic_tmc5160);

	if (tmc == NULL) {
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
