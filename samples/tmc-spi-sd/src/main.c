/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

#include "tmc5160.h"

#define SLEEP_TIME_MS   1000

uint8_t toggle, toggle_old;

const struct device *tmc0 = DEVICE_DT_GET( DT_ALIAS(tmc0) );
struct gpio_dt_spec led = GPIO_DT_SPEC_GET( DT_ALIAS(led0), gpios);

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

// start/stop motor
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
	toggle = !toggle;
}

void main(void)
{
	int ret;

	// config button
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);


	// check tmc
	if (!device_is_ready(tmc0)) {
		printk("TMC device not ready\n");
		return;
	}

	// config LED
	if( !device_is_ready(led.port) ) {
		return;
	}
	if( gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0 ) {
		return;
	}
	gpio_pin_set_dt(&led, 0);

	// set stand-still in speed mode
	//tmc_run(tmc0, 0, 0, 0);
	int8_t count = 0;

	while (1) {

		if(toggle!=toggle_old) {
			// set LED
			if( gpio_pin_set_dt(&led, toggle) < 0 ) {
				return;
			}

			count++;
			// start/stop motor
			if(count == 2)
				count=-1;

			tmc_run(tmc0, 0, count*60, 0);

			toggle_old = toggle;
		}

		k_sleep(K_MSEC(100));
	}

}
