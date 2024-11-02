/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/sys/util.h>

#define SLEEP_TIME_MS   100
#define SPEED_STEP_S  	200

uint8_t toggle, toggle_old;

const struct device *tmc0 = DEVICE_DT_GET( DT_ALIAS(tmc0) );
const struct device *mot0 = DEVICE_DT_GET( DT_ALIAS(motor0) );
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

int main(void) {

	int ret;

	// config button
	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return -ENODEV;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return -ENODEV;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);



	// check tmc
	if (!device_is_ready(mot0)) {
		printk("TMC device not ready\n");
		return -ENODEV;
	}
	stepper_enable(mot0, true);
	stepper_move(mot0, 200);
	//stepper_enable_constant_velocity_mode(mot0, STEPPER_DIRECTION_POSITIVE, SPEED_STEP_S);


	// config LED
	if( !device_is_ready(led.port) ) {
		return -ENODEV;
	}
	if( gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0 ) {
		return -ENODEV;
	}
	gpio_pin_set_dt(&led, 0);


	int8_t count = 0;

	while (1) {

		// set LED
		if( gpio_pin_toggle_dt(&led) < 0 ) {
			return -EIO;
		}

		/*if(toggle!=toggle_old) {

			count++;

			// start/stop motor
			if(count == 2){
				stepper_enable_constant_velocity_mode(mot0, STEPPER_DIRECTION_POSITIVE, SPEED_STEP_S);
			} else if(count == 1) {
				stepper_enable_constant_velocity_mode(mot0, STEPPER_DIRECTION_NEGATIVE, SPEED_STEP_S);
			}

			if(count == 2)
				count=-1;

			//tmc_run(tmc0, 0, count*60, 0);
			// test ustep=1
			stepper_move(mot0, 200);

			toggle_old = toggle;
		}*/

		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
