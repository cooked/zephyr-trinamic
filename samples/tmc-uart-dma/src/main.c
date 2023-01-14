/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include "tmc5160.h"

#define UART_NODE1 DT_ALIAS(uart1)
#define UART_NODE2 DT_ALIAS(uart2)

#define SLEEP_TIME_MS   1000

bool toggle;

const struct device *tmc0 = DEVICE_DT_GET( DT_ALIAS(tmc0) );
struct gpio_dt_spec led = GPIO_DT_SPEC_GET( DT_ALIAS(led0), gpios);

const struct device *uart1 = DEVICE_DT_GET(UART_NODE1);
const struct device *uart2 = DEVICE_DT_GET(UART_NODE2);
//const struct device *uart3 = DEVICE_DT_GET(UART_NODE3);

void main(void)
{
	// config UARTs
	if (!device_is_ready(uart1) || !device_is_ready(uart2)) {
		printk("uart devices not ready\n");
		return;
	}

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
	toggle = 1;

	uint8_t reg = TMC5160_GSTAT;
	//reg = TMC5160_INP_OUT;
	reg = TMC5160_RAMPMODE;
	uint32_t count = 0, data;

	while (1) {

		if(toggle) {
			if( gpio_pin_toggle_dt(&led) < 0 ) {
				return;
			}
		}

		//tmc_reg_write(tmc0, 0, reg, 1);
		//tmc_reg_read(tmc0, 0, TMC5160_IFCNT, &data);
		//printk( "Count %u - Register value: 0x%08X \n", count, data);

		toggle = !toggle;
		count++;

		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}
}
