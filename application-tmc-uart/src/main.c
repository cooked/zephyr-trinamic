/*
 * Copyright (c) 2022 Wrecklab BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "tmc5160.h"

#define UART_NODE1 DT_ALIAS(uart1)
#define UART_NODE2 DT_ALIAS(uart2)
#define UART_NODE3 DT_ALIAS(uart3)

#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)

bool toggle;

const struct device *tmc;
struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

const struct device *uart1 = DEVICE_DT_GET(UART_NODE1);
const struct device *uart2 = DEVICE_DT_GET(UART_NODE2);
const struct device *uart3 = DEVICE_DT_GET(UART_NODE3);

void main(void)
{
	tmc = DEVICE_DT_GET_ANY(trinamic_tmc5160);

	if (tmc == NULL) {
		return;
	}

	// config UARTs
	if (!device_is_ready(uart1) || !device_is_ready(uart2)) {
		printk("uart devices not ready\n");
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
	uint32_t data, count = 0;

	while (1) {

		if(toggle) {
			if( gpio_pin_toggle_dt(&led) < 0 ) {
				return;
			}
		}

		//tmc_reg_write(tmc, 0, reg, 1);

		//tmc_reg_read(tmc, 0, reg, &data);
		//printk( "Count %u - Register value: 0x%08X \n", count, data);

		toggle = !toggle;
		count++;

		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}
}
