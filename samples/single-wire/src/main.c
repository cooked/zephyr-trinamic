/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#define UART_NODE1 DT_ALIAS(uart1)
#define UART_NODE2 DT_ALIAS(uart2)

const struct device *const sl_uart1 = DEVICE_DT_GET(UART_NODE1);
const struct device *const sl_uart2 = DEVICE_DT_GET(UART_NODE2);

void main(void)
{
	unsigned char recv;

	if (!device_is_ready(sl_uart1) || !device_is_ready(sl_uart2)) {
		printk("uart devices not ready\n");
		return;
	}

	while (true) {

		uint8_t i;

		// TX
		//uart_poll_out(sl_uart1, 'c');
		unsigned char tx_buf[4] = { 0x0A, 0x0B, 0x0C, 0x0D };
		for(i=0; i<sizeof(tx_buf); i++) {
			uart_poll_out(sl_uart1, tx_buf[i]);
		}

		/* give the uarts some time to get the data through */
		k_msleep(50);


		int ret = uart_poll_in(sl_uart2, &recv);
		if (ret < 0) {
			printk("Receiving failed. Error: %d\n", ret);
		} else {
			printk("Received %c\n", recv);
		}

		// RX
		/*i=0;
		while(i<4) {
			ret = uart_poll_in(sl_uart2, &recv[i]);
			if( ret < 0 ) {
				k_usleep(10);
				//printk("err\n");
			} else {
				printk(" %02X ", &recv[i]);
				i++;
			}
		}
		printk("\n");*/

		k_sleep(K_MSEC(2000));
	}
}
