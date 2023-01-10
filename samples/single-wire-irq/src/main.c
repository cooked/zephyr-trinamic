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

const struct device *const uart1 = DEVICE_DT_GET(UART_NODE1);
const struct device *const uart2 = DEVICE_DT_GET(UART_NODE2);

#define N_RSP 4

static int xfer_bytes;
static char rd_data[12] = {0};
bool rcv = 0;

void uart2_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart2)) {
		return;
	}

	if(uart_irq_rx_ready(uart2)) {

		xfer_bytes += uart_fifo_read(uart2, &rd_data[xfer_bytes], N_RSP - xfer_bytes);

		if (xfer_bytes == N_RSP) {
			xfer_bytes = 0;
			uart_irq_rx_disable(uart2);
			rcv = 1;
		}

	}
}

void main(void)
{
	unsigned char recv;

	if (!device_is_ready(uart1) || !device_is_ready(uart2)) {
		printk("uart devices not ready\n");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart2, uart2_cb, NULL);

	xfer_bytes = 0;

	// flush uart
	uint8_t c;
	while (uart_fifo_read(uart2, &c, 1) > 0) {
		continue;
	}

	int ret;

	while (true) {

		if(rcv) {
			printk("%02X %02X %02X %02X \n", rd_data[0],rd_data[1],rd_data[2],rd_data[3]);
			rcv = 0;
		}

		uart_irq_rx_enable(uart2);

		// TX

		ret = uart_err_check(uart1);
		if(ret) {
			printk("UART error %d", ret);
		}


		unsigned char tx_buf[4] = { 0x0A, 0x0B, 0x0C, 0x0D };
		for (int i=0; i<sizeof(tx_buf); i++) {
			uart_poll_out(uart1, tx_buf[i]);
		}

		// RX should happen in interrupt
		k_sleep(K_MSEC(1000));
	}
}
