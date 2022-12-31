/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// see: https://docs.zephyrproject.org/3.1.0/hardware/peripherals/uart.html

#include <string.h> // memcpy
#include <zephyr/drivers/uart.h>

#include "tmc.h"
#include "tmc_uart.h"

// DMA
K_SEM_DEFINE(tx_done, 1, 1);
K_SEM_DEFINE(tx_aborted, 0, 1);
K_SEM_DEFINE(rx_rdy, 1, 1);
K_SEM_DEFINE(rx_buf_released, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

// interrupt
K_SEM_DEFINE(tx_sem, 1, 1);
K_SEM_DEFINE(rx_sem, 1, 1);

void uart_crc(uint8_t *data, uint8_t data_len) {
	int i,j;
	uint8_t *crc = data + (data_len-1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i=0; i<(data_len-1); i++) { // Execute for all bytes of a message
		currentByte = data[i]; // Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
				*crc = (*crc << 1) ^ 0x07;
			else
				*crc = (*crc << 1);
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}

int uart_read_register(const struct device *uart, uint8_t slave, uint8_t reg, uint8_t *data) {

	// TODO: add check on response address being 0xFF (meaning master)
	// TODO: add check on response checksum

	// TODO: 2nd field, put proper address
	uint8_t tx_buf[N_RD] = { SYNC_NIBBLE, slave, reg, 0 };
	uart_crc(tx_buf, N_RD);

	// ASYNC (w/ DMA)
	// !!! IMPORTANT: remember to config DMA in the dts, see:
	// https://github.com/nrfconnect/sdk-zephyr/blob/main/tests/drivers/uart/uart_async_api/src/test_uart_async.c
	// https://github.com/zephyrproject-rtos/zephyr/issues/39395
	// https://github.com/StefJar/zephyr_stm32_uart3_dma_driver/blob/master/uart3_dma.c

	uint8_t rx_buf[N_RSP+N_RD] = {0};

	// FIXME: buffer also "receives" the sent data, of course (so must be len=N_RSP+N_RD)
	uart_rx_enable(uart, rx_buf, N_RSP + N_RD, 1 * USEC_PER_SEC);
	k_sem_take(&rx_rdy, K_MSEC(1000));

	k_sem_take(&tx_done, K_MSEC(100));
	uart_tx(uart, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC); // SYS_FOREVER_US);
	// wait for TX complete
	k_sem_take(&tx_done, K_MSEC(1000));
	k_sem_give(&tx_done);

	// enable RX and wait to complete
	k_sem_take(&rx_rdy, K_MSEC(1000));
	k_sem_give(&rx_rdy);

	uart_rx_disable(uart);

	// check CRC
	uint8_t tmp[N_RSP] = {0};
	memcpy(tmp, &rx_buf[N_RD], N_RSP-1);
	uart_crc(tmp, N_RSP);
	printk( "Check CRC rcv 0x%02X, calc 0x%02X \n", rx_buf[11], tmp[7]);

	// FIXME: fish for data, getting rid of the TX data
	// FIXME: why is N_RD working and not N_RD-1 ?
	memcpy(data, &rx_buf[N_RD], N_RSP);

	return 0;
}
int uart_write_register(const struct device *uart, uint8_t slave, uint8_t reg, uint32_t value) {

	uint8_t tx_buf[N_WR] = {
		SYNC_NIBBLE,
		slave,
		REG_WRITE_BIT | reg,
		value >> 24, value >> 16, value >> 8, value,
		0
	};
	uart_crc(tx_buf, N_WR);

	k_sem_take(&tx_done, K_MSEC(100));
	uart_tx(uart, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC); // SYS_FOREVER_US);
	// wait for TX complete
	k_sem_take(&tx_done, K_MSEC(1000));
	k_sem_give(&tx_done);

	return 0;
}

// initial TMCs discover and addressing
// see DS sec 5.4 Addressing multiple slaves
void uart_discover(const struct device *uart) {

	// TODO: workflow
	// - poll a register of slave 0x00
	// if response is correct assign slave new "next" address and set NAO
	// if not, discovery end...store address map in flash (via NVS?)
}
void uart_load_slave_map(const struct device *uart){
	// TODO:
	//use the slave addr map stored in memory to init all the connected slaves

}
void uart_flush(const struct device *uart) {
	uint8_t c;

	while (uart_fifo_read(uart, &c, 1) > 0) {
		continue;
	}

	k_sem_reset(&rx_rdy);
	k_sem_reset(&tx_done);
}

// DMA callback
void uart_cb_dma(const struct device *uart, struct uart_event *evt, void *user_data) {

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		//(*(uint32_t *)user_data)++;
		break;
	case UART_RX_RDY:
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

// interrupt callback
void uart_cb(const struct device *uart, void *user_data) {

	// TODO: check that user_data contains in fact what we expect
	const struct device *dev = user_data;
	struct tmc_data *data = dev->data;

	if (!uart_irq_update(uart)) {
		return;
	}

	/*if(uart_irq_tx_ready(uart)) {
		// TODO: add support for RD and WR... change the byte amount accordingly
		//data_length = uart_fifo_fill(uart, uart_buf_tx, N_RD);
		data->xfer_bytes += uart_fifo_fill(uart, &uart_buf_tx[data->xfer_bytes],
				       			4 - data->xfer_bytes);

		if (data->xfer_bytes == 4) {
			printk("TX_IRQ: sent %d bytes, buffer %02X %02X %02X %02X\n",
				data->xfer_bytes, uart_buf_tx[0], uart_buf_tx[1], uart_buf_tx[2], uart_buf_tx[3] );
			data->xfer_bytes = 0;
			uart_irq_tx_disable(uart);
			k_sem_give(&data->tx_sem);
		}
	}*/

	if(uart_irq_rx_ready(uart)) {

		data->xfer_bytes = 0;

		data->xfer_bytes += uart_fifo_read(uart, data->rd_data, N_RSP);
		//data->xfer_bytes += uart_fifo_read(uart, &data->rd_data[data->xfer_bytes],
		//				   		N_RSP - data->xfer_bytes);

		printk("RX_READY read bytes: %u\n", data->xfer_bytes);
		for(int i=0; i<data->xfer_bytes; i++)
			printk("%02X ", data->rd_data[i]);
		printk("\n");

		if (data->xfer_bytes == N_RSP) {
			printk("RX_IRQ: read %d bytes: %02X %02X %02X %02X %02X %02X %02X %02X \n", data->xfer_bytes,
				data->rd_data[0],
				data->rd_data[1],
				data->rd_data[2],
				data->rd_data[3],
				data->rd_data[4],
				data->rd_data[5],
				data->rd_data[6],
				data->rd_data[7]
				);
			data->xfer_bytes = 0;
			uart_irq_rx_disable(uart);
			k_sem_give(&tx_sem);
			k_sem_give(&rx_sem);
		}

	}

}