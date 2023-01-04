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
K_SEM_DEFINE(rx_buf_released, 1, 1);
K_SEM_DEFINE(rx_disabled, 1, 1);

// interrupt
//K_SEM_DEFINE(tx_sem, 1, 1);
//K_SEM_DEFINE(rx_sem, 1, 1);

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


/*int uart_init(const struct device *dev) {

	const struct tmc_config *cfg = dev->config;

	k_sem_take(&rx_disabled, K_MSEC(1000));
	uart_rx_disable(cfg->uart);
	uart_callback_set(cfg->uart, cfg->cb_dma, (void *)dev);
	k_sem_take(&rx_disabled, K_MSEC(1000));
	k_sem_give(&rx_disabled);

	k_sem_give(&tx_done);
	k_sem_give(&rx_rdy);
	k_sem_give(&rx_buf_released);

	return 0;
}*/

int uart_check_crc(uint8_t *data) {

	// rx CRC
	uint8_t crc;
	memcpy(&crc, &data[N_RSP_SHIFT+7], 	1);

	// calc CRC
	uint8_t tmp[N_RSP] = {0};
	memcpy(tmp, &data[N_RSP_SHIFT], N_RSP-1);
	uart_crc(tmp, N_RSP);

	if(crc != tmp[N_RSP-1])
		return -EINVAL;

	return 0;

}

int uart_read_register(const struct device *dev, uint8_t slave, uint8_t reg, uint8_t *value) {

	const struct tmc_config *cfg = dev->config;
	struct tmc_data *data = dev->data;

	int ret;

	// TODO: add check on response address being 0xFF (meaning master)
	// TODO: add check on response checksum



	// ASYNC (w/ DMA)
	// !!! IMPORTANT: remember to config DMA in the dts, see:
	// https://github.com/nrfconnect/sdk-zephyr/blob/main/tests/drivers/uart/uart_async_api/src/test_uart_async.c
	// https://github.com/StefJar/zephyr_stm32_uart3_dma_driver/blob/master/uart3_dma.c

#ifdef CONFIG_UART_ASYNC_API
	//k_sem_take(&tx_done, K_FOREVER);
	//k_sem_take(&rx_buf_released, K_FOREVER);
	//uart_tx(cfg->uart, d->tx_buf, N_RD, SYS_FOREVER_US);
	// wait for RX complete
	//k_sem_take(&rx_buf_released, K_FOREVER);
#endif

	/* Make sure last command has been transferred */
	//k_sem_take(&data->tx_sem, UART_WAIT);
	//k_sem_reset(&data->rx_sem);

	uint8_t i;


	// TX
	uint8_t tx_buf[N_RD] = { SYNC_NIBBLE, slave, reg, 0 };
	uart_crc(tx_buf, N_RD);
	for(i=0; i<sizeof(tx_buf); i++) {
		uart_poll_out(cfg->uart_dev, tx_buf[i]);
	}

	k_usleep(40);

	// RX
	i=0;
	while(i<8) {
		ret = uart_poll_in(cfg->uart_dev, &data->rd_data[i]);

		if( ret ) {
			k_usleep(10);
			printk("err\n");
		} else {
			printk(" %02X ", data->rd_data[i]);
			i++;
		}
	}
	printk("\n");

	// TODO: here we add the extra needed bytes (shift) to get the full rsp
	//data->msg_bytes = N_RSP + N_RSP_SHIFT;


	/*
	// RX and wait for completion (timeout 1s)
	uart_irq_rx_enable(cfg->uart_dev);
	ret = k_sem_take(&data->rx_sem, UART_WAIT);

	ret = uart_check_crc(data->rd_data);
	if(ret) {
		printk( "UART CRC error");
		return ret;
	}*/

	/*printk(" - data %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
		data->rd_data[0], data->rd_data[1],
		data->rd_data[2], data->rd_data[3],
		data->rd_data[4], data->rd_data[5],
		data->rd_data[6], data->rd_data[7],
		data->rd_data[8], data->rd_data[9],
		data->rd_data[10], data->rd_data[11]
	);

	memcpy(&value, &data->rd_data[N_RSP_SHIFT], 4);
	*/

	return ret;

}
int uart_write_register(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value) {

	const struct tmc_config *cfg = dev->config;
	struct tmc_data *data = dev->data;

	uint8_t tx_buf[N_WR] = {
		SYNC_NIBBLE,
		slave,
		REG_WRITE_BIT | reg,
		value >> 24, value >> 16, value >> 8, value,
		0
	};
	uart_crc(tx_buf, N_WR);

//	k_sem_take(&tx_done, K_FOREVER);
//	uart_tx(cfg->uart, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC); // SYS_FOREVER_US);
	// wait for TX complete
//	k_sem_take(&tx_done, K_MSEC(1000));
//	k_sem_give(&tx_done);

	return 0;
}


void tmc_uart_flush(const struct device *uart_dev) {
	uint8_t c;
	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
}

void tmc_uart_isr(const struct device *uart_dev, void *user_data)
{

	const struct device *dev = user_data;
	struct tmc_data *data = dev->data;
	const struct tmc_config *cfg = dev->config;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	// RECV
	if (uart_irq_rx_ready(uart_dev)) {
		data->xfer_bytes += uart_fifo_read(uart_dev, &data->rd_data[data->xfer_bytes],
						   data->msg_bytes - data->xfer_bytes);

		// RECV complete
		if (data->xfer_bytes == data->msg_bytes) {
			data->xfer_bytes = 0;
			uart_irq_rx_disable(uart_dev);
			k_sem_give(&data->rx_sem);
			if (data->has_rsp) {
				k_sem_give(&data->tx_sem);
			}
		}
	}

	// SEND
	/*if (uart_irq_tx_ready(uart_dev)) {
		data->xfer_bytes +=
			uart_fifo_fill(uart_dev, &data->rd_data[data->xfer_bytes],
				       data->msg_bytes - data->xfer_bytes);

		// SEND complete
		if (data->xfer_bytes == data->msg_bytes) {
			data->xfer_bytes = 0;
			uart_irq_tx_disable(uart_dev);
			if (!data->has_rsp) {
				k_sem_give(&data->tx_sem);
			}
		}
	}*/
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
/*void uart_cb_dma(const struct device *uart, struct uart_event *evt, void *user_data) {

	const struct device *dev = (const struct device *) user_data;

	struct tmc_data *data = dev->data;
	const struct tmc_config *cfg = dev->config;

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		uart_rx_enable(cfg->uart_dev, data->rx_buf, N_RSP, SYS_FOREVER_US);
		break;

	case UART_TX_ABORTED:
		printk( "TX ABORTED\n");
		break;

	case UART_RX_RDY:
		uart_rx_disable(cfg->uart_dev);
		//k_sem_give(&rx_rdy);
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

}*/
