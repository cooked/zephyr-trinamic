/*
 * Copyright (c) 2022 Wrecklab BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <sys/util.h>
#include "tmc5160_spi.h"

int spi_read_register(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *data)
{
	// always 40bit, 1 byte address + 4byte data
	uint8_t tx_buf[5] = {0x7F & reg, 0, 0, 0, 0};


	const struct spi_buf spi_buf_tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1,
	};


	struct spi_buf spi_buf_rx = {
		.buf = data,
		.len = sizeof(tx_buf),
	};
	struct spi_buf_set rx = {
		.buffers = &spi_buf_rx,
		.count = 1,
	};


	int ret = 0;

	// send command (and ignore rcv)
	ret = spi_transceive_dt(bus, &tx, &rx);
	//	send dummy and keep rcv (result from previous transaction)
	ret = spi_transceive_dt(bus, &tx, &rx);

	return ret;
}

int spi_write_register(const struct spi_dt_spec *bus, uint8_t reg, uint32_t value)
{
	// TODO: replace with
	// https://docs.zephyrproject.org/3.1.0/kernel/util/index.html#c.byteswp

	/*uint8_t tx_buf[5];
	tx_buf[0] = REG_SPI_WRITE_BIT | reg;
	*(uint32_t*)&tx_buf[1] = val;*/
	uint8_t tx_buf[5] = {
		REG_SPI_WRITE_BIT | reg,
		value >> 24, value >> 16, value >> 8, value
	};

	const struct spi_buf spi_buf_tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf)
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1
	};

	int ret = 0;

	// send command (and ignore rcv)
	ret = spi_transceive_dt(bus, &tx, NULL);
	//	send dummy and keep rcv (result from previous transaction)
	//ret = spi_transceive_dt(bus, &tx, &rx);

	return ret;
}