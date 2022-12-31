/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC5160_SPI_H_
#define ZEPHYR_DRIVERS_TMC5160_SPI_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define REG_SPI_READ_BIT 	0x00
#define REG_SPI_WRITE_BIT 	0x80

// TODO: generalize to to all TMC SPI drivers

int spi_read_register(const struct spi_dt_spec *bus, uint8_t reg, uint8_t *data);
int spi_write_register(const struct spi_dt_spec *bus, uint8_t reg, uint32_t value);

#endif /* ZEPHYR_DRIVERS_TMC5160_SPI_H_ */
