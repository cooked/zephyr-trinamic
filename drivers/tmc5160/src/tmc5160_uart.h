/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC5160_UART_H_
#define ZEPHYR_DRIVERS_TMC5160_UART_H_

#include <device.h>

#define SYNC_NIBBLE 	0x05
#define REG_WRITE_BIT	0x80

#define SLAVEADDR 		0x01 // 0x01 default (internal pu resistor)
#define N_WR 			8
#define N_RD 			4
#define N_RSP 			8

int uart_read_register(const struct device *uart, uint8_t slave, uint8_t reg, uint8_t *data);
int uart_write_register(const struct device *uart, uint8_t slave, uint8_t reg, uint32_t value);

void uart_discover(const struct device *uart);
void uart_load_slave_map(const struct device *uart);

void uart_cb(const struct device *uart, void *user_data);
void uart_cb_dma(const struct device *uart, struct uart_event *evt, void *user_data);

void uart_flush(const struct device *uart_dev);

#endif /* ZEPHYR_DRIVERS_TMC5160_UART_H_ */
