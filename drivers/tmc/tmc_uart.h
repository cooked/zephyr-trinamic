/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC_UART_H_
#define ZEPHYR_DRIVERS_TMC_UART_H_

#include <zephyr/device.h>

#define SYNC_NIBBLE 	0x05
#define REG_WRITE_BIT	0x80

#define SLAVEADDR 		0x01 // 0x01 default (internal pu resistor)
#define N_WR 			8
#define N_RD 			4
#define N_RSP 			8

// TODO:this must be tweeket according to type of comm
// 3: ok for f103 nucleo using TX poll + RX interrupt
#define N_RSP_SHIFT 	3

/* Arbitrary max duration to wait for the response */
#define UART_WAIT K_SECONDS(1)


void tmc_uart_flush(const struct device *uart_dev);

void tmc_uart_cb(const struct device *uart_dev, void *user_data);
void tmc_uart_cb_dma(const struct device *uart_dev, struct uart_event *evt, void *user_data);

int tmc_uart_init(const struct device *dev);

int uart_read_register(const struct device *dev, uint8_t slave, uint8_t reg, uint8_t *value);
int uart_write_register(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value);

void uart_discover(const struct device *uart);
void uart_load_slave_map(const struct device *uart);


#endif /* ZEPHYR_DRIVERS_TMC_UART_H_ */
