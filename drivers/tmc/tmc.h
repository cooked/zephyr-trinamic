/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC_H_
#define ZEPHYR_DRIVERS_TMC_H_

#include <string.h>

#include <zephyr/drivers/gpio.h>

#ifdef CONFIG_TMC_SPI
#include <zephyr/drivers/spi.h>
#endif

#ifdef CONFIG_TMC_UART
#include <zephyr/drivers/uart.h>
#endif

// common config
#define DEFAULT_USTEP		256			// [-]
#define DEFAULT_STEPS_TURN 	200 * DEFAULT_USTEP
#define RPM_TO_PPS			DEFAULT_STEPS_TURN / 60.0f
#define DEFAULT_ROT_DIS 	1000		// [mm]

// driver structs
struct tmc_data {
	uint16_t r_sens;
	uint16_t i_run;
	uint16_t i_hold;

#if CONFIG_TMC_UART

	// async
	uint8_t tx_buf[8];
	uint8_t rx_buf[8];

	uint8_t rd_data[16];
	uint32_t data;			// Register data (payload in response message)

	uint8_t tx_bytes;	// transferred bytes
	uint8_t xfer_bytes;	// transferred bytes
	uint8_t msg_bytes;	// msg length (set at runtime )

	bool has_rsp;		// has a response? // TODO: always true


	struct k_sem tx_sem;
	struct k_sem rx_sem;



#endif

};

struct tmc_config {

#if CONFIG_TMC_SPI
	const struct spi_dt_spec spi;
#endif
#if CONFIG_TMC_UART
	const struct device *uart_dev;
	//uart_irq_callback_user_data_t cb;
	uart_callback_t cb;
#endif

	uint8_t slave;

	struct gpio_dt_spec diag0_pin;
	struct gpio_dt_spec diag1_pin;

	uint8_t run_current;
	uint8_t hold_current;
	float r_sens;

	float rpm_to_hzs;
	float rotation_distance; // [mm/turn] linear distance travelled per single motor turn

};


// registers/fileds mapping
typedef struct attr {
    uint8_t 	reg;
	uint32_t	mask;
    uint8_t		shift;
} attr;

struct field {
    char *name;
    attr a;
};

struct field_reg {
    char *name;			// name of the reg containing the field
    struct field f;		//
};

struct reg_attr {
    uint8_t reg;
    char 	*rwc;
};
struct reg {
	char *name;
    struct reg_attr attr;
};

//
// API
//
attr get_field(char *key, struct field *fields);

uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data);
uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value);

int32_t tmc_get(const struct device *dev, uint8_t slave, char *key);
void 	tmc_set(const struct device *dev, uint8_t slave, char *key, int32_t value);

//
int 	tmc_init(const struct device *dev, uint8_t slave);
void 	tmc_set_irun_ihold(const struct device *dev, uint8_t slave, uint8_t irun, uint8_t ihold);
void    tmc_run(const struct device *dev, uint8_t slave, int32_t speed, int32_t acc);
int     tmc_dump(const struct device *dev, uint8_t slave);
int 	tmc_test(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_TMC_H_ */
