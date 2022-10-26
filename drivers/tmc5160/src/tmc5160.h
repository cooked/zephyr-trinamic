/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC5160_H_
#define ZEPHYR_DRIVERS_TMC5160_H_

#include <drivers/gpio.h>

#if CONFIG_TMC5160_SPI
#include <drivers/spi.h>
#elif CONFIG_TMC5160_UART
#include <drivers/uart.h>
#endif

#include "tmc5160_reg.h"
#include "tmc5160_fields.h"


#define DEFAULT_FCLK		12000000.0f	// [Hz]
#define DEFAULT_USTEP		256			// [-]
#define DEFAULT_STEPS_TURN 	200 * DEFAULT_USTEP
#define RPM_TO_PPS			DEFAULT_STEPS_TURN / 60.0f
#define DEFAULT_ROT_DIS 	1000		// [mm]

#define TMC_T				((float)(1 << 23) / DEFAULT_FCLK)
#define TMC_TA2				((float)(1 << 41) / (DEFAULT_FCLK*DEFAULT_FCLK))

#define DEFAULT_AMAX 		500			// ???
#define DEFAULT_VMAX 		100000		//
#define DEFAULT_VSTOP		10			//

//self.accel_factor = float(1 << 41) / fCLK**2 * self._microsteps / self._full_step_dist
//self.accel_factor_t = float(1 << 17) / fCLK

struct tmc5160_data_t {
	uint16_t r_sens;
	uint16_t i_run;
	uint16_t i_hold;

#if CONFIG_TMC5160_UART
	struct k_sem tx_sem;
	struct k_sem rx_sem;

	/* Max data length is 16 bits */
	uint16_t data;
	/* Command buf length is 9 */
	uint8_t xfer_bytes;
	uint8_t rd_data[8];

#endif

};

struct tmc5160_config {

#if CONFIG_TMC5160_SPI
	const struct spi_dt_spec spi;
#elif CONFIG_TMC5160_UART
	const struct device *uart;
	uart_irq_callback_user_data_t cb;
	uart_callback_t cb_dma;
#endif

	struct gpio_dt_spec diag0_pin;
	struct gpio_dt_spec diag1_pin;

	uint8_t run_current;
	uint8_t hold_current;
	float r_sens;

	float rpm_to_hzs;

	float rotation_distance; // [mm/turn] linear distance travelled per single motor turn

};

// map
typedef struct attr {
    uint8_t 	reg;
	uint32_t	mask;
    uint8_t		shift;
} attr;

struct field {
    char *name;
    attr a;
};

struct reg {
    uint8_t reg;
    char 	*rwc;
	//char 	*fields[];
};

attr get_field(char *key);

//
uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data);
uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value);

int32_t tmc_get(const struct device *dev, uint8_t slave, char *key);
void 	tmc_set(const struct device *dev, uint8_t slave, char *key, int32_t value);

int tmc_init(const struct device *dev, uint8_t slave);

// general
void tmc_set_irun_ihold(const struct device *dev, uint8_t slave, uint8_t irun, uint8_t ihold);
// ramp
int32_t tmc_get_xtarget(const struct device *dev, uint8_t slave);
void 	tmc_set_xtarget(const struct device *dev, uint8_t slave, int32_t target);
void tmc_set_vstart(const struct device *dev, uint8_t slave, uint32_t vstart);
void tmc_set_a1(const struct device *dev, uint8_t slave, uint32_t a1);
void tmc_set_amax(const struct device *dev, uint8_t slave, uint32_t amax);
void tmc_set_v1(const struct device *dev, uint8_t slave, uint32_t v1);
void tmc_set_vmax(const struct device *dev, uint8_t slave, uint32_t vmax);
void tmc_set_dmax(const struct device *dev, uint8_t slave, uint32_t dmax);
void tmc_set_d1(const struct device *dev, uint8_t slave, uint32_t d1);
void tmc_set_vstop(const struct device *dev, uint8_t slave, uint32_t vstop);

// velocity mode
void tmc_run(const struct device *dev, uint8_t slave, int32_t speed, int32_t acc);

int tmc_test(const struct device *dev);
int tmc_dump(const struct device *dev, uint8_t slave);

#endif /* ZEPHYR_DRIVERS_TMC5160_H_ */
