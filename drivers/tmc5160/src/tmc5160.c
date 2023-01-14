/*
 * Copyright (c) 2022, Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// mind include path
// https://stackoverflow.com/questions/72294929/location-of-source-file-include-drivers-gpio-h

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>	// sys_to_xxx()
#include <zephyr/init.h>

#include "tmc.h"
#include "tmc5160.h"

#ifdef CONFIG_TMC_SPI
#include "tmc_spi.h"
#endif
#ifdef CONFIG_TMC_UART
#include "tmc_uart.h"
#endif

// piggyback on SPI log level for now
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TMC5160, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT trinamic_tmc5160

extern struct field fields[];
extern struct reg regs[];


static int tmc5160_init(const struct device *dev)
{
	struct tmc_data *data = dev->data;
	const struct tmc_config *cfg = dev->config;

	int res = 0;

	LOG_DBG("TMC5160 INIT");

#if CONFIG_TMC_SPI
	if (!spi_is_ready(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
#elif CONFIG_TMC_UART

#if CONFIG_UART_INTERRUPT_DRIVEN

	LOG_DBG("TMC UART irq-mode \n");

	uart_irq_rx_disable(cfg->uart_dev);
	uart_irq_tx_disable(cfg->uart_dev);

	tmc_uart_flush(cfg->uart_dev);

	//uart_irq_callback_user_data_set(cfg->uart_dev, cfg->cb, (void *)dev);

	k_sem_init(&data->rx_sem, 1, 1);
	k_sem_init(&data->tx_sem, 1, 1);

	data->xfer_bytes = 0;

	uart_irq_rx_enable(cfg->uart_dev);

#elif CONFIG_UART_ASYNC_API

	LOG_DBG("TMC UART dma-mode \n");
	//tmc_uart_init(dev);

#else

	LOG_DBG("TMC UART poll-mode \n");
	//tmc_uart_init(dev);

#endif

	// TODO: here we should initialize all slave... maybe with the addressing first
	tmc_init(dev, 0);

#endif

	LOG_DBG("tmc5160_init done");

	return res;
}

// TMC r/w registers
uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data) {

	const struct tmc_config *cfg = dev->config;

#if CONFIG_TMC_SPI
	uint8_t buf[5] = {0};
	spi_read_register( &(cfg->spi), reg, buf );
	// TODO: replace with sys_to.... from byteorder.h
	//sys_be32_to_cpu();
	*data = assemble_32(&buf[1]);
#elif CONFIG_TMC_UART
	uart_read_register(dev, slave, reg, data);
#endif

	return 0;
}

uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value) {

	const struct tmc_config *cfg = dev->config;

#if CONFIG_TMC_SPI
	spi_write_register( &(cfg->spi), reg, value);
#elif CONFIG_TMC_UART
	uart_write_register( dev, slave, reg, value );
#endif

	return 0;
}

int tmc_init(const struct device *dev, uint8_t slave) {

	const struct tmc_config *cfg = dev->config;

	tmc_reg_write(dev, slave, TMC5160_GSTAT, 		0x7			); // clear errors
	//tmc_reg_write(dev, slave, TMC5160_CHOPCONF, 	0x000100C3	); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)

	// write only
	tmc_set_irun_ihold(dev, slave, cfg->run_current, cfg->hold_current);

	//tmc_reg_write(dev, slave, TMC5160_TPOWERDOWN, 	0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
	//tmc_reg_write(dev, slave, TMC5160_GCONF, 		0x00000004); // EN_PWM_MODE=1 enables StealthChop (with default PWM_CONF)
	//tmc_reg_write(dev, slave, TMC5160_TPWMTHRS, 	0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM

	// prevent motion
	tmc_reg_write(dev, slave, TMC5160_XTARGET,		0);
	tmc_reg_write(dev, slave, TMC5160_XACTUAL,		0);

	// add some ramp init
	// see DS pag. 116 Getting Started
	tmc_reg_write(dev, slave, TMC5160_A1, 			DEFAULT_AMAX * 2);
	tmc_reg_write(dev, slave, TMC5160_AMAX, 		DEFAULT_AMAX);
	tmc_reg_write(dev, slave, TMC5160_V1, 			DEFAULT_VMAX / 2);
	tmc_reg_write(dev, slave, TMC5160_VMAX, 		DEFAULT_VMAX);
	tmc_reg_write(dev, slave, TMC5160_DMAX, 		DEFAULT_AMAX);
	tmc_reg_write(dev, slave, TMC5160_D1, 			DEFAULT_AMAX * 2);
	tmc_reg_write(dev, slave, TMC5160_VSTOP, 		DEFAULT_VSTOP);

	// reset position
	tmc_reg_write(dev, slave, TMC5160_RAMPMODE,	0);	// set position mode

	tmc_reg_write(dev, slave, TMC5160_RAMPSTAT,	0);	// clear ramp status

	return 0;

}
void tmc_run(const struct device *dev, uint8_t slave, int32_t speed, int32_t acc) {

	//const struct tmc5160_config *cfg = dev->config;
	//printk( "rot_dist	: %f \n", cfg->rotation_distance);
	//printk( "speed conv	: %f, %f,  %f \n", (float)speed, RPM_TO_PPS, (float)speed * RPM_TO_PPS / TMC_T);

	// AMAX acceleration and deceleration value in velocity mode (DS pag. 40)
	if(acc!=0)
		tmc_reg_write(dev, slave, TMC5160_AMAX, (uint32_t)acc );

	// VMAX velocity value in velocity mode (DS pag. 40)
	tmc_reg_write(dev, slave, TMC5160_VMAX, (uint32_t)((float)speed * RPM_TO_PPS / TMC_T) );

	// direction
	if(speed>0) {
		// + velocity mode
		tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 1);
	} else if(speed<0) {
		// - velocity mode
		tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 2);
	} else {
		// keep existing ramp_mode, but stop motion (speed=0)
	}
}

int tmc_dump(const struct device *dev, uint8_t slave) {

	uint32_t data;

	for(uint16_t reg = 0; reg < 1; reg++) {
		// dump only if readable
		if(	strchr( regs[reg].attr.rwc, 'r') ) {
			tmc_reg_read( dev, slave, regs[reg].attr.reg, &data);
			printk(" - Register (0x%02X) %s	: 0x%08X \n", regs[reg].attr.reg, regs[reg].name, data);
		}
	}

	return 0;
}

// tmc register's field getter/setter
int32_t tmc_get(const struct device *dev, uint8_t slave, char *key) {

	attr f = get_field(key, fields);

	uint32_t data;
	tmc_reg_read(dev, slave, f.reg, &data);

	data &= f.mask;
	data |= data >> f.shift;

	return (int32_t) data;
}
void tmc_set(const struct device *dev, uint8_t slave, char *key, int32_t value) {

	attr f = get_field(key, fields);
	uint32_t data;

	tmc_reg_read(dev, slave, f.reg, &data);
	data &= ~f.mask;
	data |= value << f.shift;
	tmc_reg_write(dev, slave, f.reg, data);

}

// general
void tmc_set_irun_ihold(const struct device *dev, uint8_t slave, uint8_t irun, uint8_t ihold) {

	uint32_t data = (
				( (irun << TMC5160_IRUN_SHIFT)&TMC5160_IRUN_MASK) 	|
				( (ihold << TMC5160_IHOLD_SHIFT)&TMC5160_IHOLD_MASK ) |
				( (6<<TMC5160_IHOLDDELAY_SHIFT)&TMC5160_IHOLDDELAY_MASK) );
	tmc_reg_write(dev, slave, TMC5160_IHOLD_IRUN, 	data);

}

void tmc_set_mode(const struct device *dev, uint8_t slave, uint8_t mode) {
	tmc_reg_write(dev, slave, TMC5160_RAMPMODE,	mode);
}

// ramp
int32_t tmc_get_xtarget(const struct device *dev, uint8_t slave) {

	int32_t data;
	// TODO: add error chaecks/messages
	tmc_reg_read(dev, slave, TMC5160_XTARGET, &data);
	return data;

}
void tmc_set_xtarget(const struct device *dev, uint8_t slave, int32_t target) {
	tmc_reg_write(dev, slave, TMC5160_XTARGET,	target);
}
void tmc_set_vstart(const struct device *dev, uint8_t slave, uint32_t vstart) {
	tmc_reg_write(dev, slave, TMC5160_VSTART, vstart);
}
void tmc_set_a1(const struct device *dev, uint8_t slave, uint32_t a1) {
	tmc_reg_write(dev, slave, TMC5160_A1, a1);
}
void tmc_set_amax(const struct device *dev, uint8_t slave, uint32_t amax) {
	tmc_reg_write(dev, slave, TMC5160_AMAX, amax);
}
void tmc_set_v1(const struct device *dev, uint8_t slave, uint32_t v1) {
	tmc_reg_write(dev, slave, TMC5160_V1, v1);
}
void tmc_set_vmax(const struct device *dev, uint8_t slave, uint32_t vmax) {
	tmc_reg_write(dev, slave, TMC5160_VMAX, vmax);
}
void tmc_set_dmax(const struct device *dev, uint8_t slave, uint32_t dmax) {
	tmc_reg_write(dev, slave, TMC5160_DMAX, dmax);
}
void tmc_set_d1(const struct device *dev, uint8_t slave, uint32_t d1) {
	tmc_reg_write(dev, slave, TMC5160_D1, d1);
}
void tmc_set_vstop(const struct device *dev, uint8_t slave, uint32_t vstop) {
	tmc_reg_write(dev, slave, TMC5160_VSTOP, vstop);
}



// Sensor API
static int tmc5160_sample_fetch(const struct device *dev, enum sensor_channel chan) {
	//struct bme280_data *data = dev->data;
	//uint8_t buf[8];

	return 0;
}
static int tmc5160_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	//struct tmc5160_data *data = dev->data;

	switch (chan) {
	default:
		return -EINVAL;
	}

	return 0;
}
static const struct sensor_driver_api tmc5160_api = {
	.sample_fetch = tmc5160_sample_fetch,
	.channel_get = tmc5160_channel_get,
};


// TODO:
//.diag0_pin = GPIO_DT_SPEC_GET_OR(diag0_pin, 0),
//.diag1_pin = GPIO_DT_SPEC_GET_OR(diag1_pin, 0),
//.cb_dma = uart_cb_dma,

#define TMC5160_DEFINE(inst)							\
	static struct tmc_data tmc_data_##inst = {			\
		.r_sens = DT_INST_PROP(inst, r_sens),			\
		.i_run = DT_INST_PROP(inst, current_run),		\
		.i_hold = DT_INST_PROP(inst, current_hold),		\
	};													\
														\
	static const struct tmc_config tmc_config_##inst = {			\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),						\
			    (\
			     .spi = SPI_DT_SPEC_INST_GET(inst, 					\
				 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0), \
				),													\
			    ())													\
		COND_CODE_1(DT_INST_ON_BUS(inst, uart),						\
			    (													\
			     .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)), 		\
				 .cb = tmc_uart_cb_dma,									\
				),													\
			    ())													\
		.rotation_distance = DT_INST_PROP(inst, rotation_distance), \
	};						\
							\
	DEVICE_DT_INST_DEFINE(inst, 						\
		tmc5160_init, 									\
		NULL, 											\
		&tmc_data_##inst, 								\
		&tmc_config_##inst, 							\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, 		\
		&tmc5160_api);									\

DT_INST_FOREACH_STATUS_OKAY(TMC5160_DEFINE)
