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
#include "tmc2130.h"
#include "tmc_spi.h"

// piggyback on SPI log level for now
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TMC2130, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT trinamic_tmc2130

extern struct field fields[];
extern struct reg regs[];


// TODO: replace with system utils
uint32_t assemble_32(uint8_t *p_data) {
	int i;
	uint32_t result = p_data[0];
	for (i = 1; i < 4; i++)
		result = (result << 8) + p_data[i];
	return result;
}

static int tmc2130_init(const struct device *dev)
{
	int res = 0;

#if CONFIG_TMC_SPI
	const struct tmc_config *cfg = dev->config;
	if (!spi_is_ready(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
#endif

	LOG_INF("tmc2130_init done");

	return res;
}

// TMC r/w registers
uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data) {

	int res = 0;

#if CONFIG_TMC_SPI
	const struct tmc_config *cfg = dev->config;

	uint8_t buf[5] = {0};
	spi_read_register( &(cfg->spi), reg, buf );
	// TODO: replace with sys_to.... from byteorder.h
	//sys_be32_to_cpu();
	*data = assemble_32(&buf[1]);
#else
	LOG_INF("TMC - Read register via SPI not enabled");
#endif

	return res;
}

uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value) {

	int res = 0;

#if CONFIG_TMC_SPI
	const struct tmc_config *cfg = dev->config;
	spi_write_register( &(cfg->spi), reg, value);
#else
	LOG_INF("TMC - Write register via SPI not enabled");
#endif

	return 0;
}

int tmc_init(const struct device *dev, uint8_t slave) {

	const struct tmc_config *cfg = dev->config;

	tmc_reg_write(dev, slave, TMC2130_GSTAT, 		0x7			); // clear errors
	//tmc_reg_write(dev, slave, TMC5160_CHOPCONF, 	0x000100C3	); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)

	// write only
	//tmc_set_irun_ihold(dev, slave, cfg->run_current, cfg->hold_current);

	//tmc_reg_write(dev, slave, TMC5160_TPOWERDOWN, 	0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
	//tmc_reg_write(dev, slave, TMC5160_GCONF, 		0x00000004); // EN_PWM_MODE=1 enables StealthChop (with default PWM_CONF)
	//tmc_reg_write(dev, slave, TMC5160_TPWMTHRS, 	0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM

	// TODO: add init from DS
	// see DS pag. ???

	return 0;

}
void tmc_run(const struct device *dev, uint8_t slave, int32_t speed, int32_t acc) {

	//const struct tmc5160_config *cfg = dev->config;
	//printk( "rot_dist	: %f \n", cfg->rotation_distance);
	//printk( "speed conv	: %f, %f,  %f \n", (float)speed, RPM_TO_PPS, (float)speed * RPM_TO_PPS / TMC_T);

	// AMAX acceleration and deceleration value in velocity mode (DS pag. 40)
	//if(acc!=0)
	//	tmc_reg_write(dev, slave, TMC5160_AMAX, (uint32_t)acc );

	// VMAX velocity value in velocity mode (DS pag. 40)
	//tmc_reg_write(dev, slave, TMC5160_VMAX, (uint32_t)((float)speed * RPM_TO_PPS / TMC_T) );

	// direction
	if(speed>0) {
		// + velocity mode
		//tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 1);
	} else if(speed<0) {
		// - velocity mode
		//tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 2);
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
				( (irun << TMC2130_IRUN_SHIFT)&TMC2130_IRUN_MASK) 	|
				( (ihold << TMC2130_IHOLD_SHIFT)&TMC2130_IHOLD_MASK ) |
				( (6<<TMC2130_IHOLDDELAY_SHIFT)&TMC2130_IHOLDDELAY_MASK) );
	tmc_reg_write(dev, slave, TMC2130_IHOLD_IRUN, 	data);

}

// Sensor API
static int tmc2130_sample_fetch(const struct device *dev, enum sensor_channel chan) {
	//struct bme280_data *data = dev->data;
	//uint8_t buf[8];
	return 0;
}
static int tmc2130_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	//struct tmc5160_data *data = dev->data;

	switch (chan) {
	default:
		return -EINVAL;
	}

	return 0;
}
static const struct sensor_driver_api tmc2130_api = {
	.sample_fetch = tmc2130_sample_fetch,
	.channel_get = tmc2130_channel_get,
};

// TODO:
// .diag0_pin = GPIO_DT_SPEC_GET_OR(diag0_pin, 0),
// .bus_init = tmc_spi_init,

#define TMC2130_DEFINE(inst)							\
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
				),			\
			    ())			\
		.rotation_distance = DT_INST_PROP(inst, rotation_distance), 	\
	};						\
							\
	DEVICE_DT_INST_DEFINE(inst, 						\
		tmc2130_init, 									\
		NULL, 											\
		&tmc_data_##inst, 								\
		&tmc_config_##inst, 							\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, 		\
		&tmc2130_api);									\

DT_INST_FOREACH_STATUS_OKAY(TMC2130_DEFINE)
