/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// mind include path
// https://stackoverflow.com/questions/72294929/location-of-source-file-include-drivers-gpio-h

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>	// sys_to_xxx()
#include <zephyr/init.h>

#include "tmc.h"		// tmc_data, tmc_config
#include "tmc2209.h"

#ifdef CONFIG_TMC_SD
#include <zephyr/drivers/pwm.h>
#endif
#ifdef CONFIG_TMC_UART
#include "tmc_uart.h"
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(TMC2209, CONFIG_SENSOR_LOG_LEVEL);


#define DT_DRV_COMPAT trinamic_tmc2209

extern struct field fields[];
extern struct reg regs[];


static int tmc2209_init(const struct device *dev) {

	//struct tmc_data *data = dev->data;
	const struct tmc_config *cfg = dev->config;

#if CONFIG_TMC_UART

	LOG_DBG("TMC UART poll-mode \n");
	//tmc_uart_init(dev);

#endif

	// disable stepper
	//gpio_pin_configure_dt(&cfg->en, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&cfg->en, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&cfg->dir, GPIO_OUTPUT_ACTIVE);

	LOG_DBG("tmc2209_init done");

	return 0;
}

// TMC r/w registers
uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data) {

	const struct tmc_config *cfg = dev->config;
#if CONFIG_TMC_UART
	uart_read_register(dev, cfg->slave, reg, data);
#endif
	return 0;
}

uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value) {

	const struct tmc_config *cfg = dev->config;
#if CONFIG_TMC_UART
	uart_write_register( dev, cfg->slave, reg, value );
#endif
	return 0;
}


int tmc_init(const struct device *dev, uint8_t slave) {

	const struct tmc_config *cfg = dev->config;

	tmc_reg_write(dev, slave, TMC2209_GSTAT, 		0x7			); // clear errors

	tmc_reg_write(dev, slave, TMC2209_CHOPCONF, 	0x000100C3	); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
	tmc_set_irun_ihold(dev, slave, cfg->run_current, cfg->hold_current); // write only

	tmc_reg_write(dev, slave, TMC2209_TPOWERDOWN, 	0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
	tmc_reg_write(dev, slave, TMC2209_GCONF, 		0x00000004); // EN_PWM_MODE=1 enables StealthChop (with default PWM_CONF)
	tmc_reg_write(dev, slave, TMC2209_TPWMTHRS, 	0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM

	return 0;

}

/*void tmc_enable() {

}*/

void tmc_run(const struct device *dev, uint8_t slave, int32_t speed, int32_t acc) {

#if CONFIG_TMC_SD

	struct tmc_config *cfg = dev->config;
	uint32_t period_ns = 1e9 / ( abs(speed) * RPM_TO_PPS);

	if(speed>0) {
		gpio_pin_set_dt(&cfg->dir,0);
		pwm_set_dt(&cfg->step, period_ns, (uint32_t)(period_ns/2));
	} else if(speed<0) {
		gpio_pin_set_dt(&cfg->dir,1);
		pwm_set_dt(&cfg->step, period_ns, (uint32_t)(period_ns/2));
	} else {
		pwm_set_pulse_dt(&cfg->step, 0);
	}

#else
	// TODO: implement writing to register VACTUAL

#endif

}

int tmc_dump(const struct device *dev, uint8_t slave) {

	uint32_t data;

	for(uint16_t reg = 0; reg < NREG; reg++) {
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
				( (irun << TMC2209_IRUN_SHIFT)&TMC2209_IRUN_MASK) 	|
				( (ihold << TMC2209_IHOLD_SHIFT)&TMC2209_IHOLD_MASK ) |
				( (6<<TMC2209_IHOLDDELAY_SHIFT)&TMC2209_IHOLDDELAY_MASK) );
	tmc_reg_write(dev, slave, TMC2209_IHOLD_IRUN, 	data);

}


// Sensor API
static int tmc2209_sample_fetch(const struct device *dev, enum sensor_channel chan) {
	//struct bme280_data *data = dev->data;
	//uint8_t buf[8];
	return 0;
}
static int tmc2209_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
	//struct tmc5160_data *data = dev->data;
	switch (chan) {
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct sensor_driver_api tmc2209_api = {
	.sample_fetch = tmc2209_sample_fetch,
	.channel_get = tmc2209_channel_get
};



#define TMC2209_DEFINE(inst)                                       	\
    static struct tmc_data tmc_data_##inst = {						\
		.r_sens = DT_INST_PROP(inst, r_sens),						\
		.i_run = DT_INST_PROP(inst, current_run),					\
		.i_hold = DT_INST_PROP(inst, current_hold),					\
	};                                                             	\
																	\
    static const struct tmc_config tmc_config_##inst = {			\
		COND_CODE_1(DT_INST_ON_BUS(inst, uart),						\
			    (													\
			     .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)), 		\
				 .cb = tmc_uart_cb_dma,								\
				),													\
			    ())													\
		.rotation_distance = DT_INST_PROP_OR(inst, rotation_distance, 0), \
		.slave = DT_INST_PROP(inst, slaveaddr), 					\
		.step = PWM_DT_SPEC_INST_GET_BY_IDX_OR(inst, 0, NULL),		\
		.dir = GPIO_DT_SPEC_INST_GET_OR(inst, dir_gpios, 0),		\
		.en = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, NULL),		\
		.diag0_pin = GPIO_DT_SPEC_INST_GET_OR(inst, diag0_pin, 0),	\
		.diag1_pin = GPIO_DT_SPEC_INST_GET_OR(inst, diag1_pin, 0),	\
	};																\
																	\
    DEVICE_DT_INST_DEFINE(											\
		inst,                                     					\
		tmc2209_init,                     							\
		NULL,                                     					\
		&tmc_data_##inst,                          					\
		&tmc_config_##inst,                           				\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  					\
		&tmc2209_api);

DT_INST_FOREACH_STATUS_OKAY(TMC2209_DEFINE)
