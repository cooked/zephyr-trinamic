/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
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

#include <zephyr/drivers/uart.h>

#include "tmc.h"		// tmc_data, tmc_config
#include "tmc_reg.h"
#include "tmc2209.h"
#include "tmc2209_fields.h"

#include "tmc_uart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(TMC2209, CONFIG_SENSOR_LOG_LEVEL);


#define DT_DRV_COMPAT trinamic_tmc2209


// map fields
struct field fields[] = {
	// 0x00 GCONF
	{ "shaft", 		{TMC_GCONF, 		TMC2209_SHAFT_MASK, 	TMC2209_SHAFT_SHIFT}},
	// TODO: complete above
	// 0x01 GSTAT
	{ "reset", 		{TMC_GSTAT, 		TMC2209_RESET_MASK, 	TMC2209_RESET_SHIFT}},
	{ "drverr", 	{TMC_GSTAT, 		TMC2209_DRV_ERR_MASK, 	TMC2209_DRV_ERR_SHIFT}},
	{ "uvcp", 		{TMC_GSTAT, 		TMC2209_UV_CP_MASK, 	TMC2209_UV_CP_SHIFT}},
	// 0x02 IFNCT
	{ "ifcnt", 		{TMC_IFCNT, 		TMC2209_IFCNT_MASK, 	TMC2209_IFCNT_SHIFT}},
	// 0x03 SLAVECONF
	//{ "slaveaddr", 	{TMC_SLAVECONF, 	TMC2209_SLAVEADDR_MASK, TMC2209_SLAVEADDR_SHIFT}},
	{ "senddelay", 	{TMC_SLAVECONF, 	TMC2209_SLAVECONF_MASK, TMC2209_SLAVECONF_SHIFT}},
	//...
	{ "version", 	{TMC_INP_OUT, 		TMC2209_VERSION_MASK, 				TMC2209_VERSION_SHIFT}},
	//{ "uart", 		{TMC_INP_OUT, 		TMC2209_OUTPUT_PIN_POLARITY_MASK, 	TMC2209_OUTPUT_PIN_POLARITY_SHIFT}},

	// 0x10 IHOLD_IRUN
	{ "ihold", 		{TMC_IHOLD_IRUN, 	TMC2209_IHOLD_MASK, 		TMC2209_IHOLD_SHIFT}},
    { "irun",  		{TMC_IHOLD_IRUN, 	TMC2209_IRUN_MASK, 			TMC2209_IRUN_SHIFT}},
	{ "iholddelay", {TMC_IHOLD_IRUN, 	TMC2209_IHOLDDELAY_MASK, 	TMC2209_IHOLDDELAY_SHIFT}},
	// 0x12 TSTEP
	{ "tstep", 		{TMC_TSTEP, 		TMC2209_TSTEP_MASK, 	TMC2209_TSTEP_SHIFT}},
	// 0x20 RAMPMODE
	{ "vactual", 	{TMC_VACTUAL, 		TMC2209_VACTUAL_MASK, 	TMC2209_VACTUAL_SHIFT}},

	// TODO: complete above

	// driver status
	{ "stst",		{TMC_DRVSTATUS, 	TMC2209_STST_MASK, 			TMC2209_STST_SHIFT}},
	{ "olb",		{TMC_DRVSTATUS, 	TMC2209_OLB_MASK, 			TMC2209_OLB_SHIFT}},
	{ "ola",		{TMC_DRVSTATUS, 	TMC2209_OLA_MASK, 			TMC2209_OLA_SHIFT}},
	{ "s2gb",		{TMC_DRVSTATUS, 	TMC2209_S2GB_MASK, 			TMC2209_S2GB_SHIFT}},
	{ "s2ga",		{TMC_DRVSTATUS, 	TMC2209_S2GA_MASK, 			TMC2209_S2GA_SHIFT}},
	{ "otpw",		{TMC_DRVSTATUS, 	TMC2209_OTPW_MASK, 			TMC2209_OTPW_SHIFT}},
	{ "ot",			{TMC_DRVSTATUS, 	TMC2209_OT_MASK, 			TMC2209_OT_SHIFT}},

	{ "csactual",	{TMC_DRVSTATUS, 	TMC2209_CS_ACTUAL_MASK, 	TMC2209_CS_ACTUAL_SHIFT}},

	{ "stealth",	{TMC_DRVSTATUS, 	TMC2209_STEALTH_MASK, 		TMC2209_STEALTH_SHIFT}},
	{ "s2vsb",		{TMC_DRVSTATUS, 	TMC2209_S2VSB_MASK, 		TMC2209_S2VSB_SHIFT}},
	{ "s2vsa",		{TMC_DRVSTATUS, 	TMC2209_S2VSA_MASK, 		TMC2209_S2VSA_SHIFT}},

};

char *fields_gstat[3] = {"reset", "drv_err", "uv_cp"};

struct reg regs[] = {
	{ TMC_GCONF, 		"rw"	},
	{ TMC_GSTAT, 		"rwc"   },
	{ TMC_IFCNT, 		"r"		},
	{ TMC_INP_OUT, 		"r"		},
	{ TMC_SLAVECONF, 	"w"		},
    { TMC_IHOLD_IRUN, 	"w"		},

	{ TMC_SWMODE, 		"rw"	},
	{ TMC_RAMPSTAT, 	"rw"	},
};

attr get_field(char *key) {
    int i = 0;
    char *name = fields[i].name;
    while (name) {
        if (strcmp(name, key) == 0)
            return fields[i].a;
        name = fields[++i].name;
    }

	// not found
	attr na = {0,0,0};
    return na;
}

uint32_t assemble_32(uint8_t *p_data) {
	int i;
	uint32_t result = p_data[0];
	for (i = 1; i < 4; i++)
		result = (result << 8) + p_data[i];
	return result;
}



static int tmc2209_init(const struct device *dev) {

	//struct tmc_data *data = dev->data;
	const struct tmc_config *cfg = dev->config;

//#if CONFIG_TMC_UART
	uart_rx_disable(cfg->uart);
	uart_callback_set(cfg->uart, cfg->cb_dma, (void *)dev);
	// TODO: here we should initialize all slave... maybe with the addressing first
	//tmc_init(dev, 0);
//#endif

	LOG_INF("tmc2209_init done");

	return 0;
}

// TMC r/w registers
uint8_t tmc_reg_read(const struct device *dev, uint8_t reg, uint32_t *data) {

	const struct tmc_config *cfg = dev->config;

	uint8_t buf[8] = {0};
	uart_read_register(cfg->uart, cfg->slave, reg, buf);
	*data = assemble_32(&buf[3]);

	return 0;
}

uint8_t tmc_reg_write(const struct device *dev, uint8_t reg, uint32_t value) {

	const struct tmc_config *cfg = dev->config;

	uart_write_register( cfg->uart, cfg->slave, reg, value );

	return 0;
}


int tmc_init(const struct device *dev) {

	const struct tmc_config *cfg = dev->config;

	tmc_reg_write(dev, TMC_GSTAT, 		0x7			); // clear errors
	tmc_reg_write(dev, TMC_CHOPCONF, 	0x000100C3	); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)

	// write only
	tmc_set_irun_ihold(dev, cfg->run_current, cfg->hold_current);

	tmc_reg_write(dev, TMC_TPOWERDOWN, 	0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
	tmc_reg_write(dev, TMC_GCONF, 		0x00000004); // EN_PWM_MODE=1 enables StealthChop (with default PWM_CONF)
	tmc_reg_write(dev, TMC_TPWMTHRS, 	0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM

	return 0;

}
void tmc_run(const struct device *dev, int32_t speed, int32_t acc) {

	//const struct tmc5160_config *cfg = dev->config;
	//printk( "rot_dist	: %f \n", cfg->rotation_distance);
	//printk( "speed conv	: %f, %f,  %f \n", (float)speed, RPM_TO_PPS, (float)speed * RPM_TO_PPS / TMC_T);

	// AMAX acceleration and deceleration value in velocity mode (DS pag. 40)
	//if(acc!=0)
	//	tmc_reg_write(dev, slave, TMC5160_AMAX, (uint32_t)acc );

	// VMAX velocity value in velocity mode (DS pag. 40)
	//tmc_reg_write(dev, slave, TMC5160_VMAX, (uint32_t)((float)speed * RPM_TO_PPS / TMC_T) );

	// direction
	/*if(speed>0) {
		// + velocity mode
		tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 1);
	} else if(speed<0) {
		// - velocity mode
		tmc_reg_write(dev, slave, TMC5160_RAMPMODE, 2);
	} else {
		// keep existing ramp_mode, but stop motion (speed=0)
	}*/
}

int tmc_dump(const struct device *dev) {

	uint32_t data;

	size_t nreg = sizeof(regs)/sizeof(regs[0]);

	for(uint8_t reg = 0; reg < nreg; reg++) {
		if(	strchr( regs[reg].rwc, 'r') ) {
			tmc_reg_read( dev, regs[reg].reg, &data);
			printk(" - Register 0x%02X, value: 0x%08X \n", regs[reg].reg, data);
		}
	}

	return 0;
}

// tmc register's field getter/setter
int32_t tmc_get(const struct device *dev, char *key) {

	attr f = get_field(key);

	uint32_t data;
	tmc_reg_read(dev, f.reg, &data);

	data &= f.mask;
	data |= data >> f.shift;

	return (int32_t) data;
}
void tmc_set(const struct device *dev, char *key, int32_t value) {

	attr f = get_field(key);
	uint32_t data;

	tmc_reg_read(dev, f.reg, &data);
	data &= ~f.mask;
	data |= value << f.shift;
	tmc_reg_write(dev, f.reg, data);

}

// general
void tmc_set_irun_ihold(const struct device *dev, uint8_t irun, uint8_t ihold) {

	uint32_t data = (
				( (irun << TMC2209_IRUN_SHIFT)&TMC2209_IRUN_MASK) 	|
				( (ihold << TMC2209_IHOLD_SHIFT)&TMC2209_IHOLD_MASK ) |
				( (6<<TMC2209_IHOLDDELAY_SHIFT)&TMC2209_IHOLDDELAY_MASK) );
	tmc_reg_write(dev, TMC_IHOLD_IRUN, 	data);
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


//.diag0_pin = GPIO_DT_SPEC_GET_OR(diag0_pin, 0),
//.diag1_pin = GPIO_DT_SPEC_GET_OR(diag1_pin, 0),
//.rotation_distance = DT_INST_PROP(0, rotation_distance),
//.rpm_to_hzs = ( 1 / DEFAULT_STEPS_TURN / TMC_T )

#define TMC2209_DEVICE(inst)                                       	\
    static struct tmc_data tmc2209_data_##inst = {				\
		.r_sens = DT_INST_PROP(inst, r_sens),						\
		.i_run = DT_INST_PROP(inst, current_run),					\
		.i_hold = DT_INST_PROP(inst, current_hold),					\
	};                                                             	\
    static const struct tmc_config tmc2209_cfg_##inst = {				\
		.uart = DEVICE_DT_GET(DT_INST_BUS(0)),						\
		.slave = DT_INST_PROP(inst, slaveaddr),						\
		.cb_dma = uart_cb_dma,										\
		.cb = uart_cb,												\
	};                                                             	\
    DEVICE_DT_INST_DEFINE(											\
		inst,                                     					\
		tmc2209_init,                     							\
		NULL,                                     					\
		&tmc2209_data_##inst,                          				\
		&tmc2209_cfg_##inst,                           				\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,  					\
		&tmc2209_api);

DT_INST_FOREACH_STATUS_OKAY(TMC2209_DEVICE)