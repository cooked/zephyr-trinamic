/*
 * Copyright (c) 2022 Wrecklab BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// mind include path
// https://stackoverflow.com/questions/72294929/location-of-source-file-include-drivers-gpio-h

#define DT_DRV_COMPAT trinamic_tmc5160

#include <device.h>
#include <devicetree.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/byteorder.h>	// sys_to_xxx()
#include <init.h>

#include "tmc5160.h"
#include "tmc5160_reg.h"
#include "tmc5160_fields.h"

#ifdef TMC5160_SPI
#include "tmc5160_spi.h"
#else
#include "tmc5160_uart.h"
#endif

// piggyback on SPI log level for now
#include <logging/log.h>

LOG_MODULE_REGISTER(TMC5160, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT trinamic_tmc5160

// map fields
struct field fields[] = {
	// 0x00 GCONF
#ifdef TMC5130
	{ "iscaleanalog",	{TMC5160_GCONF, 		TMC5160_SHAFT_MASK, 	TMC5160_SHAFT_SHIFT}},
#elif TMC5160
	// TODO: change to "recalibrate"
	{ "iscaleanalog",	{TMC5160_GCONF, 		TMC5160_SHAFT_MASK, 	TMC5160_SHAFT_SHIFT}},
#endif
	{ "shaft", 			{TMC5160_GCONF, 		TMC5160_SHAFT_MASK, 	TMC5160_SHAFT_SHIFT}},
	// TODO: complete above
	// 0x01 GSTAT
	{ "reset", 		{TMC5160_GSTAT, 		TMC5160_RESET_MASK, 	TMC5160_RESET_SHIFT}},
	{ "drverr", 	{TMC5160_GSTAT, 		TMC5160_DRV_ERR_MASK, 	TMC5160_DRV_ERR_SHIFT}},
	{ "uvcp", 		{TMC5160_GSTAT, 		TMC5160_UV_CP_MASK, 	TMC5160_UV_CP_SHIFT}},
	// 0x02 IFNCT
	{ "ifcnt", 		{TMC5160_IFCNT, 		TMC5160_IFCNT_MASK, 	TMC5160_IFCNT_SHIFT}},
	// 0x03 SLAVECONF
	{ "slaveaddr", 	{TMC5160_SLAVECONF, 	TMC5160_SLAVEADDR_MASK, TMC5160_SLAVEADDR_SHIFT}},
	{ "senddelay", 	{TMC5160_SLAVECONF, 	TMC5160_SENDDELAY_MASK, TMC5160_SENDDELAY_SHIFT}},
	// 0x04 IOIN
	{ "reflstep", 	{TMC5160_INP_OUT, 		TMC5160_REFL_STEP_MASK, 		TMC5160_REFL_STEP_SHIFT}},
	{ "refrdir", 	{TMC5160_INP_OUT, 		TMC5160_REFR_DIR_MASK, 			TMC5160_REFR_DIR_SHIFT}},
	{ "encbdcen", 	{TMC5160_INP_OUT, 		TMC5160_ENCB_DCEN_CFG4_MASK, 	TMC5160_ENCB_DCEN_CFG4_SHIFT}},
	{ "encadcin", 	{TMC5160_INP_OUT, 		TMC5160_ENCA_DCIN_CFG5_MASK, 	TMC5160_ENCA_DCIN_CFG5_SHIFT}},
	//...
	{ "version", 	{TMC5160_INP_OUT, 		TMC5160_VERSION_MASK, 				TMC5160_VERSION_SHIFT}},
	{ "uart", 		{TMC5160_INP_OUT, 		TMC5160_OUTPUT_PIN_POLARITY_MASK, 	TMC5160_OUTPUT_PIN_POLARITY_SHIFT}},

	// 0x10 IHOLD_IRUN
	{ "ihold", 		{TMC5160_IHOLD_IRUN, 	TMC5160_IHOLD_MASK, 		TMC5160_IHOLD_SHIFT}},
    { "irun",  		{TMC5160_IHOLD_IRUN, 	TMC5160_IRUN_MASK, 			TMC5160_IRUN_SHIFT}},
	{ "iholddelay", {TMC5160_IHOLD_IRUN, 	TMC5160_IHOLDDELAY_MASK, 	TMC5160_IHOLDDELAY_SHIFT}},
	// 0x12 TSTEP
	{ "tstep", 		{TMC5160_TSTEP, 		TMC5160_TSTEP_MASK, 	TMC5160_TSTEP_SHIFT}},
	// 0x20 RAMPMODE
	{ "rampmode", 	{TMC5160_RAMPMODE, 		TMC5160_RAMPMODE_MASK, 	TMC5160_RAMPMODE_SHIFT}},
	{ "xactual", 	{TMC5160_XACTUAL, 		TMC5160_XACTUAL_MASK, 	TMC5160_XACTUAL_SHIFT}},
	{ "vactual", 	{TMC5160_VACTUAL, 		TMC5160_VACTUAL_MASK, 	TMC5160_VACTUAL_SHIFT}},
	{ "vstart", 	{TMC5160_VSTART, 		TMC5160_VSTART_MASK, 	TMC5160_VSTART_SHIFT}},
	{ "a1", 		{TMC5160_A1, 			TMC5160_A1_MASK, 		TMC5160_A1_SHIFT}},
	{ "v1", 		{TMC5160_V1, 			TMC5160_V1_MASK, 		TMC5160_V1_SHIFT}},
	{ "amax", 		{TMC5160_AMAX, 			TMC5160_AMAX_MASK, 		TMC5160_AMAX_SHIFT}},
	{ "vmax", 		{TMC5160_VMAX, 			TMC5160_VMAX_MASK, 		TMC5160_VMAX_SHIFT}},
	{ "dmax", 		{TMC5160_DMAX, 			TMC5160_DMAX_MASK, 		TMC5160_DMAX_SHIFT}},
	{ "d1", 		{TMC5160_D1, 			TMC5160_D1_MASK, 		TMC5160_D1_SHIFT}},
	{ "vstop", 		{TMC5160_VSTOP, 		TMC5160_VSTOP_MASK, 	TMC5160_VSTOP_SHIFT}},
	{ "tzerowait", 	{TMC5160_TZEROWAIT, 	TMC5160_TZEROWAIT_MASK, TMC5160_TZEROWAIT_SHIFT}},
	{ "xtarget", 	{TMC5160_XTARGET, 		TMC5160_XTARGET_MASK, 	TMC5160_XTARGET_SHIFT}},
	// switch mode config
	{ "en_softstop",	{TMC5160_SWMODE, 	TMC5160_EN_SOFTSTOP_MASK, 	TMC5160_EN_SOFTSTOP_SHIFT}},
	{ "sg_stop",		{TMC5160_SWMODE, 	TMC5160_SG_STOP_MASK, 		TMC5160_SG_STOP_SHIFT}},
	// TODO: complete above

	// 0x35 RAMP_STAT
	{ "statussg",	{TMC5160_RAMPSTAT, 		TMC5160_STATUS_SG_MASK, 	TMC5160_STATUS_SG_SHIFT}},

	// driver status
	{ "stst",		{TMC5160_DRVSTATUS, 	TMC5160_STST_MASK, 			TMC5160_STST_SHIFT}},
	{ "olb",		{TMC5160_DRVSTATUS, 	TMC5160_OLB_MASK, 			TMC5160_OLB_SHIFT}},
	{ "ola",		{TMC5160_DRVSTATUS, 	TMC5160_OLA_MASK, 			TMC5160_OLA_SHIFT}},
	{ "s2gb",		{TMC5160_DRVSTATUS, 	TMC5160_S2GB_MASK, 			TMC5160_S2GB_SHIFT}},
	{ "s2ga",		{TMC5160_DRVSTATUS, 	TMC5160_S2GA_MASK, 			TMC5160_S2GA_SHIFT}},
	{ "otpw",		{TMC5160_DRVSTATUS, 	TMC5160_OTPW_MASK, 			TMC5160_OTPW_SHIFT}},
	{ "ot",			{TMC5160_DRVSTATUS, 	TMC5160_OT_MASK, 			TMC5160_OT_SHIFT}},
	{ "stallguard",	{TMC5160_DRVSTATUS, 	TMC5160_STALLGUARD_MASK, 	TMC5160_STALLGUARD_SHIFT}},
	{ "csactual",	{TMC5160_DRVSTATUS, 	TMC5160_CS_ACTUAL_MASK, 	TMC5160_CS_ACTUAL_SHIFT}},
	{ "fsactive",	{TMC5160_DRVSTATUS, 	TMC5160_FSACTIVE_MASK, 		TMC5160_FSACTIVE_SHIFT}},
	{ "stealth",	{TMC5160_DRVSTATUS, 	TMC5160_STEALTH_MASK, 		TMC5160_STEALTH_SHIFT}},
	{ "s2vsb",		{TMC5160_DRVSTATUS, 	TMC5160_S2VSB_MASK, 		TMC5160_S2VSB_SHIFT}},
	{ "s2vsa",		{TMC5160_DRVSTATUS, 	TMC5160_S2VSA_MASK, 		TMC5160_S2VSA_SHIFT}},
	{ "sgresult",	{TMC5160_DRVSTATUS, 	TMC5160_SG_RESULT_MASK, 	TMC5160_SG_RESULT_SHIFT}},
};

char *fields_gstat[3] = {"reset", "drv_err", "uv_cp"};

struct reg regs[] = {
	{ TMC5160_GCONF, 		"rw"	},
	{ TMC5160_GSTAT, 		"rwc"   },
	{ TMC5160_IFCNT, 		"r"		},
	{ TMC5160_INP_OUT, 		"r"		},
	{ TMC5160_SLAVECONF, 	"w"		},
    { TMC5160_IHOLD_IRUN, 	"w"		},
	// ramp
	{ TMC5160_RAMPMODE, 	"rw"	},
	{ TMC5160_XACTUAL, 		"rw"	},
	{ TMC5160_VACTUAL, 		"r"		},
	{ TMC5160_VSTART, 		"w"		},
	{ TMC5160_A1, 			"w"		},
	{ TMC5160_V1, 			"w"		},
	{ TMC5160_AMAX, 		"w"		},
	{ TMC5160_VMAX, 		"w"		},
	{ TMC5160_DMAX, 		"w"		},
	{ TMC5160_D1, 			"w"		},
	{ TMC5160_VSTOP, 		"w"		},
	{ TMC5160_TZEROWAIT, 	"w"		},
	{ TMC5160_XTARGET, 		"rw"	},

	{ TMC5160_SWMODE, 		"rw"	},
	{ TMC5160_RAMPSTAT, 	"rw"	},
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

static int tmc5160_init(const struct device *dev)
{
	const struct tmc5160_config *cfg = dev->config;

	int res = 0;

#if TMC5160_SPI
	if (!spi_is_ready(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
#else

	uart_rx_disable(cfg->uart);
	uart_callback_set(cfg->uart, cfg->cb_dma, (void *)dev);
	// TODO: here we should initialize all slave... maybe with the addressing first

	//tmc_init(dev, 0);

#endif

	LOG_INF("tmc5160_init done");

	return res;
}

// TMC r/w registers
uint8_t tmc_reg_read(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t *data) {

	const struct tmc5160_config *cfg = dev->config;

#if TMC5160_SPI
	uint8_t buf[5] = {0};
	spi_read_register( &(cfg->spi), reg, buf );
	// TODO: replace with sys_to.... from byteorder.h
	//sys_be32_to_cpu();
	*data = assemble_32(&buf[1]);
#else
	uint8_t buf[8] = {0};
	uart_read_register(cfg->uart, slave, reg, buf);
	*data = assemble_32(&buf[3]);
#endif

	return 0;
}

uint8_t tmc_reg_write(const struct device *dev, uint8_t slave, uint8_t reg, uint32_t value) {

	const struct tmc5160_config *cfg = dev->config;

#ifdef TMC5160_SPI
	spi_write_register( &(cfg->spi), reg, value);
#else
	uart_write_register( cfg->uart, slave, reg, value );
#endif

	return 0;
}

int tmc_init(const struct device *dev, uint8_t slave) {

	const struct tmc5160_config *cfg = dev->config;

	tmc_reg_write(dev, slave, TMC5160_GSTAT, 		0x7			); // clear errors
	tmc_reg_write(dev, slave, TMC5160_CHOPCONF, 	0x000100C3	); // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)

	// write only
	tmc_set_irun_ihold(dev, slave, cfg->run_current, cfg->hold_current);

	tmc_reg_write(dev, slave, TMC5160_TPOWERDOWN, 	0x0000000A); // TPOWERDOWN=10: Delay before power down in stand still
	tmc_reg_write(dev, slave, TMC5160_GCONF, 		0x00000004); // EN_PWM_MODE=1 enables StealthChop (with default PWM_CONF)
	tmc_reg_write(dev, slave, TMC5160_TPWMTHRS, 	0x000001F4); // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM

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

/*void tmc_pos(const struct device *dev, int32_t pos, int32_t acc) {

	// AMAX acceleration and deceleration value in velocity mode (DS pag. 40)
	if(acc!=0)
		tmc_reg_write(dev, TMC5160_AMAX, (uint32_t)acc );

	// VMAX velocity value in velocity mode (DS pag. 40)
	tmc_reg_write(dev, TMC5160_VMAX, (uint32_t)((float)speed * RPM_TO_PPS / TMC_T) );

	// position mode
	tmc_reg_write(dev, TMC5160_RAMPMODE, 0);
}*/

int tmc_dump(const struct device *dev, uint8_t slave) {

	uint32_t data;

	size_t nreg = sizeof(regs)/sizeof(regs[0]);

	for(uint8_t reg = 0; reg < nreg; reg++) {
		if(	strchr( regs[reg].rwc, 'r') ) {
			tmc_reg_read( dev, slave, regs[reg].reg, &data);
			printk(" - Register 0x%02X, value: 0x%08X \n", regs[reg].reg, data);
		}
	}

	return 0;
}

// tmc register's field getter/setter
int32_t tmc_get(const struct device *dev, uint8_t slave, char *key) {

	attr f = get_field(key);

	uint32_t data;
	tmc_reg_read(dev, slave, f.reg, &data);

	data &= f.mask;
	data |= data >> f.shift;

	return (int32_t) data;
}
void tmc_set(const struct device *dev, uint8_t slave, char *key, int32_t value) {

	attr f = get_field(key);
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
static int tmc5160_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
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

/* device defaults to spi mode 0/3 support */
#define TMC5160_SPI_CFG \
			(SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB \
			| SPI_MODE_CPOL | SPI_MODE_CPHA)

static struct tmc5160_config tmc5160_cfg_0 = {
#ifdef TMC5160_SPI
	.spi = SPI_DT_SPEC_INST_GET(0, TMC5160_SPI_CFG, 0),
#else
	.uart = DEVICE_DT_GET(DT_INST_BUS(0)),
	.cb = uart_cb,
	.cb_dma = uart_cb_dma,
#endif
	//.diag0_pin = GPIO_DT_SPEC_GET_OR(diag0_pin, 0),
	//.diag1_pin = GPIO_DT_SPEC_GET_OR(diag1_pin, 0),
	.rotation_distance = DT_INST_PROP(0, rotation_distance),
	//.rpm_to_hzs = ( 1 / DEFAULT_STEPS_TURN / TMC_T )
};

static struct tmc5160_data_t tmc5160_data = {
	.r_sens = DT_INST_PROP(0, r_sens),
	.i_run = DT_INST_PROP(0, current_run),
	.i_hold = DT_INST_PROP(0, current_hold),
};

DEVICE_DT_INST_DEFINE( \
	0, \
	tmc5160_init, \
	NULL, 	\
	&tmc5160_data, \
	&tmc5160_cfg_0, \
	POST_KERNEL, \
	CONFIG_SENSOR_INIT_PRIORITY, \
	&tmc5160_api);
