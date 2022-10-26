/*
 * Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <shell/shell.h>

#include "tmc5160.h"

#define CMD_ARG_N 1


extern bool toggle;
extern const struct device *tmc;

extern struct field fields[];

uint8_t slave = 0;

static int cmd_tmc(const struct shell *shell, size_t argc, char *argv[])
{
	slave = (uint8_t) strtol(argv[1], &argv[1], 10);
	return 0;
}
static int cmd_tmc_get(const struct shell *shell, size_t argc, char *argv[])
{
	uint32_t data;

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);
	attr f = get_field(argv[CMD_ARG_N+1]);

	tmc_reg_read(tmc, slave, f.reg, &data);

	shell_fprintf(shell, SHELL_NORMAL, "GET - Field '%s' (reg 0x%02X) has value: %u \n", argv[CMD_ARG_N+1], f.reg, (data&f.mask)>>f.shift);

	//shell_fprintf(shell, SHELL_NORMAL, "GET - Field '%s' (reg 0x%02X) has value: %u \n", argv[CMD_ARG_N+1], f.reg, (data&f.mask)>>f.shift);

    return 0;
}
static int cmd_tmc_set(const struct shell *shell, size_t argc, char *argv[])
{
	uint32_t data;

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);
	attr f = get_field(argv[CMD_ARG_N+1]);

	int32_t val = strtol(argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);

	/*tmc_reg_read(tmc, slave, f.reg, &data);
	data &= ~f.mask;
	data |= val<<f.shift;
	tmc_reg_write(tmc, slave, f.reg, data);
	*/

	tmc_reg_write(tmc, slave, f.reg, val);

	shell_fprintf(shell, SHELL_NORMAL,
		"SET - Field '%s' (reg 0x%02X) set to value 0x%04X \n", argv[CMD_ARG_N+1], f.reg, val);


    return 0;
}
static int cmd_tmc_dump(const struct shell *shell, size_t argc, char *argv[])
{

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);
    shell_fprintf(shell, SHELL_NORMAL, "TMC dump slave %u registers: \n", slave);

	tmc_dump(tmc, slave);

	return 0;
}
static int cmd_tmc_init(const struct shell *shell, size_t argc, char *argv[])
{
    slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	shell_fprintf(shell, SHELL_NORMAL, "TMC init slave %u in POS mode \n", slave);

	tmc_init(tmc, slave);

    return 0;
}

static int cmd_tmc_run(const struct shell *shell, size_t argc, char *argv[])
{
	// TODO: add a second parameter for acceleration

	int32_t rpm = (int32_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);
	int32_t acc = 0;

	if(argc == CMD_ARG_N+2)
		acc = (int32_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	tmc_run(tmc, slave, rpm, acc);

	shell_fprintf(shell, SHELL_NORMAL, "Run motor at %d rpm\n", rpm);

    return 0;
}
static int cmd_tmc_mode(const struct shell *shell, size_t argc, char *argv[])
{
    shell_fprintf(shell, SHELL_NORMAL, "TMC query <field_name>: %d\n", 0);
    return 0;
}
static int cmd_tmc_turn(const struct shell *shell, size_t argc, char *argv[])
{

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	// increment/decrement current position by n (float) turns
	int32_t acc, speed;

	if(argc == CMD_ARG_N+3) {
		acc = (int32_t) strtol( argv[CMD_ARG_N+3], &argv[CMD_ARG_N+3], 10);
		tmc_set_amax(tmc, slave, acc);
		speed = (int32_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);
		tmc_set_vmax(tmc, slave, speed);
	} else if(argc == CMD_ARG_N+2) {
		speed = (int32_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);
		tmc_set_vmax(tmc, slave, speed);
	}

	//float turns = strtod(argv[1], &argv[1]);
	int32_t turns = (int32_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);

	int32_t steps = tmc_get_xtarget(tmc, slave);
	steps += (int32_t) (turns * DEFAULT_STEPS_TURN);

	tmc_set_xtarget(tmc, slave, 51200);

	return 0;
}

static int cmd_tmc_goto(const struct shell *shell, size_t argc, char *argv[])
{
	// rotate to absolute position [turns]

	//float turns = strtod(argv[1], &argv[1]);
	int32_t turns = (int32_t) strtol( argv[1], &argv[1], 10);

	int32_t steps = (int32_t) (turns * DEFAULT_STEPS_TURN);
	// TODO:
	//tmc_set_xtarget(tmc, slave, steps);

	return 0;
}
static int cmd_tmc_cur(const struct shell *shell, size_t argc, char *argv[])
{

	uint8_t irun = (uint8_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);
	uint8_t ihold = (uint8_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	tmc_set_irun_ihold(tmc, slave, irun, ihold);

	shell_fprintf(shell, SHELL_NORMAL, "Set TMC current I_RUN: %u, I_HOLD: %u \n", irun, ihold);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE( tmc_cmds,

	SHELL_CMD_ARG(init, NULL, "Initialize slave in position mode",
		cmd_tmc_init, 2, 0),

	SHELL_CMD_ARG(dump, NULL, "Printout slave readable registers",
		cmd_tmc_dump, 2, 0),

	// run (start/stop)
	SHELL_CMD_ARG(run, NULL, "Start/Stop TMC stepper (at <rpm_speed>, optional accel)",
		cmd_tmc_run, 3, 1),

	// set mode (ramp or velocity)
	SHELL_CMD_ARG(mode, NULL, "Set TMC mode of operation. ($ tmc mode <slave> <0/1> // 0:velocity 1:ramp)",
		cmd_tmc_mode, 2, 0),

	// set/get register's field
	SHELL_CMD_ARG(get, NULL,	"Get TMC register's field (tmc get <slave> <field>)",
		cmd_tmc_get, 3, 0),
	SHELL_CMD_ARG(set, NULL,	"Set TMC register's field (tmc set <slave> <field> <value>)",
		cmd_tmc_set, 4, 0),


	SHELL_CMD_ARG(turn, NULL,	"Turn motor by n turns (1.5: increment, -1.5: decrement)",
		cmd_tmc_turn, 3, 2),
	SHELL_CMD_ARG(goto, NULL,	"Move TMC to position",
		cmd_tmc_goto, 2, 0),

	SHELL_CMD_ARG(cur, NULL, 	"Set TMC run and hold current (0:min, 31:max)",
		cmd_tmc_cur, 4, 0),

	SHELL_SUBCMD_SET_END
);

// [0] [1]         [2]
// tmc <slaveaddr> <subcmd>
SHELL_CMD_ARG_REGISTER(tmc, &tmc_cmds, "TMC driver control commands", NULL, 1, 0);