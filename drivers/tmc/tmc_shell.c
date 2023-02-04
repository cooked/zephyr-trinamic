/*
 * Copyright (c) 2022, Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <stdlib.h>
#include <zephyr/shell/shell.h>

//#include "tmc.h"
#include "tmc_shell.h"

extern bool toggle;
extern const struct device *tmc0;

//extern struct field fields[];

uint8_t slave = 0;

/*static int cmd_tmc_get(const struct shell *shell, size_t argc, char *argv[])
{

	if(argc!=4)
		shell_error(shell, "Wrong number of argument.");

	uint32_t data;

	char *fname = argv[3];
	attr f = get_field(fname, fields);

	//tmc_reg_read(tmc0, slave, f.reg, &data);

	shell_fprintf(shell, SHELL_NORMAL,
		"GET - Field '%s' (reg 0x%02X) value: %u \n", fname, f.reg, (data&f.mask)>>f.shift);

    return 0;
}
static int cmd_tmc_set(const struct shell *shell, size_t argc, char *argv[])
{
	if(argc!=5)
		shell_error(shell, "Wrong number of argument.");

	uint32_t data;

	attr f = get_field(argv[CMD_ARG_N+2], fields);

	// TODO: check if the register is writable

	int32_t val = strtol(argv[CMD_ARG_N+3], &argv[CMD_ARG_N+3], 10);

	//tmc_reg_read(tmc0, slave, f.reg, &data);
	data &= ~f.mask;
	data |= val<<f.shift;
	//tmc_reg_write(tmc0, slave, f.reg, data);

	//tmc_reg_write(tmc0, slave, f.reg, val);

	shell_fprintf(shell, SHELL_NORMAL,
		"SET - Field '%s' (reg 0x%02X) set to value 0x%04X \n", argv[CMD_ARG_N+1], f.reg, val);


    return 0;
}*/
static int cmd_tmc_dump(const struct shell *shell, size_t argc, char *argv[])
{

    shell_fprintf(shell, SHELL_NORMAL, "TMC dump slave %d registers: \n", slave);
	tmc_dump(tmc0, slave);

	return 0;
}
static int cmd_tmc_init(const struct shell *shell, size_t argc, char *argv[])
{
	shell_fprintf(shell, SHELL_NORMAL, "TMC init slave %u in POS mode \n", slave);

	//tmc_init(tmc0, slave);

    return 0;
}
static int cmd_tmc_run(const struct shell *shell, size_t argc, char *argv[])
{
	// TODO: add a second parameter for acceleration

	int32_t rpm = (int32_t) strtol( argv[3], &argv[3], 10);
	int32_t acc = 0;

	//if(argc == 4)
	//	acc = (int32_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);

	tmc_run(tmc0, slave, rpm, acc);

	shell_fprintf(shell, SHELL_NORMAL, "Run motor %d at %d rpm\n", slave, rpm);

    return 0;
}
static int cmd_tmc_cur(const struct shell *shell, size_t argc, char *argv[])
{

	uint8_t irun = (uint8_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);
	uint8_t ihold = (uint8_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	//tmc_set_irun_ihold(tmc0, slave, irun, ihold);

	shell_fprintf(shell, SHELL_NORMAL, "Set TMC current I_RUN: %u, I_HOLD: %u \n", irun, ihold);

	return 0;
}

int cmd_tmc(const struct shell *shell, size_t argc, char *argv[])
{
	// TODO: add check argv[1] is a number
	slave = (uint8_t) strtol(argv[1], &argv[1], 10);

	char *subcmd = argv[2];

    if( strcmp(subcmd,"init")==0 ) {
		return cmd_tmc_init(shell, argc, argv);

	} else if( strcmp(subcmd,"dump")==0 ) {
		return cmd_tmc_dump(shell, argc, argv);

	/*} else if( strcmp(subcmd,"get")==0 ) {
		return cmd_tmc_get(shell, argc, argv);

	} else if( strcmp(subcmd,"set")==0 ) {
		return cmd_tmc_set(shell, argc, argv);

	} else if( strcmp(subcmd,"cur")==0 ) {
		return cmd_tmc_cur(shell, argc, argv);
	*/
	} else if( strcmp(subcmd,"run")==0 ) {
		return cmd_tmc_run(shell, argc, argv);

	}

    return -EINVAL;

}
