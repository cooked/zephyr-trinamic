/*
 * Copyright (c) 2022, Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <stdlib.h>
#include <zephyr/shell/shell.h>

#include "tmc.h"
#include "tmc_shell.h"
#include "tmc5160.h"

//extern bool toggle;
extern const struct device *tmc0;

//extern struct field fields[];

//extern uint8_t slave;

static int cmd_tmc_mode(const struct shell *shell, size_t argc, char *argv[])
{
	uint8_t mode = (uint8_t) strtol(argv[3], &argv[3], 10);

	tmc_set_mode(tmc0, 0, mode);

    return 0;
}
/*static int cmd_tmc_turn(const struct shell *shell, size_t argc, char *argv[])
{

	slave = (uint8_t) strtol(argv[CMD_ARG_N], &argv[CMD_ARG_N], 10);

	// increment/decrement current position by n (float) turns
	int32_t acc, speed;

	if(argc == CMD_ARG_N+3) {
		acc = (int32_t) strtol( argv[CMD_ARG_N+3], &argv[CMD_ARG_N+3], 10);
		tmc_set_amax(tmc0, slave, acc);
		speed = (int32_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);
		tmc_set_vmax(tmc0, slave, speed);
	} else if(argc == CMD_ARG_N+2) {
		speed = (int32_t) strtol( argv[CMD_ARG_N+2], &argv[CMD_ARG_N+2], 10);
		tmc_set_vmax(tmc0, slave, speed);
	}

	//float turns = strtod(argv[1], &argv[1]);
	int32_t turns = (int32_t) strtol( argv[CMD_ARG_N+1], &argv[CMD_ARG_N+1], 10);

	int32_t steps = tmc_get_xtarget(tmc0, slave);
	steps += (int32_t) (turns * DEFAULT_STEPS_TURN);

	tmc_set_xtarget(tmc0, slave, 51200);

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
}*/


/*SHELL_STATIC_SUBCMD_SET_CREATE( tmc_cmds,
	// set mode (ramp or velocity)
	SHELL_CMD_ARG(mode, NULL, "Set TMC mode of operation. ($ tmc mode <slave> <0/1> // 0:velocity 1:ramp)",
		cmd_tmc_mode, 2, 0),
	SHELL_CMD_ARG(turn, NULL,	"Turn motor by n turns (1.5: increment, -1.5: decrement)",
		cmd_tmc_turn, 3, 2),
	SHELL_CMD_ARG(goto, NULL,	"Move TMC to position",
		cmd_tmc_goto, 2, 0),
	SHELL_SUBCMD_SET_END
);*/

static int cmd_tmc5160(const struct shell *shell, size_t argc, char *argv[])
{
	// if generic tmc command we're done
	/*if( cmd_tmc(shell, argc, argv)==0 ) {
		return 0;
	};*/

	char *subcmd = argv[2];
    if( strcmp(subcmd,"mode")==0 ) {
		return cmd_tmc_mode(shell, argc, argv);

	} /*else if( strcmp(subcmd,"turn")==0 ) {
		return cmd_tmc_turn(shell, argc, argv);

	} else if( strcmp(subcmd,"goto")==0 ) {
		return cmd_tmc_goto(shell, argc, argv);

	}*/
    return -EINVAL;

}

SHELL_CMD_ARG_REGISTER(tmc, NULL, "TMC driver commands", cmd_tmc5160, 3, 99);