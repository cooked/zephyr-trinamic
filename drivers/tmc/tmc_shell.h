/*
 * Copyright (c) 2022, Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TMC_SHELL_H
#define TMC_SHELL_H

#include <stdlib.h> // size_t

#include "tmc.h"

#define CMD_ARG_N 1

int cmd_tmc(const struct shell *shell, size_t argc, char *argv[]);

#endif