/**
 * Copyright 2022 Stefano Cottafavi <stefano.cottafavi@gmail.com>.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "tmc.h"

attr get_field(char *key, struct field *fields) {
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
