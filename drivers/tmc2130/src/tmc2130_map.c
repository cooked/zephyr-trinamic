/*
 * Copyright (c) 2022, Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tmc.h"
#include "tmc2130.h"

struct reg regs[] = {
	{"GCONF",		{ TMC2130_GCONF, 		"rw"	}},
	{"GSTAT",		{ TMC2130_GSTAT, 		"rwc"   }},
    {"IHOLD_IRUN",	{ TMC2130_IHOLD_IRUN, 	"w"		}},
};

// map fields
struct field fields[] = {
	// 0x00 GCONF
	{ "iscaleanalog",{TMC2130_GCONF, 		TMC2130_SHAFT_MASK, 	TMC2130_SHAFT_SHIFT}},
	{ "shaft", 		{TMC2130_GCONF, 		TMC2130_SHAFT_MASK, 	TMC2130_SHAFT_SHIFT}},
	// TODO: complete above
	// 0x01 GSTAT
	{ "reset", 		{TMC2130_GSTAT, 		TMC2130_RESET_MASK, 	TMC2130_RESET_SHIFT}},
	{ "drverr", 	{TMC2130_GSTAT, 		TMC2130_DRV_ERR_MASK, 	TMC2130_DRV_ERR_SHIFT}},
	{ "uvcp", 		{TMC2130_GSTAT, 		TMC2130_UV_CP_MASK, 	TMC2130_UV_CP_SHIFT}},

	// 0x10 IHOLD_IRUN
	{ "ihold", 		{TMC2130_IHOLD_IRUN, 	TMC2130_IHOLD_MASK, 		TMC2130_IHOLD_SHIFT}},
    { "irun",  		{TMC2130_IHOLD_IRUN, 	TMC2130_IRUN_MASK, 			TMC2130_IRUN_SHIFT}},
	{ "iholddelay", {TMC2130_IHOLD_IRUN, 	TMC2130_IHOLDDELAY_MASK, 	TMC2130_IHOLDDELAY_SHIFT}},
	// 0x12 TSTEP
	{ "tstep", 		{TMC2130_TSTEP, 		TMC2130_TSTEP_MASK, 	TMC2130_TSTEP_SHIFT}},

	// driver status
	{ "stst",		{TMC2130_DRV_STATUS, 	TMC2130_STST_MASK, 			TMC2130_STST_SHIFT}},
	{ "olb",		{TMC2130_DRV_STATUS, 	TMC2130_OLB_MASK, 			TMC2130_OLB_SHIFT}},
	{ "ola",		{TMC2130_DRV_STATUS, 	TMC2130_OLA_MASK, 			TMC2130_OLA_SHIFT}},
	{ "s2gb",		{TMC2130_DRV_STATUS, 	TMC2130_S2GB_MASK, 			TMC2130_S2GB_SHIFT}},
	{ "s2ga",		{TMC2130_DRV_STATUS, 	TMC2130_S2GA_MASK, 			TMC2130_S2GA_SHIFT}},
	{ "otpw",		{TMC2130_DRV_STATUS, 	TMC2130_OTPW_MASK, 			TMC2130_OTPW_SHIFT}},
	{ "ot",			{TMC2130_DRV_STATUS, 	TMC2130_OT_MASK, 			TMC2130_OT_SHIFT}},
	{ "stallguard",	{TMC2130_DRV_STATUS, 	TMC2130_STALLGUARD_MASK, 	TMC2130_STALLGUARD_SHIFT}},
	{ "csactual",	{TMC2130_DRV_STATUS, 	TMC2130_CS_ACTUAL_MASK, 	TMC2130_CS_ACTUAL_SHIFT}},
	{ "fsactive",	{TMC2130_DRV_STATUS, 	TMC2130_FSACTIVE_MASK, 		TMC2130_FSACTIVE_SHIFT}},
	{ "sgresult",	{TMC2130_DRV_STATUS, 	TMC2130_SG_RESULT_MASK, 	TMC2130_SG_RESULT_SHIFT}},
};