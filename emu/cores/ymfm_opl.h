/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles
***************************************************************************/

#ifndef __YMFM_OPL_H__
#define __YMFM_OPL_H__

#include "../EmuStructs.h"

// YMF262 cfg.flags: set for the FM part of a YMF278B (OPL4): stereo output is A / B only
#define YMF262_FLAG_OPL4	0x01

extern const DEV_DECL sndDev_YM3526;
extern const DEV_DECL sndDev_Y8950;
extern const DEV_DECL sndDev_YM3812;
extern const DEV_DECL sndDev_YMF262;

// YMF278B: the FM part is a linked YMF262 device (DEVID_YMF262, YMF262_FLAG_OPL4).
// Memory functions: 0x524F = ROM (the YRW801), 0x5241 = RAM.
extern const DEV_DECL sndDev_YMF278B;

#endif	// __YMFM_OPL_H__
