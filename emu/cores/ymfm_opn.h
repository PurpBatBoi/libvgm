/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles
***************************************************************************/

#ifndef __YMFM_OPN_H__
#define __YMFM_OPN_H__

#include "../EmuStructs.h"

// YM2610 cfg.flags: 0 = YM2610 mode (4 FM channels), 1 = YM2610B mode (6 FM channels)

// YM2612 option bits set by the VGM player. The ymfm core does not use them.
#define OPT_YM2612_TYPE_OPN2C_ASIC	0x10	// emulate ASIC YM3438
#define OPT_YM2612_LEGACY_MODE		0x80	// simulate behaviour of older emulation cores (for GYM files)

extern const DEV_DECL sndDev_YM2203;
extern const DEV_DECL sndDev_YM2608;
extern const DEV_DECL sndDev_YM2610;
extern const DEV_DECL sndDev_YM2612;

#endif	// __YMFM_OPN_H__
