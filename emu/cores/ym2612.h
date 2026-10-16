#ifndef __YM2612_H__
#define __YM2612_H__

#include "../EmuStructs.h"

// Option bits set by the VGM player. The ares core does not use them.
#define OPT_YM2612_TYPE_OPN2C_ASIC	0x10	// emulate ASIC YM3438
#define OPT_YM2612_LEGACY_MODE		0x80	// simulate behaviour of older emulation cores (for GYM files)

extern const DEV_DECL sndDev_YM2612;

#endif	// __YM2612_H__
