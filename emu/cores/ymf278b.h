#ifndef __YMF278B_H__
#define __YMF278B_H__

#include "../EmuStructs.h"

// The FM part is a linked YMF262 device (DEVID_YMF262, see ymf262.h).
// Memory functions: 0x524F = ROM (the YRW801), 0x5241 = RAM.

extern const DEV_DECL sndDev_YMF278B;

#endif	// __YMF278B_H__
