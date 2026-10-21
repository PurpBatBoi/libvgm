#ifndef __BEESNES_VRC6_H__
#define __BEESNES_VRC6_H__

#include "../EmuStructs.h"

// Konami VRC6 expansion audio (2 pulse channels and a saw channel).
// Registers: DEVRW_A16D8 with the CPU address ($9000-$9003, $A000-$A002, $B000-$B002, mirrored
// as on the chip: only A15-A12, A1 and A0 are decoded).
// cfg.clock is the CPU clock (1789773 Hz NTSC, 1662607 Hz PAL); 0 selects the NTSC clock.
// Mute mask: bit 0 = pulse 1, bit 1 = pulse 2, bit 2 = saw.
extern const DEV_DECL sndDev_VRC6;

#endif	// __BEESNES_VRC6_H__
