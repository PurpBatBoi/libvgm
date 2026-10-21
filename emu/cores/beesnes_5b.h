#ifndef __BEESNES_5B_H__
#define __BEESNES_5B_H__

#include "../EmuStructs.h"

// Sunsoft 5B expansion audio (3 square-wave tone channels and a noise generator, the tone/noise part
// of the AY-3-8910 family; there is no envelope generator, see beesnes_5b.c).
// Registers: DEVRW_A16D8 with the CPU address:
//   $C000-$DFFF  write: select the register (the upper 4 bits must be 0 for a later data write to count)
//   $E000-$FFFF  write: data for the selected register
// cfg.clock is the CPU clock (1789773 Hz NTSC, 1662607 Hz PAL); 0 selects the NTSC clock.
// Mute mask: bit 0-2 = tone channel A-C.
extern const DEV_DECL sndDev_Sunsoft5B;

#endif	// __BEESNES_5B_H__
