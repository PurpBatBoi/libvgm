#ifndef __ARES_MMC5_H__
#define __ARES_MMC5_H__

#include "../EmuStructs.h"

// Nintendo MMC5 expansion audio (2 pulse channels and an 8-bit PCM channel).
// Registers: DEVRW_A16D8 with the CPU address, write: $5000-$5007 (pulses), $5010-$5011 (PCM),
// $5015 (pulse enables); read: $5015 (length counter status, bits 0 and 1). Other addresses are
// ignored (the multiplier, ExRAM, banking and IRQ of the board belong to the player).
// cfg.clock is the CPU clock (1789773 Hz NTSC, 1662607 Hz PAL); 0 selects the NTSC clock.
// Mute mask: bit 0 = pulse 1, bit 1 = pulse 2, bit 2 = PCM.
extern const DEV_DECL sndDev_MMC5;

#endif	// __ARES_MMC5_H__
