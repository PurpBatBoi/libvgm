#ifndef __BEESNES_N163_H__
#define __BEESNES_N163_H__

#include "../EmuStructs.h"

// Namco 163 expansion audio (up to 8 wavetable channels, time-multiplexed).
// Registers: DEVRW_A16D8 with the CPU address, as the chip is mapped:
//   $4800-$4FFF  data port: read and write of the 128-byte internal RAM at the current address
//   $E000-$E7FF  write: bit 6 set disables the sound (the chip is enabled after a reset)
//   $F800-$FFFF  write: address port (bits 0-6 = RAM address, bit 7 = auto-increment)
// cfg.clock is the CPU clock (1789773 Hz NTSC, 1662607 Hz PAL); 0 selects the NTSC clock.
// Mute mask: bit 0-7 = channel 1-8 (channel 1 is the one at $78-$7F of the RAM).
extern const DEV_DECL sndDev_N163;

#endif	// __BEESNES_N163_H__
