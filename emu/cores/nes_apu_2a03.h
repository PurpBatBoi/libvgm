#ifndef __NES_APU_2A03_H__
#define __NES_APU_2A03_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* NES 2A03 APU: the MIT-licensed apu2a03.c core, rendered through a band-limited synth. */

/* Creates an APU clocked at `clock` Hz (the CPU clock) that renders at `rate` Hz. */
void* nesapu_2a03_create(UINT32 clock, UINT32 rate);
void nesapu_2a03_destroy(void* chip);
void nesapu_2a03_reset(void* chip);

/* `offset` is the register address minus $4000 ($00-$1F). */
void nesapu_2a03_write(void* chip, UINT8 offset, UINT8 data);
UINT8 nesapu_2a03_read(void* chip, UINT8 offset);

/* DPCM sample memory: 32 KB holding CPU addresses $8000-$FFFF. */
void nesapu_2a03_set_memory(void* chip, const UINT8* memory);

/* OPT_NES_* flags from nesintf.h. */
void nesapu_2a03_set_options(void* chip, UINT32 flags);
/* Bit 0-4: square 1, square 2, triangle, noise, DPCM. */
void nesapu_2a03_set_mute_mask(void* chip, UINT32 mask);
/* One position per channel, in the order of the mute mask. Only used by the linear mixer. */
void nesapu_2a03_set_panning(void* chip, const INT16* pan);

/* Renders `samples` samples, overwriting outputs[0] and outputs[1]. */
void nesapu_2a03_update(void* chip, UINT32 samples, DEV_SMPL** outputs);

#endif	/* __NES_APU_2A03_H__ */
