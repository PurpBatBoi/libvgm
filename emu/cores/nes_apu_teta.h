#ifndef __NES_APU_TETA_H__
#define __NES_APU_TETA_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* NES 2A03 APU, ported from TetaNES. See nes_apu_teta.c for the license. */

/* Creates an APU clocked at `clock` Hz (the CPU clock) that renders at `rate` Hz. */
void* nesapu_teta_create(UINT32 clock, UINT32 rate);
void nesapu_teta_destroy(void* chip);
void nesapu_teta_reset(void* chip);

/* `offset` is the register address minus $4000 ($00-$1F). */
void nesapu_teta_write(void* chip, UINT8 offset, UINT8 data);
UINT8 nesapu_teta_read(void* chip, UINT8 offset);

/* DPCM sample memory: 32 KB holding CPU addresses $8000-$FFFF. */
void nesapu_teta_set_memory(void* chip, const UINT8* memory);

/* OPT_NES_* flags from nesintf.h. */
void nesapu_teta_set_options(void* chip, UINT32 flags);
/* Bit 0-4: square 1, square 2, triangle, noise, DPCM. */
void nesapu_teta_set_mute_mask(void* chip, UINT32 mask);
/* One position per channel, in the order of the mute mask. Only used by the linear mixer. */
void nesapu_teta_set_panning(void* chip, const INT16* pan);

/* Renders `samples` samples, overwriting outputs[0] and outputs[1]. */
void nesapu_teta_update(void* chip, UINT32 samples, DEV_SMPL** outputs);

#endif	/* __NES_APU_TETA_H__ */
