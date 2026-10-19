#ifndef __BEENES_INTF_H__
#define __BEENES_INTF_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* Creates an APU clocked at `clock` Hz (the CPU clock) that renders at `rate` Hz. */
void* beenes_intf_create(UINT32 clock, UINT32 rate);
void beenes_intf_destroy(void* chip);
void beenes_intf_reset(void* chip);

/* `offset` is the register address minus $4000 ($00-$1F). */
void beenes_intf_write(void* chip, UINT8 offset, UINT8 data);
UINT8 beenes_intf_read(void* chip, UINT8 offset);

/* DPCM sample memory: 32 KB holding CPU addresses $8000-$FFFF. */
void beenes_intf_set_memory(void* chip, const UINT8* memory);

/* OPT_NES_* flags from nesintf.h. */
void beenes_intf_set_options(void* chip, UINT32 flags);
/* Bit 0-4: square 1, square 2, triangle, noise, DPCM. */
void beenes_intf_set_mute_mask(void* chip, UINT32 mask);
/* One position per channel, in the order of the mute mask. Only used by the linear mixer. */
void beenes_intf_set_panning(void* chip, const INT16* pan);

/* Renders `samples` samples, overwriting outputs[0] and outputs[1]. */
void beenes_intf_update(void* chip, UINT32 samples, DEV_SMPL** outputs);

#endif	/* __BEENES_INTF_H__ */
