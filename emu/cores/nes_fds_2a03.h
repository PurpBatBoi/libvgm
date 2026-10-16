#ifndef __NES_FDS_2A03_H__
#define __NES_FDS_2A03_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* Famicom Disk System sound, the FDS half of the APU2A03 NES core. See nes_fds_2a03.c. */

/* Creates an FDS sound unit clocked at `clock` Hz (the CPU clock) that renders at `rate` Hz. */
void* nesfds_2a03_create(UINT32 clock, UINT32 rate);
void nesfds_2a03_destroy(void* chip);
void nesfds_2a03_reset(void* chip);

/* `addr` is the CPU address: $4023, $4040-$407F (wavetable) or $4080-$409F. */
void nesfds_2a03_write(void* chip, UINT16 addr, UINT8 data);
UINT8 nesfds_2a03_read(void* chip, UINT16 addr);

/* OPT_NES_* flags from nesintf.h. Only OPT_NES_4085_RESET applies. */
void nesfds_2a03_set_options(void* chip, UINT32 flags);
void nesfds_2a03_set_mute(void* chip, UINT8 mute);
void nesfds_2a03_set_panning(void* chip, INT16 pan);

/* Renders `samples` samples, adding to outputs[0] and outputs[1]. */
void nesfds_2a03_update(void* chip, UINT32 samples, DEV_SMPL** outputs);

#endif	/* __NES_FDS_2A03_H__ */
