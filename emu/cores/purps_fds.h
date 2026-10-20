#ifndef __PURPS_FDS_H__
#define __PURPS_FDS_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* Famicom Disk System sound, the FDS half of the BeesNES NES core. See purps_fds.c. */

/* Creates an FDS sound unit clocked at `clock` Hz (the CPU clock) that renders at `rate` Hz. */
void* purps_fds_create(UINT32 clock, UINT32 rate);
void purps_fds_destroy(void* chip);
void purps_fds_reset(void* chip);

/* `addr` is the CPU address: $4023, $4040-$407F (wavetable) or $4080-$409F. */
void purps_fds_write(void* chip, UINT16 addr, UINT8 data);
UINT8 purps_fds_read(void* chip, UINT16 addr);

/* OPT_NES_* flags from nesintf.h. Only OPT_NES_4085_RESET applies. */
void purps_fds_set_options(void* chip, UINT32 flags);
void purps_fds_set_mute(void* chip, UINT8 mute);
void purps_fds_set_panning(void* chip, INT16 pan);

/* Renders `samples` samples, adding to outputs[0] and outputs[1]. */
void purps_fds_update(void* chip, UINT32 samples, DEV_SMPL** outputs);

#endif	/* __PURPS_FDS_H__ */
