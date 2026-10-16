#ifndef __HUC6280A_H__
#define __HUC6280A_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"

/* huc6280a: the PSG of the HuC6280A, the PC Engine's 6-channel wavetable sound. See
	huc6280a.c for the license.

	The chip reports each channel's changes of output level with the clock they happen on, and
	leaves mixing and resampling to the caller. One PSG clock is 1/7,159,090.9 s
	(315,000,000 / 44 Hz). Documents that give the chip a 3.58 MHz clock count at half this rate:
	a host keeping time in those units passes twice its count, or every pitch comes out an
	octave high. */

#define HUC6280A_CHANNELS	6

/* One channel's change of output level. `left` and `right` are signed levels. */
typedef struct
{
	UINT64 clock;
	UINT8 channel;
	INT16 left;
	INT16 right;
} HUC6280A_EVENT;

typedef void (*HUC6280A_SINK)(void* user, const HUC6280A_EVENT* event);

/* Returns a chip in its power-on state, or NULL if out of memory. */
void* huc6280a_create(void);
void huc6280a_destroy(void* chip);
/* Power-on state (§4.2). Restarts the clock at 0. */
void huc6280a_reset(void* chip);

/* Writes register `reg` (0-15) at the current clock (§3). Level changes the write causes are
	reported, at this clock, by the next huc6280a_run that advances time. */
void huc6280a_write(void* chip, UINT8 reg, UINT8 data);

/* Advances `clocks` PSG clocks, reporting each level change to `sink` (which may be NULL) with
	start <= clock <= start + clocks. A level change at exactly the end (a volume sequencer
	apply) is reported by the next call, at that clock, and a zero-length run reports nothing.
	One channel's events arrive in nondecreasing clock order; different channels' events may
	interleave out of order within a call (§12). */
void huc6280a_run(void* chip, UINT32 clocks, HUC6280A_SINK sink, void* user);

/* The libvgm C6280 core, FCC_HUCA ("huc6280a"). */
extern DEV_DEF devDef_C6280_HUCA;

#endif	/* __HUC6280A_H__ */
