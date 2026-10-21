/*
	MIT License

	Copyright (c) 2026 Purpbatboi

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef __NES_EXPOUT_H__
#define __NES_EXPOUT_H__

#include "../../stdtype.h"
#include "../../common_def.h"
#include "../EmuStructs.h"
#include "../../libs/blep_filter/blep_filter.h"
#include "nes_filter.h"

/* Output stage of the NES expansion chips (VRC6, N163, Sunsoft 5B, MMC5), the same as the one of the BeesNES
	APU (beesnes_intf.c): the chip is clocked at the CPU clock, every change of its output level is added to a
	band-limited buffer (libs/blep_filter) at the clock cycle it happens in, and the buffer is read at the output
	rate and passed through the console's output filter (nes_filter.h). The filter is linear, so filtering each
	chip alone gives the same as filtering the sum, as on the console.

	A chip core owns one NES_EXPOUT, calls nes_expout_add() from its per-cycle loop and nes_expout_render() from
	its update function. The levels are integers: NES_EXPOUT_LEVEL_ONE is the level 1.0 of the BeesNES APU mix
	(its loudest output, the scale the chips' relative gains refer to), which leaves room for the overshoot of the
	band-limited steps below 32767. The output is scaled so that this level is 0x8000. */

/* Output samples rendered as one band-limited frame at most. */
#define NES_EXPOUT_FRAME_SAMPLES	1024
#define NES_EXPOUT_LEVEL_ONE		0x4000

typedef struct nes_expout
{
	blip_t* blep;
	INT32 level;	/* the level last added to the buffer */
	NES_OUTPUT_FILTER filter;
	UINT32 rate;
} NES_EXPOUT;

/* Clocks the chip `cycles` times, calling nes_expout_add() with the level after each cycle. */
typedef void (*NES_EXPOUT_RUN)(void* chip, UINT32 cycles);

/* Returns 0 on success. */
static UINT8 nes_expout_init(NES_EXPOUT* out, UINT32 clock, UINT32 rate)
{
	out->blep = blip_new(NES_EXPOUT_FRAME_SAMPLES);
	if (out->blep == NULL)
		return 0xFF;
	blip_set_rates(out->blep, clock, rate);
	out->rate = rate;
	out->level = 0;
	return 0x00;
}

static void nes_expout_free(NES_EXPOUT* out)
{
	blip_delete(out->blep);
	out->blep = NULL;
}

/* Starts over with the chip at `level`, so that the first sample is not a step. */
static void nes_expout_reset(NES_EXPOUT* out, INT32 level)
{
	blip_clear(out->blep);
	out->level = level;
	nes_output_filter_init(&out->filter, (float)out->rate);
}

/* The chip's level after clock cycle `cycle` of the frame in progress. */
INLINE void nes_expout_add(NES_EXPOUT* out, UINT32 cycle, INT32 level)
{
	if (level != out->level)
	{
		blip_add_delta(out->blep, cycle, level - out->level);
		out->level = level;
	}
}

/* Renders `samples` samples into outputs[0] and outputs[1] (both get the same mono signal). */
static void nes_expout_render(NES_EXPOUT* out, void* chip, NES_EXPOUT_RUN run, UINT32 samples, DEV_SMPL** outputs)
{
	short pcm[NES_EXPOUT_FRAME_SAMPLES];
	UINT32 done;
	UINT32 count;
	UINT32 cycles;
	UINT32 i;
	DEV_SMPL value;

	for (done = 0; done < samples; done += count)
	{
		count = samples - done;
		if (count > NES_EXPOUT_FRAME_SAMPLES)
			count = NES_EXPOUT_FRAME_SAMPLES;
		cycles = (UINT32)blip_clocks_needed(out->blep, (int)count);
		run(chip, cycles);
		blip_end_frame(out->blep, cycles);
		blip_read_samples(out->blep, pcm, (int)count, 0);
		/* The console's output filter, back to the 0x8000 = level 1.0 scale of the output. */
		for (i = 0; i < count; i ++)
		{
			value = (DEV_SMPL)(nes_output_filter(&out->filter, (float)pcm[i]) * (0x8000 / (float)NES_EXPOUT_LEVEL_ONE));
			outputs[0][done + i] = value;
			outputs[1][done + i] = value;
		}
	}
}

#endif	/* __NES_EXPOUT_H__ */
