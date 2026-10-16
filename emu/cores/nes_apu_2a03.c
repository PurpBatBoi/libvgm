/*
	MIT License

	Copyright (c) 2021 Luke Petherbridge <me@lukeworks.tech>
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

#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../panning.h"
#include "nesintf.h"
#include "nes_apu_2a03.h"
#include "nes_synth_2a03.h"
#include "apu2a03.h"

/* How many CPU cycles a mixing block spans at most. */
#define CYCLE_SIZE	10000

#define CHN_COUNT	LSN_C_TOTAL

typedef struct nesapu_2a03
{
	UINT32 sample_rate;
	UINT8 pal;

	lsn_apu2a03_t core;

	SYNTH synth;
	FILTER_CHAIN filter[2];
	/* Mixed level as of the last change handed to the synth, so a change is a delta from it. */
	float mixed_level[2];
	/* Scratch for one block of samples. */
	float* block;

	/* DPCM sample memory for CPU addresses $8000-$FFFF. */
	const UINT8* memory;

	UINT32 options;	/* OPT_NES_* */
	UINT8 mute_mask;
	/* Per-channel gains for the linear mixer: [channel][left, right]. */
	float pan[CHN_COUNT][2];

	/* The console's non-linear mixer, from https://www.nesdev.org/wiki/APU_Mixer */
	float pulse_table[31];
	float tnd_table[203];
} NESAPU_2A03;

static UINT8 apu_dma_read(void* user_data, UINT16 addr)
{
	const NESAPU_2A03* apu = (const NESAPU_2A03*)user_data;

	return (apu->memory != NULL && addr >= 0x8000) ? apu->memory[addr - 0x8000] : 0x00;
}

/* Powers the APU core up from scratch. */
static void apu_start_core(NESAPU_2A03* apu)
{
	lsn_apu2a03_init(&apu->core, apu->pal ? LSN_AT_PAL : LSN_AT_NTSC);
	/* The core fetches each DPCM byte through dma_read. */
	lsn_apu2a03_set_dma_read(&apu->core, apu, apu_dma_read);
	/* Famiclone duty order: 25% and 50% swapped. */
	apu->core.swap_duty = (apu->options & OPT_NES_DUTY_SWAP) ? 1 : 0;
}

/* The five channels mixed, into out[0] (left) and out[1] (right). */
static void apu_mix_level(const NESAPU_2A03* apu, float* out)
{
	/* The linear approximation's gains, from the same wiki page. */
	static const float LINEAR_GAIN[CHN_COUNT] = {0.00752f, 0.00752f, 0.00851f, 0.00494f, 0.00335f};
	const lsn_triangle_t* tri = &apu->core.triangle;
	UINT8 mute = apu->mute_mask;
	UINT8 raw[CHN_COUNT];
	float level[CHN_COUNT];
	UINT8 chn;

	lsn_apu2a03_get_channel_levels(&apu->core, raw);
	for (chn = 0; chn < CHN_COUNT; chn ++)
		level[chn] = (mute & (1 << chn)) ? 0.0f : (float)raw[chn];
	/* Ultrasonic periods; a console's low-pass filter leaves the middle of the wave, which
		saves the synth a step on nearly every cycle. */
	if (! (mute & 0x04) && tri->sequencer.reload < 2 && tri->length.counter > 0 &&
		tri->linear.linear_counter > 0)
		level[LSN_C_TRIANGLE] = 7.5f;

	if (apu->options & OPT_NES_NONLINEAR_MIXER)
	{
		UINT32 tnd = (UINT32)(3.0f * level[LSN_C_TRIANGLE] + 2.0f * level[LSN_C_NOISE] +
			level[LSN_C_DMC]);

		out[0] = apu->pulse_table[(UINT32)(level[LSN_C_PULSE_0] + level[LSN_C_PULSE_1])] +
			apu->tnd_table[tnd];
		out[1] = out[0];
	}
	else
	{
		/* The linear approximation, which lets each channel be panned. */
		out[0] = out[1] = 0.0f;
		for (chn = 0; chn < CHN_COUNT; chn ++)
		{
			out[0] += LINEAR_GAIN[chn] * level[chn] * apu->pan[chn][0];
			out[1] += LINEAR_GAIN[chn] * level[chn] * apu->pan[chn][1];
		}
	}
}

/* Clocks the core `cycles` times, handing the synth each change of the mixed level. */
static void apu_run(NESAPU_2A03* apu, UINT32 cycles)
{
	UINT32 cycle;
	float level[2];

	for (cycle = 0; cycle < cycles; cycle ++)
	{
		lsn_apu2a03_tick(&apu->core);
		apu_mix_level(apu, level);
		if (level[0] != apu->mixed_level[0] || level[1] != apu->mixed_level[1])
		{
			synth_add_delta(&apu->synth, cycle,
				level[0] - apu->mixed_level[0], level[1] - apu->mixed_level[1]);
			apu->mixed_level[0] = level[0];
			apu->mixed_level[1] = level[1];
		}
	}
}

void* nesapu_2a03_create(UINT32 clock, UINT32 rate)
{
	NESAPU_2A03* apu;
	int i;

	if (clock == 0 || rate == 0 || rate > clock / 4)
		return NULL;
	apu = (NESAPU_2A03*)calloc(1, sizeof(NESAPU_2A03));
	if (apu == NULL)
		return NULL;
	apu->sample_rate = rate;
	/* PAL consoles run at 1.66 MHz, NTSC at 1.79 MHz and Dendy (NTSC timing) at 1.77 MHz. */
	apu->pal = (clock < 1700000) ? 1 : 0;

	if (! synth_init(&apu->synth, clock, rate, CYCLE_SIZE))
	{
		nesapu_2a03_destroy(apu);
		return NULL;
	}
	apu->block = (float*)calloc(apu->synth.length, sizeof(float));
	if (apu->block == NULL)
	{
		nesapu_2a03_destroy(apu);
		return NULL;
	}

	/* Entry 0 of each mixer table is left at 0 by calloc. */
	for (i = 1; i < 31; i ++)
		apu->pulse_table[i] = 95.52f / (8128.0f / (float)i + 100.0f);
	for (i = 1; i < 203; i ++)
		apu->tnd_table[i] = 163.67f / (24329.0f / (float)i + 100.0f);
	for (i = 0; i < CHN_COUNT; i ++)
		apu->pan[i][0] = apu->pan[i][1] = 1.0f;
	apu->options = OPT_NES_UNMUTE_ON_RESET | OPT_NES_NONLINEAR_MIXER | OPT_NES_ENABLE_4011 |
		OPT_NES_ENABLE_PNOISE;

	nesapu_2a03_reset(apu);
	return apu;
}

void nesapu_2a03_destroy(void* chip)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	synth_free(&apu->synth);
	free(apu->block);
	free(apu);
}

void nesapu_2a03_reset(void* chip)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	apu_start_core(apu);
	apu->mixed_level[0] = apu->mixed_level[1] = 0.0f;
	synth_clear(&apu->synth);
	filter_init(&apu->filter[0], (float)apu->sample_rate);
	filter_init(&apu->filter[1], (float)apu->sample_rate);

	/* Some VGMs were logged without the driver's $4015 write, and only play with the pulse,
		triangle and noise channels enabled from the start. */
	if (apu->options & OPT_NES_UNMUTE_ON_RESET)
		nesapu_2a03_write(apu, 0x15, 0x0F);
}

void nesapu_2a03_write(void* chip, UINT8 offset, UINT8 data)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	switch(offset)
	{
	case 0x0E:
		/* Early Famicoms and some arcade boards have no short noise mode. */
		if (! (apu->options & OPT_NES_ENABLE_PNOISE))
			data &= 0x7F;
		break;
	case 0x11:
		if (! (apu->options & OPT_NES_ENABLE_4011))
			return;
		break;
	}
	lsn_apu2a03_write(&apu->core, (UINT16)(0x4000 | offset), data);
}

UINT8 nesapu_2a03_read(void* chip, UINT8 offset)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	return lsn_apu2a03_read(&apu->core, (UINT16)(0x4000 | offset));
}

void nesapu_2a03_set_memory(void* chip, const UINT8* memory)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	apu->memory = memory;
}

/* UNMUTE_ON_RESET and the options that describe hardware variants are honored, the first at the
	next reset. The others (PHASE_REFRESH, DPCM_ANTI_CLICK, RANDOMIZE_NOISE, TRI_MUTE, TRI_NULL)
	tune the MAME and NSFPlay cores' approximations of behavior this core emulates as the
	hardware does. */
void nesapu_2a03_set_options(void* chip, UINT32 flags)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	apu->options = flags;
	apu->core.swap_duty = (flags & OPT_NES_DUTY_SWAP) ? 1 : 0;
}

void nesapu_2a03_set_mute_mask(void* chip, UINT32 mask)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;

	apu->mute_mask = (UINT8)mask;
}

void nesapu_2a03_set_panning(void* chip, const INT16* pan)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;
	INT32 gains[2];
	UINT8 chn;

	for (chn = 0; chn < CHN_COUNT; chn ++)
	{
		Panning_Calculate(gains, pan[chn]);
		apu->pan[chn][0] = (float)gains[0] / PANNING_NORMAL;
		apu->pan[chn][1] = (float)gains[1] / PANNING_NORMAL;
	}
}

void nesapu_2a03_update(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	NESAPU_2A03* apu = (NESAPU_2A03*)chip;
	SYNTH* s = &apu->synth;
	UINT32 done = 0;

	while (done < samples)
	{
		INT64 needed = ((INT64)(samples - done) << SYNTH_FRAC_BITS) - s->offset;
		UINT32 cycles;
		UINT32 count;
		UINT32 i;
		UINT8 chn;

		/* The fewest cycles that complete the remaining samples, at most one block. */
		if (needed > (INT64)CYCLE_SIZE * s->samples_per_cycle)
			cycles = CYCLE_SIZE;
		else
			cycles = (UINT32)((needed + s->samples_per_cycle - 1) / s->samples_per_cycle);
		apu_run(apu, cycles);
		count = synth_end_block(s, cycles);
		if (count > samples - done)
			count = samples - done;

		for (chn = 0; chn < 2; chn ++)
		{
			synth_read(s, chn, count, apu->block);
			for (i = 0; i < count; i ++)
				outputs[chn][done + i] = (DEV_SMPL)(filter_consume(&apu->filter[chn], apu->block[i]) * 0x8000);
		}
		synth_consume(s, count);
		done += count;
	}
}
