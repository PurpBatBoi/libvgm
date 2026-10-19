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

#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../panning.h"
#include "../../libs/blep_filter/blep_filter.h"
#include "nesintf.h"
#include "beenes_intf.h"
#include "nes_filter.h"
#include "beenes_apu.h"

#define CHN_COUNT	BEENES_C_TOTAL

/* Output samples rendered as one band-limited frame at most. */
#define FRAME_SAMPLES	1024
/* blep_filter works on integer levels: the mixed level 1.0 (the loudest the non-linear mixer gives)
	is this, which leaves room for the overshoot of the band-limited steps below 32767. */
#define LEVEL_ONE	0x4000

typedef struct beenes_intf
{
	UINT32 sample_rate;
	UINT8 pal;

	beenes_apu_t core;

	/* One band-limited buffer per side, with the integer level last added to it. */
	blip_t* blep[2];
	INT32 blep_level[2];
	NES_OUTPUT_FILTER filter[2];

	/* DPCM sample memory for CPU addresses $8000-$FFFF. */
	const UINT8* memory;

	UINT32 options;	/* OPT_NES_* */
	UINT8 mute_mask;
	/* Per-channel gains for the linear mixer: [channel][left, right]. */
	float pan[CHN_COUNT][2];

	/* The console's non-linear mixer, from https://www.nesdev.org/wiki/APU_Mixer */
	float pulse_table[31];
	float tnd_table[203];
} BEENES_INTF;

static UINT8 apu_dma_read(void* user_data, UINT16 addr)
{
	const BEENES_INTF* apu = (const BEENES_INTF*)user_data;

	return (apu->memory != NULL && addr >= 0x8000) ? apu->memory[addr - 0x8000] : 0x00;
}

/* Powers the APU core up from scratch. */
static void apu_start_core(BEENES_INTF* apu)
{
	beenes_apu_init(&apu->core, apu->pal ? BEENES_AT_PAL : BEENES_AT_NTSC);
	/* The core fetches each DPCM byte through dma_read. */
	beenes_apu_set_dma_read(&apu->core, apu, apu_dma_read);
	/* Famiclone duty order: 25% and 50% swapped. */
	apu->core.swap_duty = (apu->options & OPT_NES_DUTY_SWAP) ? 1 : 0;
}

/* The five channels mixed, into out[0] (left) and out[1] (right). */
static void apu_mix_level(const BEENES_INTF* apu, float* out)
{
	/* The linear approximation's gains, from the same wiki page. */
	static const float LINEAR_GAIN[CHN_COUNT] = {0.00752f, 0.00752f, 0.00851f, 0.00494f, 0.00335f};
	const beenes_triangle_t* tri = &apu->core.triangle;
	UINT8 mute = apu->mute_mask;
	UINT8 raw[CHN_COUNT];
	float level[CHN_COUNT];
	UINT8 chn;

	beenes_apu_get_channel_levels(&apu->core, raw);
	for (chn = 0; chn < CHN_COUNT; chn ++)
		level[chn] = (mute & (1 << chn)) ? 0.0f : (float)raw[chn];
	/* At the shortest periods the triangle is ultrasonic, and the console's low-pass leaves only
		the middle of its wave. Using that level also saves a step on nearly every cycle. */
	if (! (mute & 0x04) && tri->sequencer.reload < 2 && tri->length.counter > 0 &&
		tri->linear.linear_counter > 0)
		level[BEENES_C_TRIANGLE] = 7.5f;

	if (apu->options & OPT_NES_NONLINEAR_MIXER)
	{
		UINT32 tnd = (UINT32)(3.0f * level[BEENES_C_TRIANGLE] + 2.0f * level[BEENES_C_NOISE] +
			level[BEENES_C_DMC]);

		out[0] = apu->pulse_table[(UINT32)(level[BEENES_C_PULSE_0] + level[BEENES_C_PULSE_1])] +
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

/* Clocks the core `cycles` times, adding each change of the mixed level as a band-limited step. */
static void apu_run(BEENES_INTF* apu, UINT32 cycles)
{
	UINT32 cycle;
	float level[2];
	INT32 value;
	UINT8 side;

	for (cycle = 0; cycle < cycles; cycle ++)
	{
		beenes_apu_tick(&apu->core);
		apu_mix_level(apu, level);
		for (side = 0; side < 2; side ++)
		{
			value = (INT32)(level[side] * LEVEL_ONE + 0.5f);
			if (value != apu->blep_level[side])
			{
				blip_add_delta(apu->blep[side], cycle, value - apu->blep_level[side]);
				apu->blep_level[side] = value;
			}
		}
	}
}

void* beenes_intf_create(UINT32 clock, UINT32 rate)
{
	BEENES_INTF* apu;
	int i;

	if (clock == 0 || rate == 0 || rate > clock / 4)
		return NULL;
	apu = (BEENES_INTF*)calloc(1, sizeof(BEENES_INTF));
	if (apu == NULL)
		return NULL;
	apu->sample_rate = rate;
	/* PAL consoles run at 1.66 MHz, NTSC at 1.79 MHz and Dendy (NTSC timing) at 1.77 MHz. */
	apu->pal = (clock < 1700000) ? 1 : 0;

	for (i = 0; i < 2; i ++)
	{
		apu->blep[i] = blip_new(FRAME_SAMPLES);
		if (apu->blep[i] == NULL)
		{
			beenes_intf_destroy(apu);
			return NULL;
		}
		blip_set_rates(apu->blep[i], clock, rate);
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

	beenes_intf_reset(apu);
	return apu;
}

void beenes_intf_destroy(void* chip)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

	blip_delete(apu->blep[0]);
	blip_delete(apu->blep[1]);
	free(apu);
}

void beenes_intf_reset(void* chip)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;
	UINT8 side;

	apu_start_core(apu);
	for (side = 0; side < 2; side ++)
	{
		blip_clear(apu->blep[side]);
		apu->blep_level[side] = 0;
		nes_output_filter_init(&apu->filter[side], (float)apu->sample_rate);
	}

	/* Some VGMs were logged without the driver's $4015 write, and only play with the pulse,
		triangle and noise channels enabled from the start. */
	if (apu->options & OPT_NES_UNMUTE_ON_RESET)
		beenes_intf_write(apu, 0x15, 0x0F);
}

void beenes_intf_write(void* chip, UINT8 offset, UINT8 data)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

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
	beenes_apu_write(&apu->core, (UINT16)(0x4000 | offset), data);
}

UINT8 beenes_intf_read(void* chip, UINT8 offset)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

	return beenes_apu_read(&apu->core, (UINT16)(0x4000 | offset));
}

void beenes_intf_set_memory(void* chip, const UINT8* memory)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

	apu->memory = memory;
}

/* UNMUTE_ON_RESET and the options that describe hardware variants are honored, the first at the
	next reset. The others (PHASE_REFRESH, DPCM_ANTI_CLICK, RANDOMIZE_NOISE, TRI_MUTE, TRI_NULL)
	tune the MAME and NSFPlay cores' approximations of behavior this core emulates as the
	hardware does. */
void beenes_intf_set_options(void* chip, UINT32 flags)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

	apu->options = flags;
	apu->core.swap_duty = (flags & OPT_NES_DUTY_SWAP) ? 1 : 0;
}

void beenes_intf_set_mute_mask(void* chip, UINT32 mask)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;

	apu->mute_mask = (UINT8)mask;
}

void beenes_intf_set_panning(void* chip, const INT16* pan)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;
	INT32 gains[2];
	UINT8 chn;

	for (chn = 0; chn < CHN_COUNT; chn ++)
	{
		Panning_Calculate(gains, pan[chn]);
		apu->pan[chn][0] = (float)gains[0] / PANNING_NORMAL;
		apu->pan[chn][1] = (float)gains[1] / PANNING_NORMAL;
	}
}

void beenes_intf_update(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	BEENES_INTF* apu = (BEENES_INTF*)chip;
	short pcm[FRAME_SAMPLES];
	UINT32 done;
	UINT32 count;
	UINT32 cycles;
	UINT32 i;
	UINT8 side;

	for (done = 0; done < samples; done += count)
	{
		count = samples - done;
		if (count > FRAME_SAMPLES)
			count = FRAME_SAMPLES;
		/* Both buffers run at the same rates, so they need the same cycles. */
		cycles = (UINT32)blip_clocks_needed(apu->blep[0], (int)count);
		apu_run(apu, cycles);
		for (side = 0; side < 2; side ++)
		{
			blip_end_frame(apu->blep[side], cycles);
			blip_read_samples(apu->blep[side], pcm, (int)count, 0);
			/* The console's output filters, back to the 0x8000 = level 1.0 scale of the output. */
			for (i = 0; i < count; i ++)
				outputs[side][done + i] = (DEV_SMPL)(nes_output_filter(&apu->filter[side], (float)pcm[i]) *
					(0x8000 / (float)LEVEL_ONE));
		}
	}
}
