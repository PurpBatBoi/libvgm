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
#include "nesintf.h"
#include "nes_fds_2a03.h"
#include "nes_synth_2a03.h"

/* At most this many CPU cycles are rendered as one synth block. */
#define CYCLE_SIZE	10000
/* The wave and mod units tick every 16 CPU cycles. */
#define TICK_CYCLES	16

/* Mod counter change for each 3-bit mod table entry. Entry 4 resets the counter instead. */
static const INT32 MOD_ADJUST[8] = {0, 1, 2, 4, 0, -4, -2, -1};
#define MOD_RESET	4

typedef struct
{
	UINT8 off;	/* bit 7: the gain is set directly and the envelope doesn't run */
	UINT8 increase;	/* bit 6 */
	UINT8 speed;	/* bits 0-5 */
	UINT8 gain;
	INT32 timer;	/* CPU cycles until the next tick */
} FDS_ENVELOPE;

typedef struct nesfds_2a03
{
	UINT32 clock_rate;
	UINT32 sample_rate;

	UINT8 io_enable;	/* $4023 */
	UINT8 wave[64];	/* $4040-$407F, 6-bit samples */
	UINT8 mod_table[32];

	UINT16 pitch;	/* $4082/$4083, 12 bits */
	UINT8 wave_halt;	/* $4083.7 */
	UINT8 env_halt;	/* $4083.6 */
	UINT32 wave_acc;	/* 24 bits; the wave position is bits 18-23 */
	UINT8 volume;	/* volume gain as latched at wave position 0 */

	FDS_ENVELOPE vol_env;	/* $4080 */
	FDS_ENVELOPE mod_env;	/* $4084 */
	UINT8 env_speed;	/* $408A */

	UINT16 mod_freq;	/* $4086/$4087, 12 bits */
	UINT8 mod_halt;	/* $4087.7 */
	UINT8 mod_force;	/* $4087.6 */
	UINT32 mod_acc;	/* 18 bits; the mod table address is bits 13-17, bit 12 the "ghost" bit */
	INT32 mod_counter;	/* $4085, 7-bit signed */

	UINT8 write_enable;	/* $4089.7 */
	UINT8 held;	/* whether the wave output is held, which $4089.7 does */
	UINT8 hold_value;
	UINT8 master_volume;	/* $4089.0-1 */

	UINT8 reset_4085;	/* OPT_NES_4085_RESET */
	UINT8 muted;
	float pan[2];

	/* Cycles from the start of the block to the next tick. */
	UINT32 next_tick;
	/* Output level as last handed to the synth. */
	float level;
	/* Output level of one step of wave * volume: 2.4 times a full square at full scale. */
	float scale;

	SYNTH synth;
	LOW_PASS low_pass_2k;
	FILTER_CHAIN filter;
	float* block;
} NESFDS_2A03;

/* "Master volume (0: full; 1: 2/3; 2: 2/4; 3: 2/5)" */
static const float MASTER_VOLUME[4] = {1.0f, 2.0f / 3.0f, 2.0f / 4.0f, 2.0f / 5.0f};


/* ---- Envelopes ---- */

/* CPU cycles per envelope tick: 8 * (e + 1) * (m + 1). */
static INT32 envelope_period(const NESFDS_2A03* fds, const FDS_ENVELOPE* e)
{
	return 8 * ((INT32)e->speed + 1) * ((INT32)fds->env_speed + 1);
}

/* $4080/$4084 */
static void envelope_write(NESFDS_2A03* fds, FDS_ENVELOPE* e, UINT8 val)
{
	e->off = (val & 0x80) ? 1 : 0;
	e->increase = (val & 0x40) ? 1 : 0;
	e->speed = val & 0x3F;
	if (e->off)
		e->gain = val & 0x3F;
	/* A write restarts the envelope's timer. */
	e->timer = envelope_period(fds, e);
}

static UINT8 envelope_running(const NESFDS_2A03* fds, const FDS_ENVELOPE* e)
{
	return ! e->off && ! fds->env_halt && ! fds->wave_halt && fds->env_speed != 0;
}

/* Runs an envelope for `cycles` cycles. */
static void envelope_run(NESFDS_2A03* fds, FDS_ENVELOPE* e, UINT32 cycles)
{
	if (! envelope_running(fds, e))
		return;
	e->timer -= (INT32)cycles;
	while (e->timer <= 0)
	{
		/* Gains set above 32 by hand can still decrease but no longer increase. */
		if (e->increase)
		{
			if (e->gain < 32)
				e->gain ++;
		}
		else if (e->gain > 0)
		{
			e->gain --;
		}
		e->timer += envelope_period(fds, e);
	}
}


/* ---- Wave and mod units ---- */

static UINT8 wave_position(const NESFDS_2A03* fds)
{
	return (UINT8)(fds->wave_acc >> 18);
}

/* The pitch the wave accumulator advances by, the mod counter applied as the wiki gives it. */
static UINT32 modulated_pitch(UINT16 pitch, INT32 counter, UINT8 gain)
{
	/* Unsigned arithmetic keeps the two's complement bits the wiki's C code works on. */
	UINT32 temp = (UINT32)(counter * (INT32)gain);

	/* Round up to 6 bits only if the sign is positive (ignoring bit 4). */
	if ((temp & 0x0F) && ! (temp & 0x800))
		temp += 0x20;
	/* Drop 4 bits and center on 0x40. */
	temp += 0x400;
	temp = (temp >> 4) & 0xFF;
	return ((UINT32)pitch * temp) & 0xFFFFF;
}

/* Applies a mod table entry to the 7-bit signed mod counter, which wraps. */
static void apply_mod_entry(NESFDS_2A03* fds, UINT8 entry)
{
	if (entry == MOD_RESET)
		fds->mod_counter = 0;
	else
		fds->mod_counter = ((fds->mod_counter + MOD_ADJUST[entry] + 64) & 0x7F) - 64;
}

static void mod_tick(NESFDS_2A03* fds)
{
	UINT8 carry;

	/* $4083.7 also halts the mod table accumulator. */
	if (fds->mod_halt || fds->wave_halt)
		return;
	if (fds->mod_force)
	{
		carry = 1;
	}
	else
	{
		UINT32 low;

		if (fds->mod_freq == 0)
			return;
		low = (fds->mod_acc & 0xFFF) + fds->mod_freq;
		carry = (low > 0xFFF) ? 1 : 0;
		fds->mod_acc = (fds->mod_acc & ~(UINT32)0xFFF) | (low & 0xFFF);
	}
	if (carry)
	{
		/* The entry at the current address, then on to the next address bit. Bit 12 makes
			each entry apply twice. */
		apply_mod_entry(fds, fds->mod_table[(fds->mod_acc >> 13) & 0x1F]);
		fds->mod_acc = (fds->mod_acc + 0x1000) & 0x3FFFF;
	}
}

static void wave_tick(NESFDS_2A03* fds)
{
	if (fds->wave_halt)
		return;
	fds->wave_acc = (fds->wave_acc + modulated_pitch(fds->pitch, fds->mod_counter, fds->mod_env.gain)) & 0xFFFFFF;
}

/* Volume gain changes take effect at wave position 0, where the PWM comparator latches. */
static void latch_volume(NESFDS_2A03* fds)
{
	if (wave_position(fds) == 0)
		fds->volume = fds->vol_env.gain;
}

/* The channel's output level, before filtering. */
static float output_level(const NESFDS_2A03* fds)
{
	UINT8 value;
	UINT8 volume;

	if (fds->muted)
		return 0.0f;
	value = fds->held ? fds->hold_value : fds->wave[wave_position(fds)];
	/* The PWM period is 32 cycles, so gains above 32 act as 32. */
	volume = (fds->volume > 32) ? 32 : fds->volume;
	return (float)(value * volume) * MASTER_VOLUME[fds->master_volume] * fds->scale;
}

static void add_level_delta(NESFDS_2A03* fds, UINT32 cycle)
{
	float level = output_level(fds);

	if (level != fds->level)
	{
		synth_add_delta(&fds->synth, cycle, level - fds->level, 0.0f);
		fds->level = level;
	}
}

/* Runs a block of `cycles` cycles. */
static void fds_run(NESFDS_2A03* fds, UINT32 cycles)
{
	/* Register writes since the last block land at its start. */
	add_level_delta(fds, 0);
	while (fds->next_tick < cycles)
	{
		envelope_run(fds, &fds->vol_env, TICK_CYCLES);
		envelope_run(fds, &fds->mod_env, TICK_CYCLES);
		mod_tick(fds);
		wave_tick(fds);
		latch_volume(fds);
		add_level_delta(fds, fds->next_tick);
		fds->next_tick += TICK_CYCLES;
	}
	fds->next_tick -= cycles;
}


void* nesfds_2a03_create(UINT32 clock, UINT32 rate)
{
	NESFDS_2A03* fds;

	if (clock == 0 || rate == 0 || rate > clock / 4)
		return NULL;
	fds = (NESFDS_2A03*)calloc(1, sizeof(NESFDS_2A03));
	if (fds == NULL)
		return NULL;
	fds->clock_rate = clock;
	fds->sample_rate = rate;
	if (! synth_init(&fds->synth, clock, rate, CYCLE_SIZE))
	{
		nesfds_2a03_destroy(fds);
		return NULL;
	}
	fds->block = (float*)calloc(fds->synth.length, sizeof(float));
	if (fds->block == NULL)
	{
		nesfds_2a03_destroy(fds);
		return NULL;
	}
	/* The wiki: "roughly 2.4x the maximum volume of the APU square". A square at volume 15
		reaches 95.52 / (8128 / 15 + 100) on the APU's mixer scale (nes_apu_2a03.c). */
	fds->scale = 2.4f * (95.52f / (8128.0f / 15.0f + 100.0f)) / (63.0f * 32.0f);
	fds->pan[0] = fds->pan[1] = 1.0f;
	nesfds_2a03_reset(fds);
	return fds;
}

void nesfds_2a03_destroy(void* chip)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	synth_free(&fds->synth);
	free(fds->block);
	free(fds);
}

void nesfds_2a03_reset(void* chip)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	memset(fds->wave, 0, sizeof(fds->wave));
	memset(fds->mod_table, 0, sizeof(fds->mod_table));
	fds->pitch = 0;
	fds->wave_halt = 0;
	fds->env_halt = 0;
	fds->wave_acc = 0;
	fds->volume = 0;
	fds->mod_freq = 0;
	fds->mod_halt = 0;
	fds->mod_force = 0;
	fds->mod_acc = 0;
	fds->mod_counter = 0;
	fds->write_enable = 0;
	fds->held = 0;
	fds->hold_value = 0;
	fds->master_volume = 0;
	fds->next_tick = 0;
	fds->level = 0.0f;

	/* VGMs start after the BIOS has run, so the registers it sets start at its values:
		$4023 = $83, $408A = $E8 and $4080 = $80. */
	fds->io_enable = 0x83;
	fds->env_speed = 0xE8;
	envelope_write(fds, &fds->vol_env, 0x80);
	envelope_write(fds, &fds->mod_env, 0x80);

	synth_clear(&fds->synth);
	low_pass_init(&fds->low_pass_2k, (float)fds->sample_rate, 2000.0f);
	filter_init(&fds->filter, (float)fds->sample_rate);
}

void nesfds_2a03_write(void* chip, UINT16 addr, UINT8 data)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	if (addr == 0x4023)
	{
		fds->io_enable = data;
		return;
	}
	/* The sound registers only work with $4023.1 set. */
	if (! (fds->io_enable & 0x02))
		return;

	if (addr >= 0x4040 && addr <= 0x407F)
	{
		if (fds->write_enable)
			fds->wave[addr - 0x4040] = data & 0x3F;
		return;
	}

	switch(addr)
	{
	case 0x4080:
		envelope_write(fds, &fds->vol_env, data);
		/* Muting by writing a gain of 0 takes effect at once. */
		if (fds->vol_env.off && fds->vol_env.gain == 0)
			fds->volume = 0;
		latch_volume(fds);
		break;
	case 0x4082:
		fds->pitch = (UINT16)((fds->pitch & 0xF00) | data);
		break;
	case 0x4083:
		fds->pitch = (UINT16)((fds->pitch & 0x0FF) | ((data & 0x0F) << 8));
		fds->env_halt = (data & 0x40) ? 1 : 0;
		if (fds->env_halt)
		{
			fds->vol_env.timer = envelope_period(fds, &fds->vol_env);
			fds->mod_env.timer = envelope_period(fds, &fds->mod_env);
		}
		fds->wave_halt = (data & 0x80) ? 1 : 0;
		if (fds->wave_halt)
		{
			/* Halting resets the accumulator, so the output sits at $4040's value. */
			fds->wave_acc = 0;
			latch_volume(fds);
		}
		break;
	case 0x4084:
		envelope_write(fds, &fds->mod_env, data);
		break;
	case 0x4085:
		fds->mod_counter = (INT32)(data & 0x7F) - ((data & 0x40) ? 0x80 : 0);
		/* Some emulators reset the mod table phase here, and some rips rely on it. */
		if (fds->reset_4085)
			fds->mod_acc = 0;
		break;
	case 0x4086:
		fds->mod_freq = (UINT16)((fds->mod_freq & 0xF00) | data);
		break;
	case 0x4087:
		fds->mod_freq = (UINT16)((fds->mod_freq & 0x0FF) | ((data & 0x0F) << 8));
		fds->mod_force = (data & 0x40) ? 1 : 0;
		fds->mod_halt = (data & 0x80) ? 1 : 0;
		/* Clears accumulator bits 0-12; the table address in bits 13-17 stays. */
		if (fds->mod_halt)
			fds->mod_acc &= ~(UINT32)0x1FFF;
		break;
	case 0x4088:
		/* Only while the mod unit is halted: replaces the entry at the current position and
			moves to the next one. */
		if (fds->mod_halt)
		{
			fds->mod_table[(fds->mod_acc >> 13) & 0x1F] = data & 0x07;
			fds->mod_acc = (fds->mod_acc + 0x2000) & 0x3FFFF;
		}
		break;
	case 0x4089:
		fds->write_enable = (data & 0x80) ? 1 : 0;
		/* Write enable also holds the output at its current level while the wave unit runs on. */
		if (fds->write_enable && ! fds->held)
			fds->hold_value = fds->wave[wave_position(fds)];
		fds->held = fds->write_enable;
		fds->master_volume = data & 0x03;
		break;
	case 0x408A:
		fds->env_speed = data;
		break;
	}
}

UINT8 nesfds_2a03_read(void* chip, UINT16 addr)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	/* Bits 6-7 of most reads come from open bus, which reads as 01. */
	if (addr >= 0x4040 && addr <= 0x407F)
		return (UINT8)(0x40 | (fds->write_enable ? fds->wave[addr - 0x4040] : fds->wave[wave_position(fds)]));
	switch(addr)
	{
	case 0x4090:
		return (UINT8)(0x40 | fds->vol_env.gain);
	case 0x4091:
		return (UINT8)((fds->wave_acc >> 12) & 0xFF);
	case 0x4092:
		return (UINT8)(0x40 | fds->mod_env.gain);
	case 0x4093:
		return (UINT8)((fds->mod_acc >> 5) & 0x7F);
	case 0x4097:
		return (UINT8)(fds->mod_counter & 0x7F);
	}
	return 0x00;
}

void nesfds_2a03_set_options(void* chip, UINT32 flags)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	fds->reset_4085 = (flags & OPT_NES_4085_RESET) ? 1 : 0;
}

void nesfds_2a03_set_mute(void* chip, UINT8 mute)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;

	fds->muted = mute;
}

void nesfds_2a03_set_panning(void* chip, INT16 pan)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;
	INT32 gains[2];

	Panning_Calculate(gains, pan);
	fds->pan[0] = (float)gains[0] / PANNING_NORMAL;
	fds->pan[1] = (float)gains[1] / PANNING_NORMAL;
}

void nesfds_2a03_update(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	NESFDS_2A03* fds = (NESFDS_2A03*)chip;
	UINT32 done = 0;

	while (done < samples)
	{
		SYNTH* s = &fds->synth;
		INT64 needed = ((INT64)(samples - done) << SYNTH_FRAC_BITS) - s->offset;
		UINT32 cycles;
		UINT32 count;
		UINT32 i;

		/* The fewest cycles that complete the remaining samples, at most one block. */
		if (needed > (INT64)CYCLE_SIZE * s->samples_per_cycle)
			cycles = CYCLE_SIZE;
		else
			cycles = (UINT32)((needed + s->samples_per_cycle - 1) / s->samples_per_cycle);
		fds_run(fds, cycles);
		count = synth_end_block(s, cycles);
		if (count > samples - done)
			count = samples - done;

		synth_read(s, 0, count, fds->block);
		for (i = 0; i < count; i ++)
		{
			/* The FDS's own ~2 kHz low-pass, then the console's output filters. */
			float y = filter_consume(&fds->filter, low_pass_consume(&fds->low_pass_2k, fds->block[i])) * 0x8000;

			outputs[0][done + i] += (DEV_SMPL)(y * fds->pan[0]);
			outputs[1][done + i] += (DEV_SMPL)(y * fds->pan[1]);
		}
		synth_consume(s, count);
		done += count;
	}
}
