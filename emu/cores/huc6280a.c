/*
	The MIT License (MIT)

	Copyright (c) 2026 Purpbatboi

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../EmuCores.h"
#include "../EmuHelper.h"
#include "../RatioCntr.h"
#include "huc6280a.h"

#define REG_SELECT	0
#define REG_MASTER_BALANCE	1
#define REG_FREQ_LO	2
#define REG_FREQ_HI	3
#define REG_CONTROL	4
#define REG_BALANCE	5
#define REG_DATA	6
#define REG_NOISE	7
#define REG_LFO_FREQ	8
#define REG_LFO_CTRL	9

#define CTRL_ENABLE	0x80
#define CTRL_DDA	0x40
/* LFO control bit 7: holds channel 1 and restarts its wave. */
#define LFO_TRIGGER	0x80
/* Channels 4 and 5 have a noise generator. */
#define FIRST_NOISE_CHN	4

#define LEFT	0
#define RIGHT	1

/* libvgm device: output samples rendered per call into the core. */
#define BLOCK_SAMPLES	256

/* Output modes (§6). */
#define OUT_OFF	0
#define OUT_NOISE	1
#define OUT_AVERAGE	2
#define OUT_WAVE	3

typedef struct
{
	UINT16 freq;	/* 12 bits */
	UINT8 control;	/* enable, DDA, volume */
	UINT8 balance;
	UINT8 noise;	/* channels 4-5 only */
	UINT8 wave_pos;	/* wave RAM read/write position, 5 bits */
	UINT8 sample;	/* current 5-bit sample */
	UINT8 wave[32];
	UINT16 wave_sum;	/* sum of wave[], 0-992 */
	INT32 step_period;	/* clocks per wave step, recomputed at fixed points */
	INT32 step_counter;	/* clocks until the next wave step */
	UINT8 atten[2];	/* applied attenuation, left/right, 0-31 */
	INT16 level[2];	/* last reported level, left/right */
	UINT32 lfsr;	/* channels 4-5 only, 18 bits */
	INT32 noise_counter;	/* clocks until the next LFSR clock */
} PSG_CHANNEL;

typedef struct
{
	DEV_DATA _devData;	/* libvgm's, which must come first */
	/* libvgm device: PSG clocks per output sample, and the muted channels (bit 0-5). */
	RATIO_CNTR sample_clock;
	UINT8 mute_mask;

	/* Everything from here on is chip state, which a reset clears. */
	UINT64 clock;
	UINT8 select;
	UINT8 master_balance;
	UINT8 lfo_freq;
	UINT8 lfo_ctrl;

	/* Volume sequencer (§11.3) */
	UINT8 seq_active;
	UINT8 seq_pending;
	UINT8 seq_slot;	/* 0-31 */
	UINT8 seq_atten;	/* attenuation read in the last read slot */
	INT32 seq_countdown;	/* clocks until the next slot */

	PSG_CHANNEL ch[HUC6280A_CHANNELS];

	/* Where level changes go, for the length of a huc6280a_run call. */
	HUC6280A_SINK sink;
	void* user;

	/* libvgm device: each channel's current level. */
	INT16 out_level[HUC6280A_CHANNELS][2];
	/* For each sample of the block being rendered: the clock it ends on; the level changes
		within it, each weighted by the clocks from the change to the sample's end; and the sum
		of those changes, which carries into every later sample. */
	UINT64 sample_end[BLOCK_SAMPLES];
	INT64 sample_partial[BLOCK_SAMPLES][2];
	INT32 sample_change[BLOCK_SAMPLES][2];
	UINT32 block_samples;
} HUC6280A;


/* ---- Levels (§11) ---- */

/* §11.1: balance level (0-15) to the value subtracted from 31. */
static const UINT8 BALANCE_ATTEN[16] =
{
	0x00, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x10, 0x13, 0x15, 0x17, 0x19, 0x1B, 0x1D, 0x1F,
};

/* The amplitude scale (8/6) / 2^(atten/4) is ATTEN_SCALE[atten % 4] / (3 * 2^(20 + atten / 4)),
	with ATTEN_SCALE[f] = round(4 * 2^(-f/4) * 2^20). */
static const INT32 ATTEN_SCALE[4] = {4194304, 3526975, 2965821, 2493948};

/* trunc(x / (3 * 2^shift)), as trunc(trunc(x / 3) / 2^shift): the same value, with a constant
	divisor and a shift. Works on the magnitude, as C90 leaves the rounding of a negative
	quotient and the right shift of a negative value to the compiler. */
static INT32 div3_shift(INT32 x, UINT8 shift)
{
	INT32 magnitude = ((x < 0) ? -x : x) / 3 >> shift;

	return (x < 0) ? -magnitude : magnitude;
}

/* Floor division by 8192, again on non-negative operands only. */
static INT32 floor_div_8192(INT32 x)
{
	return (x >= 0) ? x / 8192 : -((-x + 8191) / 8192);
}

/* §11.4: the level of a 5-bit sample at attenuation `atten` (0-31),
	trunc(scale * (2 * sample - 31) * 128), and the averaging-mode gain trunc(scale * 65536),
	with scale = (8/6) / 2^(atten/4) and 0 at attenuation 31. These integer forms equal the
	definitions for every input. */
static INT16 sample_level(UINT8 atten, UINT8 sample)
{
	if (atten >= 31)
		return 0;
	return (INT16)div3_shift(ATTEN_SCALE[atten & 0x03] * (2 * (INT32)sample - 31),
		(UINT8)(13 + (atten >> 2)));
}

static INT32 average_gain(UINT8 atten)
{
	if (atten >= 31)
		return 0;
	return div3_shift(ATTEN_SCALE[atten & 0x03], (UINT8)(4 + (atten >> 2)));
}

/* §11.2: the attenuation the registers currently ask for on one side. */
static UINT8 target_atten(const HUC6280A* psg, UINT8 chn, UINT8 side)
{
	UINT8 shift = (side == LEFT) ? 4 : 0;
	UINT8 master = (UINT8)(31 - BALANCE_ATTEN[(psg->master_balance >> shift) & 0x0F]);
	UINT8 channel = (UINT8)(31 - BALANCE_ATTEN[(psg->ch[chn].balance >> shift) & 0x0F]);
	UINT8 volume = (UINT8)(31 - (psg->ch[chn].control & 0x1F));
	UINT8 atten = (UINT8)(master + channel + volume);

	return (atten < 31) ? atten : 31;
}


/* ---- Channel state ---- */

/* 0: LFO off; 1-3: channel 1 modulates channel 0, scaled by 1, 4 or 16. */
static UINT8 lfo_mode(const HUC6280A* psg)
{
	return psg->lfo_ctrl & 0x03;
}

/* §10.5: the LFO is modulating channel 0 as the wave steps. */
static UINT8 lfo_modulating(const HUC6280A* psg)
{
	return lfo_mode(psg) && ! (psg->lfo_ctrl & LFO_TRIGGER) && (psg->ch[1].control & CTRL_ENABLE);
}

/* §5.1, §10.2, §10.3: recomputes a channel's step period. Called from register writes and,
	while the LFO is on, at each channel 0 step, so a change of channel 1's sample or of the LFO
	speed takes effect only at those points. */
static void update_step_period(HUC6280A* psg, UINT8 chn)
{
	UINT8 mode = lfo_mode(psg);
	INT32 freq = psg->ch[chn].freq;
	INT32 period;

	if (chn == 0 && mode)
	{
		/* Channel 1's sample, centered on 0, offsets the frequency, which wraps at 12 bits. The
			offset is at least -256, so adding 0x1000 keeps the sum positive for the mask. */
		INT32 offset = ((INT32)psg->ch[1].sample - 16) * (1 << ((mode - 1) * 2));

		freq = (freq + offset + 0x1000) & 0xFFF;
	}
	period = (freq ? freq : 4096) * 2;
	if (chn == 1 && mode)
		period *= psg->lfo_freq ? psg->lfo_freq : 256;
	psg->ch[chn].step_period = period;
}

/* §6: the first match wins. */
static UINT8 output_mode(const HUC6280A* psg, UINT8 chn)
{
	const PSG_CHANNEL* ch = &psg->ch[chn];
	UINT8 on = ch->control & (CTRL_ENABLE | CTRL_DDA);

	if (! on)
		return OUT_OFF;
	if (chn >= FIRST_NOISE_CHN && (ch->noise & 0x80) && on == CTRL_ENABLE)
		return OUT_NOISE;
	if (on == CTRL_ENABLE && ch->step_period <= 7 && ! (chn == 1 && (psg->lfo_ctrl & LFO_TRIGGER)))
		return OUT_AVERAGE;
	return OUT_WAVE;
}

/* §5.2: whether the wave counter runs. */
static UINT8 wave_counter_runs(const HUC6280A* psg, UINT8 chn)
{
	UINT8 control = psg->ch[chn].control;

	return (control & CTRL_ENABLE) && ! (control & CTRL_DDA) &&
		! (chn == 1 && (psg->lfo_ctrl & LFO_TRIGGER));
}

/* The channel's current level on each side (§6, §7, §9.3, §11.4). */
static void channel_levels(const HUC6280A* psg, UINT8 chn, INT16* level)
{
	const PSG_CHANNEL* ch = &psg->ch[chn];
	UINT8 mode = output_mode(psg, chn);
	UINT8 side;

	for (side = 0; side < 2; side ++)
	{
		UINT8 atten = ch->atten[side];

		switch(mode)
		{
		case OUT_OFF:
			level[side] = 0;
			break;
		case OUT_NOISE:
			level[side] = sample_level(atten, (ch->lfsr & 0x01) ? 31 : 0);
			break;
		case OUT_AVERAGE:
			level[side] = (INT16)floor_div_8192(average_gain(atten) * ((INT32)ch->wave_sum - 496));
			break;
		default:
			level[side] = sample_level(atten, ch->sample);
			break;
		}
	}
}

/* §12: reports the channel's level at `clock` if it changed. */
static void report_level(HUC6280A* psg, UINT8 chn, UINT64 clock)
{
	PSG_CHANNEL* ch = &psg->ch[chn];
	INT16 level[2];
	HUC6280A_EVENT event;

	channel_levels(psg, chn, level);
	if (level[LEFT] == ch->level[LEFT] && level[RIGHT] == ch->level[RIGHT])
		return;
	ch->level[LEFT] = level[LEFT];
	ch->level[RIGHT] = level[RIGHT];
	if (psg->sink == NULL)
		return;
	event.clock = clock;
	event.channel = chn;
	event.left = level[LEFT];
	event.right = level[RIGHT];
	psg->sink(psg->user, &event);
}


/* ---- Wave and noise ---- */

/* §3.4 rule 1, §3.9: back to wave position 0 with its sample, and a full step to the next. */
static void restart_wave(PSG_CHANNEL* ch)
{
	ch->wave_pos = 0;
	ch->sample = ch->wave[0];
	ch->step_counter = ch->step_period;
}

/* One wave step at `clock` (§5.2), reported in the output mode in effect before the step. For
	channel 0 with the LFO on, the step then recomputes the period from channel 1's current
	sample (§10.4, §10.5), which may change the mode; the new mode's level first appears at the
	next step or span start. The next step is at least `min_period` clocks away. */
static void wave_step(HUC6280A* psg, UINT8 chn, UINT64 clock, INT32 min_period)
{
	PSG_CHANNEL* ch = &psg->ch[chn];

	ch->wave_pos = (UINT8)((ch->wave_pos + 1) & 0x1F);
	ch->sample = ch->wave[ch->wave_pos];
	report_level(psg, chn, clock);
	if (chn == 0 && lfo_mode(psg))
		update_step_period(psg, 0);
	ch->step_counter = (ch->step_period > min_period) ? ch->step_period : min_period;
}

/* Runs one channel's wave counter for `clocks` clocks from `start` (§5.2). */
static void run_wave(HUC6280A* psg, UINT8 chn, UINT64 start, INT64 clocks)
{
	PSG_CHANNEL* ch = &psg->ch[chn];
	INT64 elapsed = 0;

	if (! wave_counter_runs(psg, chn))
		return;

	/* A channel that reports nothing per step and can't change its period within the span
		takes all its steps at once, with the same result as taking them one at a time. */
	if (output_mode(psg, chn) != OUT_WAVE && ! (chn == 0 && lfo_mode(psg)))
	{
		INT64 steps;

		if (ch->step_counter > clocks)
		{
			ch->step_counter = (INT32)(ch->step_counter - clocks);
			return;
		}
		steps = (clocks - ch->step_counter) / ch->step_period + 1;
		ch->wave_pos = (UINT8)((ch->wave_pos + steps) & 0x1F);
		ch->sample = ch->wave[ch->wave_pos];
		ch->step_counter = (INT32)(ch->step_counter + steps * ch->step_period - clocks);
		return;
	}

	while (ch->step_counter <= clocks - elapsed)
	{
		elapsed += ch->step_counter;
		wave_step(psg, chn, start + (UINT64)elapsed, 0);
	}
	ch->step_counter = (INT32)(ch->step_counter - (clocks - elapsed));
}

/* §10.5: runs channels 0 and 1 together. At each channel 0 step, channel 1 first runs to that
	clock, then channel 0's period is recomputed from channel 1's new sample and its counter
	reloaded with at least 7 clocks. */
static void run_lfo_pair(HUC6280A* psg, UINT64 start, INT64 clocks)
{
	PSG_CHANNEL* carrier = &psg->ch[0];
	INT64 elapsed = 0;	/* both channels have run to start + elapsed */

	if (wave_counter_runs(psg, 0))
	{
		while (carrier->step_counter <= clocks - elapsed)
		{
			INT64 step = carrier->step_counter;

			run_wave(psg, 1, start + (UINT64)elapsed, step);
			elapsed += step;
			wave_step(psg, 0, start + (UINT64)elapsed, 7);
		}
		carrier->step_counter = (INT32)(carrier->step_counter - (clocks - elapsed));
	}
	run_wave(psg, 1, start + (UINT64)elapsed, clocks - elapsed);
}

/* §9.2 */
static INT32 noise_period(const PSG_CHANNEL* ch)
{
	INT32 divider = 31 - (INT32)(ch->noise & 0x1F);

	return divider ? divider * 128 : 64;
}

/* §9.1 */
static void lfsr_step(PSG_CHANNEL* ch)
{
	UINT32 v = ch->lfsr;
	UINT32 feedback = (v ^ (v >> 1) ^ (v >> 11) ^ (v >> 12) ^ (v >> 17)) & 0x01;

	ch->lfsr = (v >> 1) | (feedback << 17);
}

/* Runs every channel and the LFSRs for `clocks` clocks from `start`. */
static void run_span(HUC6280A* psg, UINT64 start, INT64 clocks)
{
	UINT8 first_wave = 0;
	UINT8 chn;

	/* Level changes since the last span (writes, the volume sequencer) land here. */
	for (chn = 0; chn < HUC6280A_CHANNELS; chn ++)
		report_level(psg, chn, start);
	if (lfo_modulating(psg))
	{
		run_lfo_pair(psg, start, clocks);
		first_wave = 2;
	}
	for (chn = first_wave; chn < HUC6280A_CHANNELS; chn ++)
		run_wave(psg, chn, start, clocks);

	/* §9.2: the LFSR runs freely on channels 4 and 5; in noise mode each LFSR clock is reported
		at its exact time (§9.3). */
	for (chn = FIRST_NOISE_CHN; chn < HUC6280A_CHANNELS; chn ++)
	{
		PSG_CHANNEL* ch = &psg->ch[chn];
		UINT8 noisy = (output_mode(psg, chn) == OUT_NOISE);
		INT64 next = ch->noise_counter;

		while (next <= clocks)
		{
			lfsr_step(ch);
			if (noisy)
				report_level(psg, chn, start + (UINT64)next);
			next += noise_period(ch);
		}
		ch->noise_counter = (INT32)(next - clocks);
	}
}

/* §11.3: one volume sequencer slot. An applied attenuation is reported by the span that starts
	at this clock (§12). */
static void sequencer_slot(HUC6280A* psg)
{
	UINT8 slot = psg->seq_slot;
	UINT8 side = ((slot >> 1) & 0x01) ? LEFT : RIGHT;
	UINT8 chn = slot >> 2;

	if (! (slot & 0x01))
	{
		if (chn < HUC6280A_CHANNELS)
			psg->seq_atten = target_atten(psg, chn, side);
		psg->seq_countdown = 255;
	}
	else
	{
		if (chn < HUC6280A_CHANNELS)
			psg->ch[chn].atten[side] = psg->seq_atten;
		psg->seq_countdown = 1;
	}

	psg->seq_slot = (UINT8)((slot + 1) & 0x1F);
	if (psg->seq_slot == 0)
	{
		if (psg->seq_pending)
			psg->seq_pending = 0;
		else
			psg->seq_active = 0;
	}
}


/* ---- Register writes ---- */

/* §3.4 */
static void write_control(PSG_CHANNEL* ch, UINT8 data)
{
	UINT8 old = ch->control;

	if ((old & CTRL_DDA) && ! (data & CTRL_DDA))
		restart_wave(ch);
	if (! (old & CTRL_ENABLE) && (data & CTRL_ENABLE) && ! (data & CTRL_DDA))
	{
		ch->wave_pos = (UINT8)((ch->wave_pos + 1) & 0x1F);
		ch->sample = ch->wave[ch->wave_pos];
	}
	ch->control = data;
}

/* §3.6 */
static void write_data(PSG_CHANNEL* ch, UINT8 data)
{
	UINT8 sample = data & 0x1F;

	if (! (ch->control & CTRL_DDA))
	{
		ch->wave_sum = (UINT16)(ch->wave_sum - ch->wave[ch->wave_pos] + sample);
		ch->wave[ch->wave_pos] = sample;
		if (! (ch->control & CTRL_ENABLE))
			ch->wave_pos = (UINT8)((ch->wave_pos + 1) & 0x1F);
	}
	if (ch->control & CTRL_ENABLE)
		ch->sample = sample;
}

/* §10.4: after a write to channel 1, channel 0 follows channel 1's sample at once, unless the LFO
	is modulating it: then channel 0 recomputes only at its own wave steps. */
static void update_carrier_after_write(HUC6280A* psg)
{
	if (psg->select == 1 && ! lfo_modulating(psg))
		update_step_period(psg, 0);
}

/* §10.4: a frequency or control write recomputes the selected channel. */
static void update_after_write(HUC6280A* psg)
{
	update_step_period(psg, psg->select);
	update_carrier_after_write(psg);
}


/* ---- Interface ---- */

void* huc6280a_create(void)
{
	HUC6280A* psg = (HUC6280A*)calloc(1, sizeof(HUC6280A));

	if (psg == NULL)
		return NULL;
	huc6280a_reset(psg);
	return psg;
}

void huc6280a_destroy(void* chip)
{
	free(chip);
}

void huc6280a_reset(void* chip)
{
	HUC6280A* psg = (HUC6280A*)chip;
	UINT8 chn;

	/* §4.2 */
	memset(&psg->clock, 0, sizeof(HUC6280A) - offsetof(HUC6280A, clock));
	RC_RESET(&psg->sample_clock);
	for (chn = 0; chn < HUC6280A_CHANNELS; chn ++)
	{
		PSG_CHANNEL* ch = &psg->ch[chn];

		update_step_period(psg, chn);
		ch->step_counter = ch->step_period;
		ch->atten[LEFT] = 31;
		ch->atten[RIGHT] = 31;
		if (chn >= FIRST_NOISE_CHN)
		{
			ch->lfsr = 1;
			ch->noise_counter = 1;
		}
	}
}

void huc6280a_write(void* chip, UINT8 reg, UINT8 data)
{
	HUC6280A* psg = (HUC6280A*)chip;
	PSG_CHANNEL* ch;

	switch(reg)
	{
	case REG_SELECT:
		psg->select = data & 0x07;
		return;
	case REG_MASTER_BALANCE:
		psg->master_balance = data;
		psg->seq_pending = 1;
		return;
	case REG_LFO_FREQ:
		psg->lfo_freq = data;
		return;
	case REG_LFO_CTRL:
		psg->lfo_ctrl = data & 0x83;
		if (data & LFO_TRIGGER)
			restart_wave(&psg->ch[1]);
		update_step_period(psg, 0);
		update_step_period(psg, 1);
		return;
	}

	/* Registers 2-7 act on the selected channel. Selecting 6 or 7 makes them do nothing, and
		registers 10-15 are reserved (§2, §3). */
	if (psg->select >= HUC6280A_CHANNELS)
		return;
	ch = &psg->ch[psg->select];
	switch(reg)
	{
	case REG_FREQ_LO:
		ch->freq = (UINT16)((ch->freq & 0xF00) | data);
		update_after_write(psg);
		break;
	case REG_FREQ_HI:
		ch->freq = (UINT16)((ch->freq & 0x0FF) | ((data & 0x0F) << 8));
		update_after_write(psg);
		break;
	case REG_CONTROL:
		write_control(ch, data);
		update_after_write(psg);
		psg->seq_pending = 1;
		break;
	case REG_BALANCE:
		ch->balance = data;
		psg->seq_pending = 1;
		break;
	case REG_DATA:
		write_data(ch, data);
		update_carrier_after_write(psg);
		break;
	case REG_NOISE:
		if (psg->select >= FIRST_NOISE_CHN)
			ch->noise = data;
		break;
	}
}

void huc6280a_run(void* chip, UINT32 clocks, HUC6280A_SINK sink, void* user)
{
	HUC6280A* psg = (HUC6280A*)chip;
	INT64 remaining = clocks;

	psg->sink = sink;
	psg->user = user;
	if (! psg->seq_active && psg->seq_pending)
	{
		psg->seq_active = 1;
		psg->seq_pending = 0;
		psg->seq_slot = 0;
		psg->seq_countdown = 1;
	}

	/* With no time passing, pending level changes aren't reported yet; the next run that
		advances time reports them at its first clock (§12). */
	if (clocks == 0)
		return;

	/* Split the span at sequencer slots, so attenuation changes land at their exact clocks
		between wave steps. */
	for (;;)
	{
		INT64 span = remaining;
		UINT8 slot_due = psg->seq_active && psg->seq_countdown <= remaining;

		if (slot_due)
			span = psg->seq_countdown;
		run_span(psg, psg->clock, span);
		psg->clock += (UINT64)span;
		remaining -= span;
		if (! slot_due)
		{
			if (psg->seq_active)
				psg->seq_countdown = (INT32)(psg->seq_countdown - span);
			break;
		}
		sequencer_slot(psg);
		if (remaining == 0)
			break;	/* the next run reports from this clock */
	}
}


/* ---- libvgm device ----
	Like libvgm's other C6280 cores (MAME, Ootake), it renders at clock / 16 unless asked for
	another rate, and leaves the conversion to the output rate to libvgm's resampler. Each sample
	is the exact average of the level over its span of PSG clocks. */

/* Records a channel's change of level in the sample it happens in. A change on the block's last
	clock belongs to the next block, which starts from the channels' levels. */
static void accumulate_level(void* user, const HUC6280A_EVENT* event)
{
	HUC6280A* psg = (HUC6280A*)user;
	UINT8 chn = event->channel;

	if (! ((psg->mute_mask >> chn) & 0x01))
	{
		/* The first sample ending after the change. */
		UINT32 lo = 0;
		UINT32 hi = psg->block_samples;

		while (lo < hi)
		{
			UINT32 mid = (lo + hi) / 2;

			if (psg->sample_end[mid] > event->clock)
				hi = mid;
			else
				lo = mid + 1;
		}
		if (lo < psg->block_samples)
		{
			INT32 left = event->left - psg->out_level[chn][LEFT];
			INT32 right = event->right - psg->out_level[chn][RIGHT];
			INT64 weight = (INT64)(psg->sample_end[lo] - event->clock);

			psg->sample_partial[lo][LEFT] += left * weight;
			psg->sample_partial[lo][RIGHT] += right * weight;
			psg->sample_change[lo][LEFT] += left;
			psg->sample_change[lo][RIGHT] += right;
		}
	}
	psg->out_level[chn][LEFT] = event->left;
	psg->out_level[chn][RIGHT] = event->right;
}

/* sum / clocks rounded toward zero, on the magnitude as C90 leaves negative division to the
	compiler. */
static DEV_SMPL average_level(INT64 sum, UINT64 clocks)
{
	INT64 magnitude = ((sum < 0) ? -sum : sum) / (INT64)clocks;

	return (DEV_SMPL)((sum < 0) ? -magnitude : magnitude);
}

static void device_update(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	HUC6280A* psg = (HUC6280A*)chip;
	UINT32 done = 0;

	/* The core runs a block of samples per call, as a call per sample costs more than the
		emulation. */
	while (done < samples)
	{
		UINT32 count = (samples - done < BLOCK_SAMPLES) ? samples - done : BLOCK_SAMPLES;
		UINT64 start = psg->clock;
		UINT64 end = start;
		INT32 level[2] = {0, 0};
		UINT32 i;
		UINT8 chn;

		/* The level the block starts at. */
		for (chn = 0; chn < HUC6280A_CHANNELS; chn ++)
		{
			if (! ((psg->mute_mask >> chn) & 0x01))
			{
				level[LEFT] += psg->out_level[chn][LEFT];
				level[RIGHT] += psg->out_level[chn][RIGHT];
			}
		}
		for (i = 0; i < count; i ++)
		{
			RC_STEP(&psg->sample_clock);
			end += RC_GET_VAL(&psg->sample_clock);
			RC_MASK(&psg->sample_clock);
			psg->sample_end[i] = end;
			psg->sample_partial[i][LEFT] = psg->sample_partial[i][RIGHT] = 0;
			psg->sample_change[i][LEFT] = psg->sample_change[i][RIGHT] = 0;
		}
		psg->block_samples = count;

		huc6280a_run(psg, (UINT32)(end - start), accumulate_level, psg);

		/* Each sample sums the level it started at over its clocks, plus the changes within it. */
		for (i = 0; i < count; i ++)
		{
			UINT64 clocks = psg->sample_end[i] - (i ? psg->sample_end[i - 1] : start);

			outputs[LEFT][done + i] = average_level(
				level[LEFT] * (INT64)clocks + psg->sample_partial[i][LEFT], clocks);
			outputs[RIGHT][done + i] = average_level(
				level[RIGHT] * (INT64)clocks + psg->sample_partial[i][RIGHT], clocks);
			level[LEFT] += psg->sample_change[i][LEFT];
			level[RIGHT] += psg->sample_change[i][RIGHT];
		}
		done += count;
	}
}

static void device_set_mute_mask(void* chip, UINT32 mute_mask)
{
	HUC6280A* psg = (HUC6280A*)chip;

	psg->mute_mask = (UINT8)(mute_mask & 0x3F);
}

static UINT8 device_start(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, huc6280a_write},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, device_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};

DEV_DEF devDef_C6280_HUCA =
{
	"HuC6280", "huc6280a", FCC_HUCA,
	device_start,
	huc6280a_destroy,
	huc6280a_reset,
	device_update,
	NULL,	/* SetOptionBits */
	device_set_mute_mask,
	NULL,	/* SetPanning */
	NULL,	/* SetSampleRateChangeCallback */
	NULL,	/* SetLoggingCallback */
	NULL,	/* LinkDevice */
	devFunc,	/* rwFuncs */
};

static UINT8 device_start(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	HUC6280A* psg;
	UINT32 rate = cfg->clock / 16;

	SRATE_CUSTOM_HIGHEST(cfg->srMode, rate, cfg->smplRate);
	/* At least one PSG clock per sample. */
	if (rate > cfg->clock * 2)
		rate = cfg->clock * 2;

	psg = (HUC6280A*)huc6280a_create();
	if (psg == NULL)
		return 0xFF;
	/* The PSG counts at twice the clock VGM files give the chip (see huc6280a.h). */
	RC_SET_RATIO(&psg->sample_clock, cfg->clock * 2, rate);
	psg->_devData.chipInf = psg;
	INIT_DEVINF(retDevInf, &psg->_devData, rate, &devDef_C6280_HUCA);
	return 0x00;
}
