/*
	MIT License

	Copyright (c) 2021 Luke Petherbridge <me@lukeworks.tech>

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
#include "nes_apu_teta.h"
#include "nes_synth_teta.h"

/* How many CPU cycles a mixing block spans at most before every cycle counter rolls back to zero. */
#define CYCLE_SIZE	10000
#define NO_CHANGE	0xFFFFFFFFU

#define CHN_PULSE1	0
#define CHN_PULSE2	1
#define CHN_TRIANGLE	2
#define CHN_NOISE	3
#define CHN_DMC	4
#define CHN_COUNT	5


/* ---- Timer ----
	A divider clocked every (period + 1) cycles. */

typedef struct
{
	UINT32 cycle;
	UINT16 counter;
	UINT16 period;
} TIMER;

/* The cycle this timer's next expiry falls on. */
static UINT32 timer_next_expiry(const TIMER* t)
{
	return t->cycle + (UINT32)t->counter + 1;
}

/* Advances toward `target`, stopping at each expiry. Returns 1 having advanced to the cycle the
	divider expired on, so a caller loops `while (timer_run_to(t, target))`; the 0 that ends the
	loop leaves the timer at `target`. This is a per-cycle divider with the cycles in between collapsed
	into one subtraction, so a 10,000-cycle block takes one call per expiry. */
static UINT8 timer_run_to(TIMER* t, UINT32 target)
{
	/* A timer can sit ahead of the target (the DMC's is parked a cycle forward by a reset),
		and advancing to a target already passed must do nothing. */
	if (target <= t->cycle)
		return 0;
	/* counter + 1 cycles to expiry: the counter is decremented on every cycle but the one it
		reads zero on, which is the one that fires. */
	if (target >= timer_next_expiry(t))
	{
		t->cycle = timer_next_expiry(t);
		t->counter = t->period;
		return 1;
	}
	t->counter = (UINT16)(t->counter - (target - t->cycle));
	t->cycle = target;
	return 0;
}

/* Lowers `next` to the timer's next expiry. */
static UINT32 timer_earliest(UINT32 next, const TIMER* t)
{
	UINT32 cycle = timer_next_expiry(t);

	return (cycle < next) ? cycle : next;
}


/* ---- Length counter ---- */

static const UINT8 LENGTH_TABLE[32] =
{
	10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
	12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
};

typedef struct
{
	UINT8 enabled;
	UINT8 halt;
	UINT8 new_halt;
	UINT8 counter;
	UINT8 previous_counter;
	UINT8 reload;
} LENGTH_COUNTER;

/* $4003/$4007/$400B/$400F: the load is taken from bits 3-7. */
static void length_write(LENGTH_COUNTER* l, UINT8 val)
{
	if (l->enabled)
	{
		l->reload = LENGTH_TABLE[val >> 3];
		l->previous_counter = l->counter;
	}
}

static void length_set_enabled(LENGTH_COUNTER* l, UINT8 enabled)
{
	if (! enabled)
		l->counter = 0;
	l->enabled = enabled;
}

/* Applies a pending length load and halt flag. A load is dropped if the frame counter clocked
	the counter in the same cycle, which is what comparing with previous_counter detects. */
static void length_reload(LENGTH_COUNTER* l)
{
	if (l->reload > 0)
	{
		if (l->counter == l->previous_counter)
			l->counter = l->reload;
		l->reload = 0;
	}
	l->halt = l->new_halt;
}

static void length_clock(LENGTH_COUNTER* l)
{
	if (l->counter > 0 && ! l->halt)
		l->counter --;
}


/* ---- Envelope ---- */

typedef struct
{
	UINT8 start;
	UINT8 constant_volume;
	UINT8 volume;
	UINT8 divider;
	UINT8 counter;
	UINT8 loops;
} ENVELOPE;

/* $4000/$4004/$400C: bit 5 both loops the envelope and halts the length counter. */
static void envelope_write_ctrl(ENVELOPE* e, LENGTH_COUNTER* l, UINT8 val)
{
	l->new_halt = (val & 0x20) ? 1 : 0;
	e->loops = l->new_halt;
	e->constant_volume = (val & 0x10) ? 1 : 0;
	e->volume = val & 0x0F;
}

static void envelope_clock(ENVELOPE* e)
{
	if (e->start)
	{
		e->start = 0;
		e->counter = 15;
		e->divider = e->volume;
	}
	else if (e->divider > 0)
	{
		e->divider --;
	}
	else
	{
		e->divider = e->volume;
		if (e->counter > 0)
			e->counter --;
		else if (e->loops)
			e->counter = 15;
	}
}

/* The loop flag survives a reset, as in TetaNES. */
static void envelope_reset(ENVELOPE* e)
{
	e->start = e->constant_volume = e->volume = e->divider = e->counter = 0;
}

/* The volume of a pulse or the noise, 0-15: the envelope's, until the length counter runs out. */
static UINT8 envelope_volume(const ENVELOPE* e, const LENGTH_COUNTER* l)
{
	if (l->counter == 0)
		return 0;
	return e->constant_volume ? e->volume : e->counter;
}


/* ---- Pulse ---- */

/* Bit n is step n of each duty sequence. */
static const UINT8 DUTY_TABLE[4] = {0x80, 0xC0, 0xF0, 0x3F};

typedef struct
{
	UINT8 enabled;
	UINT8 negate;
	UINT8 reload;
	UINT8 shift;
	UINT8 divider;
	UINT8 period;
} SWEEP;

typedef struct
{
	UINT8 one;	/* pulse 1 negates in ones' complement */
	UINT16 real_period;
	TIMER timer;
	UINT8 duty;	/* entry in DUTY_TABLE */
	UINT8 duty_cycle;	/* step of the duty sequence */
	LENGTH_COUNTER length;
	ENVELOPE envelope;
	SWEEP sweep;
} PULSE;

static UINT16 pulse_target_period(const PULSE* p)
{
	UINT16 delta = (UINT16)(p->real_period >> p->sweep.shift);

	if (p->sweep.negate)
		return (UINT16)(p->real_period - delta - p->one);
	return (UINT16)(p->real_period + delta);
}

static UINT8 pulse_is_muted(const PULSE* p)
{
	return p->real_period < 8 || (! p->sweep.negate && pulse_target_period(p) > 0x7FF);
}

static void pulse_set_period(PULSE* p, UINT16 period)
{
	p->real_period = period;
	p->timer.period = (UINT16)(period * 2 + 1);
}

static void pulse_clock_sweep(PULSE* p)
{
	SWEEP* s = &p->sweep;
	UINT16 target = pulse_target_period(p);

	s->divider = (UINT8)(s->divider - 1);
	if (s->divider == 0)
	{
		if (s->shift > 0 && s->enabled && p->real_period >= 8 && target <= 0x7FF)
			pulse_set_period(p, target);
		s->divider = s->period;
	}

	if (s->reload)
	{
		s->divider = s->period;
		s->reload = 0;
	}
}

/* Channel level, 0-15. */
static UINT8 pulse_level(const PULSE* p)
{
	if (pulse_is_muted(p) || ! ((DUTY_TABLE[p->duty] >> p->duty_cycle) & 0x01))
		return 0;
	return envelope_volume(&p->envelope, &p->length);
}

/* The period register survives a reset, as in TetaNES. */
static void pulse_reset(PULSE* p)
{
	memset(&p->timer, 0, sizeof(TIMER));
	memset(&p->length, 0, sizeof(LENGTH_COUNTER));
	memset(&p->sweep, 0, sizeof(SWEEP));
	envelope_reset(&p->envelope);
	p->duty = 0;
	p->duty_cycle = 0;
}


/* ---- Triangle ---- */

static const UINT8 TRIANGLE_SEQUENCE[32] =
{
	15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
};

typedef struct
{
	UINT8 reload;
	UINT8 control;
	UINT8 counter_reload;
	UINT8 counter;
} LINEAR_COUNTER;

typedef struct
{
	TIMER timer;
	UINT8 sequence;
	LENGTH_COUNTER length;
	LINEAR_COUNTER linear;
} TRIANGLE;

static float triangle_level(const TRIANGLE* t)
{
	/* Ultrasonic periods; a console's low-pass filter leaves the middle of the wave. */
	if (t->timer.period < 2)
		return 7.5f;
	return (float)TRIANGLE_SEQUENCE[t->sequence];
}


/* ---- Noise ---- */

static const UINT16 NOISE_PERIOD[2][16] =
{
	{4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068},	/* NTSC */
	{4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778},	/* PAL */
};

typedef struct
{
	TIMER timer;
	UINT16 shift;
	UINT8 shift_mode;	/* 1: feedback from bit 6 instead of bit 1 */
	LENGTH_COUNTER length;
	ENVELOPE envelope;
} NOISE;

static UINT8 noise_level(const NOISE* n)
{
	return (n->shift & 0x01) ? 0 : envelope_volume(&n->envelope, &n->length);
}


/* ---- DMC ---- */

static const UINT16 DMC_PERIOD[2][16] =
{
	{428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54},	/* NTSC */
	{398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50},	/* PAL */
};

typedef struct
{
	TIMER timer;
	UINT8 irq_enabled;
	UINT8 irq_pending;
	UINT8 dma_pending;
	UINT8 loops;
	UINT16 addr;
	UINT16 sample_addr;
	UINT16 bytes_remaining;
	UINT16 sample_length;
	UINT8 sample_buffer;
	UINT8 buffer_empty;
	UINT8 output_level;
	UINT8 bits_remaining;
	UINT8 shift;
	UINT8 silence;
} DMC;

static void dmc_init_sample(DMC* d)
{
	d->addr = d->sample_addr;
	d->bytes_remaining = d->sample_length;
}

static void dmc_step_output(DMC* d)
{
	if (! d->silence)
	{
		/* Move the output level, clamped to 0-127. */
		if (d->shift & 0x01)
		{
			if (d->output_level <= 125)
				d->output_level += 2;
		}
		else if (d->output_level >= 2)
		{
			d->output_level -= 2;
		}
		d->shift >>= 1;
	}

	if (d->bits_remaining > 0)
		d->bits_remaining --;

	if (d->bits_remaining == 0)
	{
		d->bits_remaining = 8;
		d->silence = d->buffer_empty;
		if (! d->buffer_empty)
		{
			d->shift = d->sample_buffer;
			d->buffer_empty = 1;
			if (d->bytes_remaining > 0)
				d->dma_pending = 1;
		}
	}
}


/* ---- Frame counter ----

	mode 0: 4-step  effective rate (approx)
	- - - f f f      60 Hz
	- l - - l -     120 Hz
	e e e - e -     240 Hz

	mode 1: 5-step  effective rate (approx)
	- - - - - -     (interrupt flag never set)
	- l - - l -     96 Hz
	e e e - e -     192 Hz
*/

#define FRAME_NONE	0
#define FRAME_QUARTER	1
#define FRAME_HALF	2

/* [pal][mode][step] */
static const UINT32 STEP_CYCLES[2][2][6] =
{
	{
		{7457, 14913, 22371, 29828, 29829, 29830},
		{7457, 14913, 22371, 29829, 37281, 37282},
	},
	{
		{8313, 16627, 24939, 33252, 33253, 33254},
		{8313, 16627, 24939, 33253, 41565, 41566},
	},
};
static const UINT8 FRAME_TYPE[6] =
{
	FRAME_QUARTER, FRAME_HALF, FRAME_QUARTER, FRAME_NONE, FRAME_HALF, FRAME_NONE,
};

typedef struct
{
	UINT8 step;
	UINT8 mode;
	UINT8 write_pending;
	UINT8 write_buffer;
	UINT8 write_delay;
	UINT8 block_counter;
	UINT32 cycle;
	UINT8 inhibit_irq;
	UINT8 irq_pending;
} FRAME_COUNTER;

/* Runs up to `cycles` cycles, stopping at the next step, and returns how many ran. A step that
	clocks the channels returns its type in *frame (*frame may be set twice, see below). */
static UINT32 frame_clock(FRAME_COUNTER* f, UINT8 pal, UINT32 cycles, UINT8* frame)
{
	UINT32 cycles_ran = 0;
	UINT32 step_cycles = STEP_CYCLES[pal][f->mode][f->step];

	*frame = FRAME_NONE;
	if (f->cycle + cycles >= step_cycles)
	{
		UINT8 type;

		if (! f->inhibit_irq && f->mode == 0 && f->step >= 3)
			f->irq_pending = 1;

		type = FRAME_TYPE[f->step];
		if (type != FRAME_NONE && f->block_counter == 0)
		{
			*frame = type;
			/* Writes to $4017 can't clock for the next cycle (odd + following even cycle). */
			f->block_counter = 2;
		}

		if (step_cycles >= f->cycle)
			cycles_ran = step_cycles - f->cycle;

		f->step ++;
		if (f->step == 6)
		{
			f->step = 0;
			f->cycle = 0;
		}
		else
		{
			f->cycle += cycles_ran;
		}
	}
	else
	{
		cycles_ran = cycles;
		f->cycle += cycles_ran;
	}

	if (f->write_pending)
	{
		f->write_delay --;
		if (f->write_delay == 0)
		{
			f->mode = (f->write_buffer & 0x80) ? 1 : 0;
			f->step = 0;
			f->cycle = 0;
			f->write_pending = 0;
			if (f->mode == 1 && f->block_counter == 0)
			{
				/* Writing $4017 with bit 7 set immediately clocks a half frame. A step's own
					clock blocks this, so the two never both happen. */
				*frame = FRAME_HALF;
				f->block_counter = 2;
			}
		}
	}

	if (f->block_counter > 0)
		f->block_counter --;

	return cycles_ran;
}


/* ---- APU ---- */

typedef struct nesapu_teta
{
	UINT32 sample_rate;
	UINT8 pal;

	/* Cycles into the current block. */
	UINT32 master_clock;
	/* Total CPU cycles clocked, for the parity of $4017 writes. */
	UINT32 cpu_cycle;
	/* Cycles the frame counter has run in this block. */
	UINT32 clock;
	/* Cycles the mixer has consumed in this block, which trails `clock`. */
	UINT32 mix_clock;
	/* Mixed level as of mix_clock, so a change is a delta from it. */
	float mixed_level[2];
	/* Set by a register write: the next cycle is clocked on its own, as TetaNES does after a
		write, so a length load only races a frame clock that falls on that cycle. */
	UINT8 single_step;

	FRAME_COUNTER frame_counter;
	PULSE pulse[2];
	TRIANGLE triangle;
	NOISE noise;
	DMC dmc;

	SYNTH synth;
	FILTER_CHAIN filter[2];
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
} NESAPU_TETA;

/* The five channels mixed, into out[0] (left) and out[1] (right). */
static void apu_mix_level(const NESAPU_TETA* apu, float* out)
{
	/* The linear approximation's gains, from the same wiki page. */
	static const float LINEAR_GAIN[CHN_COUNT] = {0.00752f, 0.00752f, 0.00851f, 0.00494f, 0.00335f};
	UINT8 mute = apu->mute_mask;
	float level[CHN_COUNT];
	UINT8 chn;

	level[CHN_PULSE1] = (mute & 0x01) ? 0.0f : (float)pulse_level(&apu->pulse[0]);
	level[CHN_PULSE2] = (mute & 0x02) ? 0.0f : (float)pulse_level(&apu->pulse[1]);
	level[CHN_TRIANGLE] = (mute & 0x04) ? 0.0f : triangle_level(&apu->triangle);
	level[CHN_NOISE] = (mute & 0x08) ? 0.0f : (float)noise_level(&apu->noise);
	level[CHN_DMC] = (mute & 0x10) ? 0.0f : (float)apu->dmc.output_level;

	if (apu->options & OPT_NES_NONLINEAR_MIXER)
	{
		UINT32 tnd = (UINT32)(3.0f * level[CHN_TRIANGLE] + 2.0f * level[CHN_NOISE] + level[CHN_DMC]);

		out[0] = apu->pulse_table[(UINT32)(level[CHN_PULSE1] + level[CHN_PULSE2])] + apu->tnd_table[tnd];
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

/* Hands the synth however much the mixed level moved since it was last told. */
static void apu_add_mix_delta(NESAPU_TETA* apu)
{
	float level[2];

	apu_mix_level(apu, level);
	if (level[0] != apu->mixed_level[0] || level[1] != apu->mixed_level[1])
	{
		synth_add_delta(&apu->synth, apu->mix_clock,
			level[0] - apu->mixed_level[0], level[1] - apu->mixed_level[1]);
		apu->mixed_level[0] = level[0];
		apu->mixed_level[1] = level[1];
	}
}

/* The next cycle any channel's output could change on. A channel whose output is pinned has
	none, however fast its divider runs. */
static UINT32 apu_next_change(const NESAPU_TETA* apu)
{
	const TRIANGLE* t = &apu->triangle;
	const NOISE* n = &apu->noise;
	UINT32 next = timer_next_expiry(&apu->dmc.timer);
	UINT8 i;

	for (i = 0; i < 2; i ++)
	{
		const PULSE* p = &apu->pulse[i];

		if (! pulse_is_muted(p) && envelope_volume(&p->envelope, &p->length) > 0)
			next = timer_earliest(next, &p->timer);
	}
	/* Below period 2 the triangle's output is pinned, and either counter at zero holds its
		sequencer. */
	if (t->timer.period >= 2 && t->length.counter > 0 && t->linear.counter > 0)
		next = timer_earliest(next, &t->timer);
	/* Silenced by its envelope or length counter, the noise's shift register runs without the
		output following it. */
	if (envelope_volume(&n->envelope, &n->length) > 0)
		next = timer_earliest(next, &n->timer);
	return next;
}

/* Fetches the DMC's next sample byte if it asked for one. */
static void apu_service_dma(NESAPU_TETA* apu)
{
	DMC* d = &apu->dmc;

	if (! d->dma_pending)
		return;
	d->dma_pending = 0;
	if (d->bytes_remaining == 0)
		return;
	d->sample_buffer = (apu->memory != NULL && d->addr >= 0x8000) ? apu->memory[d->addr - 0x8000] : 0x00;
	d->buffer_empty = 0;
	d->addr = (d->addr == 0xFFFF) ? 0x8000 : (UINT16)(d->addr + 1);
	d->bytes_remaining --;
	if (d->bytes_remaining == 0)
	{
		if (d->loops)
			dmc_init_sample(d);
		else if (d->irq_enabled)
			d->irq_pending = 1;
	}
}

/* Runs every channel forward to `cycle`. */
static void apu_clock_channels(NESAPU_TETA* apu, UINT32 cycle)
{
	TRIANGLE* t = &apu->triangle;
	NOISE* n = &apu->noise;
	UINT8 i;

	for (i = 0; i < 2; i ++)
	{
		PULSE* p = &apu->pulse[i];

		while (timer_run_to(&p->timer, cycle))
			p->duty_cycle = (UINT8)((p->duty_cycle - 1) & 0x07);
	}
	while (timer_run_to(&t->timer, cycle))
	{
		if (t->length.counter > 0 && t->linear.counter > 0)
			t->sequence = (UINT8)((t->sequence + 1) & 0x1F);
	}
	while (timer_run_to(&n->timer, cycle))
	{
		UINT16 feedback = (UINT16)((n->shift ^ (n->shift >> (n->shift_mode ? 6 : 1))) & 0x01);

		n->shift = (UINT16)((n->shift >> 1) | (feedback << 14));
	}
	while (timer_run_to(&apu->dmc.timer, cycle))
	{
		dmc_step_output(&apu->dmc);
		apu_service_dma(apu);
	}
}

/* Advances every channel to `target`, recording each change of the mixed output as it happens.
	It walks from one cycle a channel could change on to the next rather than visiting every
	cycle. */
static void apu_channels_clock_to(NESAPU_TETA* apu, UINT32 target)
{
	/* Anything that moved the mix since the last visit, such as a register write or a length
		counter clocked to zero, lands here at the cycle it happened on. */
	apu_add_mix_delta(apu);
	while (apu->mix_clock < target)
	{
		UINT32 next = apu_next_change(apu);

		/* A channel can sit ahead of mix_clock, and a stop that isn't in the future would not
			make progress. */
		if (next < apu->mix_clock + 1)
			next = apu->mix_clock + 1;
		if (next > target)
			next = target;
		apu_clock_channels(apu, next);
		apu->mix_clock = next;
		apu_add_mix_delta(apu);
	}
}

/* Clocks the envelopes and the linear counter, and on a half frame the length counters and
	sweeps too. */
static void apu_clock_frame(NESAPU_TETA* apu, UINT8 frame)
{
	LINEAR_COUNTER* linear = &apu->triangle.linear;
	UINT8 i;

	envelope_clock(&apu->pulse[0].envelope);
	envelope_clock(&apu->pulse[1].envelope);
	envelope_clock(&apu->noise.envelope);
	if (linear->reload)
		linear->counter = linear->counter_reload;
	else if (linear->counter > 0)
		linear->counter --;
	if (! linear->control)
		linear->reload = 0;

	if (frame != FRAME_HALF)
		return;
	for (i = 0; i < 2; i ++)
	{
		length_clock(&apu->pulse[i].length);
		pulse_clock_sweep(&apu->pulse[i]);
	}
	length_clock(&apu->triangle.length);
	length_clock(&apu->noise.length);
}

/* Runs the frame counter and all channels up to `cycle` of the block. */
static void apu_clock_to(NESAPU_TETA* apu, UINT32 cycle)
{
	apu->master_clock = cycle;
	while (apu->master_clock > apu->clock)
	{
		FRAME_COUNTER* fc = &apu->frame_counter;
		UINT32 cycles = apu->master_clock - apu->clock;
		UINT8 frame;

		/* A pending $4017 write and the write block count single cycles. */
		if (apu->single_step || fc->write_pending || fc->block_counter > 0)
			cycles = 1;
		apu->single_step = 0;
		apu->clock += frame_clock(fc, apu->pal, cycles, &frame);
		if (frame != FRAME_NONE)
		{
			/* Channels catch up to the frame step before it clocks them. */
			apu_channels_clock_to(apu, apu->clock);
			apu_clock_frame(apu, frame);
		}

		length_reload(&apu->pulse[0].length);
		length_reload(&apu->pulse[1].length);
		length_reload(&apu->triangle.length);
		length_reload(&apu->noise.length);

		apu_channels_clock_to(apu, apu->clock);
	}
}

void* nesapu_teta_create(UINT32 clock, UINT32 rate)
{
	NESAPU_TETA* apu;
	int i;

	if (clock == 0 || rate == 0 || rate > clock / 4)
		return NULL;
	apu = (NESAPU_TETA*)calloc(1, sizeof(NESAPU_TETA));
	if (apu == NULL)
		return NULL;
	apu->sample_rate = rate;
	/* PAL consoles run at 1.66 MHz, NTSC at 1.79 MHz and Dendy (NTSC timing) at 1.77 MHz. */
	apu->pal = (clock < 1700000) ? 1 : 0;

	if (! synth_init(&apu->synth, clock, rate, CYCLE_SIZE))
	{
		nesapu_teta_destroy(apu);
		return NULL;
	}
	apu->block = (float*)calloc(apu->synth.length, sizeof(float));
	if (apu->block == NULL)
	{
		nesapu_teta_destroy(apu);
		return NULL;
	}

	/* Entry 0 of each mixer table is left at 0 by calloc. */
	for (i = 1; i < 31; i ++)
		apu->pulse_table[i] = 95.52f / (8128.0f / (float)i + 100.0f);
	for (i = 1; i < 203; i ++)
		apu->tnd_table[i] = 163.67f / (24329.0f / (float)i + 100.0f);
	for (i = 0; i < CHN_COUNT; i ++)
		apu->pan[i][0] = apu->pan[i][1] = 1.0f;
	apu->pulse[0].one = 1;
	apu->options = OPT_NES_UNMUTE_ON_RESET | OPT_NES_NONLINEAR_MIXER | OPT_NES_ENABLE_4011 |
		OPT_NES_ENABLE_PNOISE;

	nesapu_teta_reset(apu);
	return apu;
}

void nesapu_teta_destroy(void* chip)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;

	synth_free(&apu->synth);
	free(apu->block);
	free(apu);
}

void nesapu_teta_reset(void* chip)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;
	TRIANGLE* t = &apu->triangle;
	NOISE* n = &apu->noise;
	DMC* d = &apu->dmc;
	FRAME_COUNTER* fc = &apu->frame_counter;

	apu->cpu_cycle = 0;
	apu->master_clock = 0;
	apu->clock = 0;
	apu->mix_clock = 0;
	apu->mixed_level[0] = apu->mixed_level[1] = 0.0f;
	apu->single_step = 0;

	/* After reset the APU acts as if $00 was written to $4017 9-12 clocks before the first
		instruction. TetaNES shortens the delay (3 for an even cycle) by one to pass its reset
		timing tests. */
	memset(fc, 0, sizeof(FRAME_COUNTER));
	fc->write_pending = 1;
	fc->write_delay = 2;

	pulse_reset(&apu->pulse[0]);
	pulse_reset(&apu->pulse[1]);

	/* The triangle's timer survives a reset, as in TetaNES. */
	memset(&t->length, 0, sizeof(LENGTH_COUNTER));
	memset(&t->linear, 0, sizeof(LINEAR_COUNTER));
	t->sequence = 0;

	memset(&n->timer, 0, sizeof(TIMER));
	n->timer.period = (UINT16)(NOISE_PERIOD[apu->pal][0] - 1);
	memset(&n->length, 0, sizeof(LENGTH_COUNTER));
	envelope_reset(&n->envelope);
	n->shift = 1;
	n->shift_mode = 0;

	memset(d, 0, sizeof(DMC));
	d->timer.period = (UINT16)(DMC_PERIOD[apu->pal][0] - 1);
	d->timer.counter = d->timer.period;
	/* TetaNES parks the timer a cycle ahead after a reset, which its DMA timing tests need. */
	d->timer.cycle = 1;
	d->sample_addr = 0xC000;
	d->sample_length = 1;
	d->buffer_empty = 1;
	d->bits_remaining = 8;
	d->silence = 1;

	synth_clear(&apu->synth);
	filter_init(&apu->filter[0], (float)apu->sample_rate);
	filter_init(&apu->filter[1], (float)apu->sample_rate);

	/* Some VGMs were logged without the driver's $4015 write, and only play with the pulse,
		triangle and noise channels enabled from the start. */
	if (apu->options & OPT_NES_UNMUTE_ON_RESET)
		nesapu_teta_write(apu, 0x15, 0x0F);
}

void nesapu_teta_write(void* chip, UINT8 offset, UINT8 data)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;
	PULSE* p = &apu->pulse[(offset >> 2) & 0x01];
	TRIANGLE* t = &apu->triangle;
	NOISE* n = &apu->noise;
	DMC* d = &apu->dmc;
	FRAME_COUNTER* fc = &apu->frame_counter;

	apu_clock_to(apu, apu->master_clock);
	switch(offset)
	{
	case 0x00:
	case 0x04:
		/* Famiclone duty order: 25% and 50% swapped. */
		if ((apu->options & OPT_NES_DUTY_SWAP) && ((data >> 6) == 1 || (data >> 6) == 2))
			data ^= 0xC0;
		p->duty = data >> 6;
		envelope_write_ctrl(&p->envelope, &p->length, data);
		break;
	case 0x01:
	case 0x05:
		p->sweep.enabled = (data & 0x80) ? 1 : 0;
		p->sweep.negate = (data & 0x08) ? 1 : 0;
		p->sweep.period = (UINT8)(((data & 0x70) >> 4) + 1);
		p->sweep.shift = data & 0x07;
		p->sweep.reload = 1;
		break;
	case 0x02:
	case 0x06:
		pulse_set_period(p, (UINT16)((p->real_period & 0x0700) | data));
		break;
	case 0x03:
	case 0x07:
		/* Restarts the duty sequencer but leaves the divider running. */
		length_write(&p->length, data);
		pulse_set_period(p, (UINT16)((p->real_period & 0x00FF) | ((data & 0x07) << 8)));
		p->duty_cycle = 0;
		p->envelope.start = 1;
		break;
	case 0x08:
		t->linear.control = (data & 0x80) ? 1 : 0;
		t->linear.counter_reload = data & 0x7F;
		t->length.new_halt = t->linear.control;
		break;
	case 0x0A:
		t->timer.period = (UINT16)((t->timer.period & 0xFF00) | data);
		break;
	case 0x0B:
		length_write(&t->length, data);
		t->timer.period = (UINT16)((t->timer.period & 0x00FF) | ((data & 0x07) << 8));
		t->linear.reload = 1;
		break;
	case 0x0C:
		envelope_write_ctrl(&n->envelope, &n->length, data);
		break;
	case 0x0E:
		n->timer.period = (UINT16)(NOISE_PERIOD[apu->pal][data & 0x0F] - 1);
		/* Early Famicoms and some arcade boards have no short noise mode. */
		n->shift_mode = ((data & 0x80) && (apu->options & OPT_NES_ENABLE_PNOISE)) ? 1 : 0;
		break;
	case 0x0F:
		length_write(&n->length, data);
		n->envelope.start = 1;
		break;
	case 0x10:
		d->irq_enabled = (data & 0x80) ? 1 : 0;
		d->loops = (data & 0x40) ? 1 : 0;
		d->timer.period = (UINT16)(DMC_PERIOD[apu->pal][data & 0x0F] - 1);
		if (! d->irq_enabled)
			d->irq_pending = 0;
		break;
	case 0x11:
		if (apu->options & OPT_NES_ENABLE_4011)
			d->output_level = data & 0x7F;
		break;
	case 0x12:
		d->sample_addr = (UINT16)(0xC000 | (data << 6));
		break;
	case 0x13:
		d->sample_length = (UINT16)((data << 4) | 1);
		break;
	case 0x15:
		length_set_enabled(&apu->pulse[0].length, data & 0x01);
		length_set_enabled(&apu->pulse[1].length, data & 0x02);
		length_set_enabled(&t->length, data & 0x04);
		length_set_enabled(&n->length, data & 0x08);
		if (! (data & 0x10))
		{
			d->bytes_remaining = 0;
		}
		else if (d->bytes_remaining == 0)
		{
			/* sample_length is never 0, so this always leaves bytes to fetch. */
			dmc_init_sample(d);
			if (d->buffer_empty)
				d->dma_pending = 1;
		}
		d->irq_pending = 0;
		apu_service_dma(apu);
		break;
	case 0x17:
		fc->write_pending = 1;
		fc->write_buffer = data;
		/* Writes occurring on odd clocks are delayed. */
		fc->write_delay = ((apu->cpu_cycle + apu->master_clock) & 0x01) ? 4 : 3;
		fc->inhibit_irq = (data & 0x40) ? 1 : 0;
		if (fc->inhibit_irq)
			fc->irq_pending = 0;
		break;
	}
	apu->single_step = 1;
}

UINT8 nesapu_teta_read(void* chip, UINT8 offset)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;
	UINT8 status = 0x00;

	if (offset != 0x15)
		return 0x00;
	apu_clock_to(apu, apu->master_clock);
	if (apu->pulse[0].length.counter > 0)
		status |= 0x01;
	if (apu->pulse[1].length.counter > 0)
		status |= 0x02;
	if (apu->triangle.length.counter > 0)
		status |= 0x04;
	if (apu->noise.length.counter > 0)
		status |= 0x08;
	if (apu->dmc.bytes_remaining > 0)
		status |= 0x10;
	if (apu->frame_counter.irq_pending)
		status |= 0x40;
	if (apu->dmc.irq_pending)
		status |= 0x80;
	/* Reading $4015 acknowledges the frame IRQ. */
	apu->frame_counter.irq_pending = 0;
	return status;
}

void nesapu_teta_set_memory(void* chip, const UINT8* memory)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;

	apu->memory = memory;
}

/* UNMUTE_ON_RESET and the options that describe hardware variants are honored, the first at the
	next reset. The others (PHASE_REFRESH, DPCM_ANTI_CLICK, RANDOMIZE_NOISE, TRI_MUTE, TRI_NULL)
	tune the MAME and NSFPlay cores' approximations of behavior this core emulates as the
	hardware does. */
void nesapu_teta_set_options(void* chip, UINT32 flags)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;

	apu->options = flags;
}

void nesapu_teta_set_mute_mask(void* chip, UINT32 mask)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;

	apu->mute_mask = (UINT8)mask;
}

void nesapu_teta_set_panning(void* chip, const INT16* pan)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;
	INT32 gains[2];
	UINT8 chn;

	for (chn = 0; chn < CHN_COUNT; chn ++)
	{
		Panning_Calculate(gains, pan[chn]);
		apu->pan[chn][0] = (float)gains[0] / PANNING_NORMAL;
		apu->pan[chn][1] = (float)gains[1] / PANNING_NORMAL;
	}
}

void nesapu_teta_update(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	NESAPU_TETA* apu = (NESAPU_TETA*)chip;
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
		apu_clock_to(apu, cycles);
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

		/* Start the next block, putting every cycle counter back to zero. The channels are
			level with master_clock here. */
		apu->cpu_cycle += cycles;
		apu->master_clock = apu->clock = apu->mix_clock = 0;
		apu->pulse[0].timer.cycle = apu->pulse[1].timer.cycle = 0;
		apu->triangle.timer.cycle = apu->noise.timer.cycle = apu->dmc.timer.cycle = 0;
		done += count;
	}
}
