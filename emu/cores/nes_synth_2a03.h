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

#ifndef __NES_SYNTH_2A03_H__
#define __NES_SYNTH_2A03_H__

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../stdtype.h"

#define SYNTH_PI	3.14159265358979323846
#define SYNTH_TAU	(2.0 * SYNTH_PI)


/* ---- Band-limited synthesis ----

	Each change of the mixed level is added as a band-limited step: the output-rate response of
	an ideal step at that instant, precomputed at SYNTH_PHASES sub-sample positions. Summing the
	responses reconstructs the waveform with nothing above Nyquist in it, so nothing aliases.
	The cost follows the number of level changes rather than the number of cycles. */

#define SYNTH_PHASES	32
#define SYNTH_PHASE_BITS	5
#define SYNTH_WIDTH	16
#define SYNTH_FRAC_BITS	20
#define SYNTH_ONE	((INT64)1 << SYNTH_FRAC_BITS)

typedef struct
{
	/* Change in output level at each output sample, per stereo channel. */
	float* deltas[2];
	UINT32 length;
	/* SYNTH_PHASES impulse responses of SYNTH_WIDTH taps, one per sub-sample position. */
	float kernel[SYNTH_PHASES * SYNTH_WIDTH];
	/* Output samples per input cycle, in SYNTH_FRAC_BITS fixed point. */
	INT64 samples_per_cycle;
	/* Running level, carried across reads. */
	float level[2];
	/* Sub-sample position of cycle 0 of the current block, in fixed point. */
	INT64 offset;
} SYNTH;

/* One Blackman-windowed sinc per sub-sample phase, each normalized to sum to 1 so a step
	settles at its amplitude whatever fraction of a sample it lands on. */
static void synth_build_kernel(SYNTH* s)
{
	double center = (double)(SYNTH_WIDTH / 2);
	int phase;
	int tap;

	for (phase = 0; phase < SYNTH_PHASES; phase ++)
	{
		float* taps = &s->kernel[phase * SYNTH_WIDTH];
		float offset = (float)phase / SYNTH_PHASES;
		float sum = 0.0f;

		for (tap = 0; tap < SYNTH_WIDTH; tap ++)
		{
			/* Distance from the step, in output samples. */
			float x = (float)tap - (float)center + 1.0f - offset;
			float sinc;
			float w;
			float window;

			if (fabs(x) < 1e-6)
			{
				sinc = 1.0f;
			}
			else
			{
				float pi_x = (float)SYNTH_PI * x;
				sinc = (float)sin(pi_x) / pi_x;
			}
			w = ((float)tap + 1.0f - offset) / SYNTH_WIDTH;
			window = (float)(0.42 - 0.5 * cos(SYNTH_TAU * w) + 0.08 * cos(2.0 * SYNTH_TAU * w));
			taps[tap] = sinc * window;
		}
		for (tap = 0; tap < SYNTH_WIDTH; tap ++)
			sum += taps[tap];
		for (tap = 0; tap < SYNTH_WIDTH; tap ++)
			taps[tap] /= sum;
	}
}

/* Sizes the synth for blocks of up to `block_cycles` cycles. Returns 0 if out of memory. */
static UINT8 synth_init(SYNTH* s, UINT32 clock_rate, UINT32 sample_rate, UINT32 block_cycles)
{
	s->samples_per_cycle = (INT64)((double)sample_rate / (double)clock_rate * (double)SYNTH_ONE);
	/* + 1 for the fractional offset a block can carry in, and SYNTH_WIDTH so a step placed at
		the last sample still writes its whole response. */
	s->length = (UINT32)(((INT64)block_cycles * s->samples_per_cycle) >> SYNTH_FRAC_BITS) +
		1 + SYNTH_WIDTH;
	s->deltas[0] = (float*)calloc(s->length, sizeof(float));
	s->deltas[1] = (float*)calloc(s->length, sizeof(float));
	if (s->deltas[0] == NULL || s->deltas[1] == NULL)
		return 0;
	synth_build_kernel(s);
	s->level[0] = s->level[1] = 0.0f;
	s->offset = 0;
	return 1;
}

static void synth_free(SYNTH* s)
{
	free(s->deltas[0]);
	free(s->deltas[1]);
}

static void synth_clear(SYNTH* s)
{
	memset(s->deltas[0], 0, s->length * sizeof(float));
	memset(s->deltas[1], 0, s->length * sizeof(float));
	s->level[0] = s->level[1] = 0.0f;
	s->offset = 0;
}

/* Ends a block of `cycles`, returning how many output samples it completed. The leftover
	fraction of a sample starts the next block. */
static UINT32 synth_end_block(SYNTH* s, UINT32 cycles)
{
	INT64 end = s->offset + (INT64)cycles * s->samples_per_cycle;
	UINT32 count = (UINT32)(end >> SYNTH_FRAC_BITS);

	s->offset = end - ((INT64)count << SYNTH_FRAC_BITS);
	return count;
}

/* Records a change of the output level `cycle` cycles into the block. */
static void synth_add_delta(SYNTH* s, UINT32 cycle, float left, float right)
{
	INT64 position;
	UINT32 sample;
	const float* taps;
	int tap;

	if (left == 0.0f && right == 0.0f)
		return;
	position = s->offset + (INT64)cycle * s->samples_per_cycle;
	sample = (UINT32)(position >> SYNTH_FRAC_BITS);
	if (sample + SYNTH_WIDTH > s->length)
		return;
	/* Anything finer than a phase is truncated rather than rounded. */
	taps = &s->kernel[(UINT32)((position >> (SYNTH_FRAC_BITS - SYNTH_PHASE_BITS)) &
		(SYNTH_PHASES - 1)) * SYNTH_WIDTH];
	for (tap = 0; tap < SYNTH_WIDTH; tap ++)
	{
		s->deltas[0][sample + tap] += left * taps[tap];
		s->deltas[1][sample + tap] += right * taps[tap];
	}
}

/* Integrates `count` samples of channel `chn` into `out`. Call synth_consume afterwards. */
static void synth_read(SYNTH* s, UINT8 chn, UINT32 count, float* out)
{
	UINT32 i;

	for (i = 0; i < count; i ++)
	{
		s->level[chn] += s->deltas[chn][i];
		out[i] = s->level[chn];
	}
}

/* Drops `count` read samples. What a step contributed past them stays for the next read. */
static void synth_consume(SYNTH* s, UINT32 count)
{
	UINT8 chn;

	if (count > s->length)
		count = s->length;
	for (chn = 0; chn < 2; chn ++)
	{
		memmove(s->deltas[chn], s->deltas[chn] + count, (s->length - count) * sizeof(float));
		memset(s->deltas[chn] + (s->length - count), 0, count * sizeof(float));
	}
}


/* ---- Output filters ----
	The console's own RC corners: two high-pass stages and a 14 kHz low-pass, in that order. */

/* Flushes values far below hearing to zero, so the decaying filters never produce denormals. */
static float flush(float value)
{
	return (fabs(value) < 1e-20) ? 0.0f : value;
}

typedef struct
{
	float alpha;
	float output;
} LOW_PASS;

typedef struct
{
	float alpha;
	float prev_output;
	float prev_input;
	float delta;
} HIGH_PASS;

typedef struct
{
	HIGH_PASS high_pass_90;
	HIGH_PASS high_pass_440;
	LOW_PASS low_pass_14k;
} FILTER_CHAIN;

static void low_pass_init(LOW_PASS* f, float sample_rate, float cutoff)
{
	float period = 1.0f / sample_rate;
	float rc = (float)(1.0 / (SYNTH_TAU * cutoff));

	f->alpha = period / (rc + period);
	f->output = 0.0f;
}

static float low_pass_consume(LOW_PASS* f, float sample)
{
	f->output = flush(f->output + f->alpha * (sample - f->output));
	return f->output;
}

static void high_pass_init(HIGH_PASS* f, float sample_rate, float cutoff)
{
	float period = 1.0f / sample_rate;
	float cutoff_period = 1.0f / cutoff;

	f->alpha = cutoff_period / (cutoff_period + period);
	f->prev_output = 0.0f;
	f->prev_input = 0.0f;
	f->delta = 0.0f;
}

static float high_pass_output(const HIGH_PASS* f)
{
	return f->alpha * f->prev_output + f->alpha * f->delta;
}

static void high_pass_consume(HIGH_PASS* f, float sample)
{
	f->prev_output = flush(high_pass_output(f));
	f->delta = flush(sample - f->prev_input);
	f->prev_input = sample;
}

static void filter_init(FILTER_CHAIN* f, float sample_rate)
{
	high_pass_init(&f->high_pass_90, sample_rate, 90.0f);
	high_pass_init(&f->high_pass_440, sample_rate, 440.0f);
	low_pass_init(&f->low_pass_14k, sample_rate, 14000.0f);
}

static float filter_consume(FILTER_CHAIN* f, float sample)
{
	high_pass_consume(&f->high_pass_90, sample);
	high_pass_consume(&f->high_pass_440, high_pass_output(&f->high_pass_90));
	return low_pass_consume(&f->low_pass_14k, high_pass_output(&f->high_pass_440));
}


#endif	/* __NES_SYNTH_2A03_H__ */
