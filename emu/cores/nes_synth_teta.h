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

#ifndef __NES_SYNTH_TETA_H__
#define __NES_SYNTH_TETA_H__

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../stdtype.h"
#include "../../libs/blep_filter/blep_filter.h"

#define TETA_PI	3.14159265358979323846
#define TETA_TAU	(2.0 * TETA_PI)


/* ---- Band-limited synthesis ----

	Each change of the mixed level is added to blep_filter at its input-clock position. It
	reconstructs band-limited output directly at the requested sample rate, without chip
	oversampling. */

#define SYNTH_BUFFER_SIZE	4000
#define SYNTH_LEVEL_SCALE	16384.0f
typedef struct
{
	/* One band-limited sample buffer per output channel. */
	blip_t* buffers[2];
	UINT32 length;
} SYNTH;

/* Sizes the synth for blocks of up to `block_cycles` cycles. Returns 0 if out of memory. */
static UINT8 synth_init(SYNTH* s, UINT32 clock_rate, UINT32 sample_rate, UINT32 block_cycles)
{
	s->length = (UINT32)(((INT64)block_cycles * sample_rate + clock_rate - 1) / clock_rate) + 1;
	s->buffers[0] = blip_new(SYNTH_BUFFER_SIZE);
	s->buffers[1] = blip_new(SYNTH_BUFFER_SIZE);
	if (s->buffers[0] == NULL || s->buffers[1] == NULL)
	{
		blip_delete(s->buffers[0]);
		blip_delete(s->buffers[1]);
		s->buffers[0] = s->buffers[1] = NULL;
		return 0;
	}
	blip_set_rates(s->buffers[0], clock_rate, sample_rate);
	blip_set_rates(s->buffers[1], clock_rate, sample_rate);
	return 1;
}

static void synth_free(SYNTH* s)
{
	blip_delete(s->buffers[0]);
	blip_delete(s->buffers[1]);
	s->buffers[0] = s->buffers[1] = NULL;
}

static void synth_clear(SYNTH* s)
{
	blip_clear(s->buffers[0]);
	blip_clear(s->buffers[1]);
}

static UINT32 synth_clocks_needed(const SYNTH* s, UINT32 samples)
{
	if (samples > SYNTH_BUFFER_SIZE)
		samples = SYNTH_BUFFER_SIZE;
	return (UINT32)blip_clocks_needed(s->buffers[0], (int)samples);
}

/* Ends a block of `cycles`, returning the number of complete output samples. */
static UINT32 synth_end_block(SYNTH* s, UINT32 cycles)
{
	blip_end_frame(s->buffers[0], cycles);
	blip_end_frame(s->buffers[1], cycles);
	return (UINT32)blip_samples_avail(s->buffers[0]);
}

/* Records a change of the output level `cycle` cycles into the block. */
static void synth_add_delta(SYNTH* s, UINT32 cycle, float left, float right)
{
	int left_delta = (int)(left * SYNTH_LEVEL_SCALE + ((left < 0.0f) ? -0.5f : 0.5f));
	int right_delta = (int)(right * SYNTH_LEVEL_SCALE + ((right < 0.0f) ? -0.5f : 0.5f));

	if (left_delta != 0)
		blip_add_delta(s->buffers[0], cycle, left_delta);
	if (right_delta != 0)
		blip_add_delta(s->buffers[1], cycle, right_delta);
}

/* Reads and converts the filter's signed 16-bit output to the cores' float mix scale. */
static void synth_read(SYNTH* s, UINT8 chn, UINT32 count, float* out)
{
	short samples[SYNTH_BUFFER_SIZE];
	UINT32 i;

	if (count > SYNTH_BUFFER_SIZE)
		count = SYNTH_BUFFER_SIZE;
	blip_read_samples(s->buffers[chn], samples, (int)count, 0);
	for (i = 0; i < count; i ++)
		out[i] = (float)samples[i] / SYNTH_LEVEL_SCALE;
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
	float rc = (float)(1.0 / (TETA_TAU * cutoff));

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


#endif	/* __NES_SYNTH_TETA_H__ */
