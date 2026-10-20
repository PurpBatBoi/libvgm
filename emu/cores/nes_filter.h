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

#ifndef __NES_FILTER_H__
#define __NES_FILTER_H__

#include <math.h>

#include "../../stdtype.h"

/* Output filters of the NES / Famicom, shared by the BeesNES APU (beesnes_intf.c) and the FDS (purps_fds.c).
	https://www.nesdev.org/wiki/APU_Mixer: the Famicom specifies a first-order high-pass at 37 Hz. The NES
	adds a second high-pass at 440 Hz and its first one is at 90 Hz; those two are not used here, because they
	take 5 to 19 dB off everything below 440 Hz (thin bass, quiet DPCM, sharper triangle clicks).
	The first-order low-pass at 14 kHz is the NES's, kept to round off the steps near the Nyquist frequency.
	https://www.nesdev.org/wiki/FDS_audio: the FDS output has its own low-pass of about 2 kHz.
	Each one is an RC stage, discretized at the output sample rate. */

#define NES_FILTER_PI	3.14159265358979323846

/* One RC low-pass: y += k * (x - y) */
typedef struct
{
	float k;
	float y;
} NES_LOW_PASS;

/* One RC high-pass: y = k * (y + x - x_prev) */
typedef struct
{
	float k;
	float y;
	float x_prev;
} NES_HIGH_PASS;

/* The console's chain, in the order the signal goes through it */
typedef struct
{
	NES_HIGH_PASS hp_37;
	NES_LOW_PASS lp_14k;
} NES_OUTPUT_FILTER;

static void nes_low_pass_init(NES_LOW_PASS* f, float sample_rate, float cutoff)
{
	double rc = 1.0 / (2.0 * NES_FILTER_PI * cutoff);
	double dt = 1.0 / sample_rate;

	f->k = (float)(dt / (rc + dt));
	f->y = 0.0f;
}

static float nes_low_pass(NES_LOW_PASS* f, float x)
{
	f->y += f->k * (x - f->y);
	return f->y;
}

static void nes_high_pass_init(NES_HIGH_PASS* f, float sample_rate, float cutoff)
{
	double rc = 1.0 / (2.0 * NES_FILTER_PI * cutoff);
	double dt = 1.0 / sample_rate;

	f->k = (float)(rc / (rc + dt));
	f->y = 0.0f;
	f->x_prev = 0.0f;
}

static float nes_high_pass(NES_HIGH_PASS* f, float x)
{
	f->y = f->k * (f->y + x - f->x_prev);
	f->x_prev = x;
	/* With a constant input the output decays forever: stop it before it reaches denormal numbers, which are slow. */
	if (f->y > -1e-20f && f->y < 1e-20f)
		f->y = 0.0f;
	return f->y;
}

static void nes_output_filter_init(NES_OUTPUT_FILTER* f, float sample_rate)
{
	nes_high_pass_init(&f->hp_37, sample_rate, 37.0f);
	nes_low_pass_init(&f->lp_14k, sample_rate, 14000.0f);
}

static float nes_output_filter(NES_OUTPUT_FILTER* f, float x)
{
	return nes_low_pass(&f->lp_14k, nes_high_pass(&f->hp_37, x));
}

#endif	/* __NES_FILTER_H__ */
