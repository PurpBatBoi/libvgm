/*
  Band-limited resampling sample buffer.

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

#include "blep_filter.h"

#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#if INT_MAX < 0x7FFFFFFF
	#error "blip_buf needs int to be at least 32 bits wide"
#endif

/* ------------------------------------------------------------------------
 * Optional SIMD. Pure C90 builds simply skip these headers.
 */
#if !defined(BLIP_BUF_NO_SIMD) && (defined(__SSE2__) || defined(_M_X64) || \
		defined(_M_AMD64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
	#include <emmintrin.h>
	#define BLIP_USE_SSE2 1
#endif

/* ------------------------------------------------------------------------
 * Position type: 64 bits wherever the compiler offers it.
 */
#if defined(BLIP_BUF_TEST_32BIT_POS)          /* exercise the fallback */
	typedef unsigned int blip_pos_t;
	#define BLIP_POS_FRAC 20
#elif ULONG_MAX / 0xFFFFFFFFUL > 0xFFFFFFFFUL
	typedef unsigned long blip_pos_t;          /* LP64 */
	#define BLIP_POS_FRAC 52
#elif defined(_MSC_VER)
	typedef unsigned __int64 blip_pos_t;       /* Win64 / Win32 */
	#define BLIP_POS_FRAC 52
#elif defined(__GNUC__)
	__extension__ typedef unsigned long long blip_pos_t; /* ILP32 gcc/clang */
	#define BLIP_POS_FRAC 52
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
	typedef unsigned long long blip_pos_t;
	#define BLIP_POS_FRAC 52
#else
	typedef unsigned long blip_pos_t;          /* 32-bit fallback */
	#define BLIP_POS_FRAC 20                   /* coarser rate accuracy */
#endif

#define BLIP_POS_ONE ((blip_pos_t) 1 << BLIP_POS_FRAC)

/* Kernel addressing: 20 fraction bits = 5 phase bits + 15 blend bits. */
#define BLIP_SUB_BITS     20
#define BLIP_PHASE_BITS   5
#define BLIP_PHASES       (1 << BLIP_PHASE_BITS)
#define BLIP_INTERP_BITS  (BLIP_SUB_BITS - BLIP_PHASE_BITS)
#define BLIP_INTERP_MASK  ((1 << BLIP_INTERP_BITS) - 1)

/* Each impulse sums to 1 << BLIP_KERNEL_BITS, which is also the scale of
 * the integrator relative to 16-bit output. */
#define BLIP_KERNEL_BITS  15
#define BLIP_TAPS         16
#define BLIP_CENTER       7   /* tap that sits on the delta at phase 0 */

/* Deltas may land up to this many samples past the end of the frame. */
#define BLIP_LATE         2
/* Extra slots kept after the last readable sample. */
#define BLIP_PAD          (BLIP_TAPS + BLIP_LATE)

/* DC-blocking leak per output sample is 2^-BLIP_BASS_SHIFT
 * (about 14 Hz corner at 44.1 kHz). */
#ifndef BLIP_BASS_SHIFT
	#define BLIP_BASS_SHIFT 9
#endif

/* ------------------------------------------------------------------------
 *
 * 16 taps x 32 phases, designed by constrained weighted least squares:
 * flat passband (-0.2 dB at 0.3 x sample rate), stopband weighted by
 * 1/sin^2(pi f) because aliases that fold to low frequencies get amplified
 * when the buffer is integrated. Every phase sums to exactly 32768 and has
 * its centroid exactly on the event. Phase p mirrors phase 32-p.
 *
 * Layout: blip_kernel[p][2*i] is tap i of phase p and blip_kernel[p][2*i+1]
 * is tap i of phase p+1. With the two blend endpoints side by side, one
 * pmaddwd instruction handles four taps, and each row is exactly one
 * 64-byte cache line.
 */
#if defined(__GNUC__)
	#define BLIP_ALIGN64 __attribute__((aligned(64)))
	#define BLIP_KERNEL_ALIGNED 1
#elif defined(_MSC_VER)
	#define BLIP_ALIGN64_PRE __declspec(align(64))
	#define BLIP_KERNEL_ALIGNED 1
#endif
#ifndef BLIP_ALIGN64
	#define BLIP_ALIGN64
#endif
#ifndef BLIP_ALIGN64_PRE
	#define BLIP_ALIGN64_PRE
#endif

/* constrained WLS design: passband 0..0.250 (weight 10), 16 taps, 32 phases */
BLIP_ALIGN64_PRE static short const blip_kernel [32] [32] BLIP_ALIGN64 =
{
	{ /* phase  0 */
		   -23,   -20,    46,    33,   100,   130,  -711,  -746,
		  2013,  1994, -3828, -3606,  5480,  4647, 26614, 26585,
		  5480,  6334, -3828, -4034,  2013,  2019,  -711,  -670,
		   100,    68,    46,    60,   -23,   -30,     0,     4
	},
	{ /* phase  1 */
		   -20,   -17,    33,    20,   130,   157,  -746,  -774,
		  1994,  1962, -3606, -3370,  4647,  3839, 26585, 26496,
		  6334,  7208, -4034, -4221,  2019,  2011,  -670,  -621,
		    68,    34,    60,    74,   -30,   -34,     4,     4
	},
	{ /* phase  2 */
		   -17,   -14,    20,     8,   157,   181,  -774,  -795,
		  1962,  1918, -3370, -3122,  3839,  3058, 26496, 26349,
		  7208,  8099, -4221, -4390,  2011,  1989,  -621,  -566,
		    34,    -3,    74,    89,   -34,   -38,     4,     5
	},
	{ /* phase  3 */
		   -14,   -12,     8,    -2,   181,   202,  -795,  -810,
		  1918,  1863, -3122, -2864,  3058,  2307, 26349, 26143,
		  8099,  9002, -4390, -4534,  1989,  1951,  -566,  -504,
		    -3,   -42,    89,   104,   -38,   -41,     5,     5
	},
	{ /* phase  4 */
		   -12,    -9,    -2,   -13,   202,   221,  -810,  -818,
		  1863,  1798, -2864, -2598,  2307,  1586, 26143, 25879,
		  9002,  9916, -4534, -4655,  1951,  1898,  -504,  -435,
		   -42,   -82,   104,   120,   -41,   -46,     5,     6
	},
	{ /* phase  5 */
		    -9,    -7,   -13,   -22,   221,   237,  -818,  -820,
		  1798,  1723, -2598, -2327,  1586,   899, 25879, 25560,
		  9916, 10837, -4655, -4750,  1898,  1830,  -435,  -360,
		   -82,  -124,   120,   135,   -46,   -49,     6,     6
	},
	{ /* phase  6 */
		    -7,    -5,   -22,   -31,   237,   251,  -820,  -816,
		  1723,  1640, -2327, -2052,   899,   246, 25560, 25186,
		 10837, 11761, -4750, -4815,  1830,  1745,  -360,  -278,
		  -124,  -169,   135,   151,   -49,   -53,     6,     7
	},
	{ /* phase  7 */
		    -5,    -3,   -31,   -38,   251,   261,  -816,  -807,
		  1640,  1550, -2052, -1776,   246,  -370, 25186, 24758,
		 11761, 12686, -4815, -4851,  1745,  1645,  -278,  -190,
		  -169,  -213,   151,   165,   -53,   -56,     7,     7
	},
	{ /* phase  8 */
		    -3,    -1,   -38,   -45,   261,   270,  -807,  -792,
		  1550,  1452, -1776, -1500,  -370,  -948, 24758, 24279,
		 12686, 13607, -4851, -4855,  1645,  1528,  -190,   -97,
		  -213,  -259,   165,   180,   -56,   -59,     7,     8
	},
	{ /* phase  9 */
		    -1,     1,   -45,   -51,   270,   276,  -792,  -773,
		  1452,  1349, -1500, -1226,  -948, -1488, 24279, 23751,
		 13607, 14522, -4855, -4826,  1528,  1396,   -97,     1,
		  -259,  -305,   180,   195,   -59,   -61,     8,     7
	},
	{ /* phase 10 */
		     1,     2,   -51,   -56,   276,   279,  -773,  -749,
		  1349,  1242, -1226,  -955, -1488, -1988, 23751, 23176,
		 14522, 15427, -4826, -4762,  1396,  1248,     1,   103,
		  -305,  -352,   195,   209,   -61,   -64,     7,     8
	},
	{ /* phase 11 */
		     2,     3,   -56,   -60,   279,   280,  -749,  -722,
		  1242,  1131,  -955,  -689, -1988, -2447, 23176, 22555,
		 15427, 16319, -4762, -4662,  1248,  1085,   103,   210,
		  -352,  -399,   209,   222,   -64,   -66,     8,     8
	},
	{ /* phase 12 */
		     3,     5,   -60,   -63,   280,   278,  -722,  -690,
		  1131,  1016,  -689,  -430, -2447, -2867, 22555, 21893,
		 16319, 17193, -4662, -4524,  1085,   907,   210,   320,
		  -399,  -445,   222,   234,   -66,   -67,     8,     8
	},
	{ /* phase 13 */
		     5,     6,   -63,   -66,   278,   275,  -690,  -655,
		  1016,   900,  -430,  -180, -2867, -3244, 21893, 21190,
		 17193, 18047, -4524, -4348,   907,   715,   320,   434,
		  -445,  -490,   234,   245,   -67,   -69,     8,     8
	},
	{ /* phase 14 */
		     6,     7,   -66,   -68,   275,   270,  -655,  -617,
		   900,   783,  -180,    61, -3244, -3581, 21190, 20452,
		 18047, 18876, -4348, -4133,   715,   509,   434,   549,
		  -490,  -534,   245,   255,   -69,   -68,     8,     7
	},
	{ /* phase 15 */
		     7,     7,   -68,   -68,   270,   263,  -617,  -577,
		   783,   666,    61,   291, -3581, -3877, 20452, 19679,
		 18876, 19679, -4133, -3877,   509,   291,   549,   666,
		  -534,  -577,   255,   263,   -68,   -68,     7,     7
	},
	{ /* phase 16 */
		     7,     7,   -68,   -68,   263,   255,  -577,  -534,
		   666,   549,   291,   509, -3877, -4133, 19679, 18876,
		 19679, 20452, -3877, -3581,   291,    61,   666,   783,
		  -577,  -617,   263,   270,   -68,   -68,     7,     7
	},
	{ /* phase 17 */
		     7,     8,   -68,   -69,   255,   245,  -534,  -490,
		   549,   434,   509,   715, -4133, -4348, 18876, 18047,
		 20452, 21190, -3581, -3244,    61,  -180,   783,   900,
		  -617,  -655,   270,   275,   -68,   -66,     7,     6
	},
	{ /* phase 18 */
		     8,     8,   -69,   -67,   245,   234,  -490,  -445,
		   434,   320,   715,   907, -4348, -4524, 18047, 17193,
		 21190, 21893, -3244, -2867,  -180,  -430,   900,  1016,
		  -655,  -690,   275,   278,   -66,   -63,     6,     5
	},
	{ /* phase 19 */
		     8,     8,   -67,   -66,   234,   222,  -445,  -399,
		   320,   210,   907,  1085, -4524, -4662, 17193, 16319,
		 21893, 22555, -2867, -2447,  -430,  -689,  1016,  1131,
		  -690,  -722,   278,   280,   -63,   -60,     5,     3
	},
	{ /* phase 20 */
		     8,     8,   -66,   -64,   222,   209,  -399,  -352,
		   210,   103,  1085,  1248, -4662, -4762, 16319, 15427,
		 22555, 23176, -2447, -1988,  -689,  -955,  1131,  1242,
		  -722,  -749,   280,   279,   -60,   -56,     3,     2
	},
	{ /* phase 21 */
		     8,     7,   -64,   -61,   209,   195,  -352,  -305,
		   103,     1,  1248,  1396, -4762, -4826, 15427, 14522,
		 23176, 23751, -1988, -1488,  -955, -1226,  1242,  1349,
		  -749,  -773,   279,   276,   -56,   -51,     2,     1
	},
	{ /* phase 22 */
		     7,     8,   -61,   -59,   195,   180,  -305,  -259,
		     1,   -97,  1396,  1528, -4826, -4855, 14522, 13607,
		 23751, 24279, -1488,  -948, -1226, -1500,  1349,  1452,
		  -773,  -792,   276,   270,   -51,   -45,     1,    -1
	},
	{ /* phase 23 */
		     8,     7,   -59,   -56,   180,   165,  -259,  -213,
		   -97,  -190,  1528,  1645, -4855, -4851, 13607, 12686,
		 24279, 24758,  -948,  -370, -1500, -1776,  1452,  1550,
		  -792,  -807,   270,   261,   -45,   -38,    -1,    -3
	},
	{ /* phase 24 */
		     7,     7,   -56,   -53,   165,   151,  -213,  -169,
		  -190,  -278,  1645,  1745, -4851, -4815, 12686, 11761,
		 24758, 25186,  -370,   246, -1776, -2052,  1550,  1640,
		  -807,  -816,   261,   251,   -38,   -31,    -3,    -5
	},
	{ /* phase 25 */
		     7,     6,   -53,   -49,   151,   135,  -169,  -124,
		  -278,  -360,  1745,  1830, -4815, -4750, 11761, 10837,
		 25186, 25560,   246,   899, -2052, -2327,  1640,  1723,
		  -816,  -820,   251,   237,   -31,   -22,    -5,    -7
	},
	{ /* phase 26 */
		     6,     6,   -49,   -46,   135,   120,  -124,   -82,
		  -360,  -435,  1830,  1898, -4750, -4655, 10837,  9916,
		 25560, 25879,   899,  1586, -2327, -2598,  1723,  1798,
		  -820,  -818,   237,   221,   -22,   -13,    -7,    -9
	},
	{ /* phase 27 */
		     6,     5,   -46,   -41,   120,   104,   -82,   -42,
		  -435,  -504,  1898,  1951, -4655, -4534,  9916,  9002,
		 25879, 26143,  1586,  2307, -2598, -2864,  1798,  1863,
		  -818,  -810,   221,   202,   -13,    -2,    -9,   -12
	},
	{ /* phase 28 */
		     5,     5,   -41,   -38,   104,    89,   -42,    -3,
		  -504,  -566,  1951,  1989, -4534, -4390,  9002,  8099,
		 26143, 26349,  2307,  3058, -2864, -3122,  1863,  1918,
		  -810,  -795,   202,   181,    -2,     8,   -12,   -14
	},
	{ /* phase 29 */
		     5,     4,   -38,   -34,    89,    74,    -3,    34,
		  -566,  -621,  1989,  2011, -4390, -4221,  8099,  7208,
		 26349, 26496,  3058,  3839, -3122, -3370,  1918,  1962,
		  -795,  -774,   181,   157,     8,    20,   -14,   -17
	},
	{ /* phase 30 */
		     4,     4,   -34,   -30,    74,    60,    34,    68,
		  -621,  -670,  2011,  2019, -4221, -4034,  7208,  6334,
		 26496, 26585,  3839,  4647, -3370, -3606,  1962,  1994,
		  -774,  -746,   157,   130,    20,    33,   -17,   -20
	},
	{ /* phase 31 */
		     4,     0,   -30,   -23,    60,    46,    68,   100,
		  -670,  -711,  2019,  2013, -4034, -3828,  6334,  5480,
		 26585, 26614,  4647,  5480, -3606, -3828,  1994,  2013,
		  -746,  -711,   130,   100,    33,    46,   -20,   -23
	}
};

struct blip_t
{
	blip_pos_t factor;      /* output samples per input clock (fixed point) */
	blip_pos_t offset;      /* fractional sample position of clock 0        */
	int        avail;       /* complete samples ready to be read             */
	int        size;        /* capacity in samples                          */
	int        integrator;  /* running sum, scaled by 1 << BLIP_KERNEL_BITS */
};

#define BLIP_SLOTS( m ) ((int*) ((m) + 1))

#ifndef NDEBUG
static void blip_check_platform( void )
{
	/* The code relies on arithmetic (sign-extending) right shifts of
	   negative ints. C90 leaves this implementation-defined, but every
	   mainstream compiler behaves this way. */
	int const minus_three = -3;
	assert( (minus_three >> 1) == -2 );
	assert( (blip_pos_t) (blip_max_frame + BLIP_LATE + 1) <
			((blip_pos_t) -1 >> BLIP_POS_FRAC) );
	assert( BLIP_POS_ONE / blip_max_ratio >= 1 );
}
#endif

blip_t* blip_new( int sample_count )
{
	blip_t* m;
	size_t slots;

	assert( sample_count >= 0 );
	if ( sample_count < 0 )
		return NULL;

	slots = (size_t) sample_count + BLIP_PAD;
	if ( slots > ((size_t) -1 - sizeof (blip_t)) / sizeof (int) )
		return NULL;

	m = (blip_t*) malloc( sizeof (blip_t) + slots * sizeof (int) );
	if ( m == NULL )
		return NULL;

#ifndef NDEBUG
	blip_check_platform();
#endif

	m->size   = sample_count;
	m->factor = BLIP_POS_ONE / blip_max_ratio;
	blip_clear( m );
	return m;
}

void blip_delete( blip_t* m )
{
	if ( m != NULL )
	{
		/* Make use-after-free bugs fail loudly rather than subtly. */
		memset( m, 0, sizeof (blip_t) );
		free( m );
	}
}

void blip_set_rates( blip_t* m, double clock_rate, double sample_rate )
{
	double const f = (double) BLIP_POS_ONE * sample_rate / clock_rate;

	/* f < 1 means clock_rate / sample_rate exceeds blip_max_ratio. The
	   upper limit (1024 samples per clock) just keeps factor in range. */
	assert( f >= 1.0 && f <= (double) BLIP_POS_ONE * 1024.0 );

	/* Round to nearest; the frame arithmetic below stays exact whichever
	   way the last bit falls, so nearest gives the best long-term rate. */
	m->factor = (blip_pos_t) (f + 0.5);
	if ( m->factor == 0 )
		m->factor = 1;
}

void blip_clear( blip_t* m )
{
	m->offset     = 0;
	m->avail      = 0;
	m->integrator = 0;
	memset( BLIP_SLOTS( m ), 0, ((size_t) m->size + BLIP_PAD) * sizeof (int) );
}

int blip_clocks_needed( const blip_t* m, int sample_count )
{
	blip_pos_t target;

	assert( sample_count >= 0 && m->avail + sample_count <= m->size );
	/* sample_count << BLIP_POS_FRAC must not overflow */
	assert( (blip_pos_t) sample_count <= ((blip_pos_t) -1 >> BLIP_POS_FRAC) );

	target = (blip_pos_t) sample_count << BLIP_POS_FRAC;
	if ( target <= m->offset )
		return 0;

	/* Smallest whole clock count c with c * factor + offset >= target. */
	return (int) ((target - m->offset + m->factor - 1) / m->factor);
}

void blip_end_frame( blip_t* m, unsigned int clock_duration )
{
	blip_pos_t const end = (blip_pos_t) clock_duration * m->factor + m->offset;

	m->avail  += (int) (end >> BLIP_POS_FRAC);
	m->offset  = end & (BLIP_POS_ONE - 1);

	/* Too many samples for the buffer: read more often or enlarge it. */
	assert( m->avail <= m->size );
}

int blip_samples_avail( const blip_t* m )
{
	return m->avail;
}

int blip_read_samples( blip_t* m, short out [], int count, int stereo )
{
	assert( count >= 0 );

	if ( count > m->avail )
		count = m->avail;

	if ( count > 0 )
	{
		int* const buf   = BLIP_SLOTS( m );
		int  const step  = stereo ? 2 : 1;
		int  const left  = m->avail - count;
		int  const start = m->integrator;
		int  sum = start;
		unsigned range = 0;
		short* o = out;
		int  i;

		for ( i = 0; i < count; ++i )
		{
			int const s = sum >> BLIP_KERNEL_BITS;
			sum = sum + buf [i] - (sum >> BLIP_BASS_SHIFT);
			range |= (unsigned) s + 32768u;
			*o = (short) s;
			o += step;
		}

		if ( range > 65535u )
		{
			/* Something clipped: redo the block with saturation. */
			sum = start;
			o = out;
			for ( i = 0; i < count; ++i )
			{
				int s = sum >> BLIP_KERNEL_BITS;
				sum = sum + buf [i] - (sum >> BLIP_BASS_SHIFT);
				if ( s > 32767 )
					s = 32767;
				else if ( s < -32768 )
					s = -32768;
				*o = (short) s;
				o += step;
			}
		}
		m->integrator = sum;

		/* Shift remaining samples plus the pending-delta tail down. */
		memmove( buf, buf + count, ((size_t) left + BLIP_PAD) * sizeof (int) );
		memset( buf + left + BLIP_PAD, 0, (size_t) count * sizeof (int) );
		m->avail = left;
	}

	return count;
}

#if defined(BLIP_USE_SSE2)
	#if defined(BLIP_KERNEL_ALIGNED)
		#define BLIP_LOAD_KERNEL _mm_load_si128
	#else
		#define BLIP_LOAD_KERNEL _mm_loadu_si128
	#endif
#endif

/* Converts a clock time into a 32-bit value: whole samples above bit 20,
   sub-sample fraction below it. Frames are at most a few thousand samples,
   so the whole part always fits in the remaining 12 bits. */
#define BLIP_LOCATE( m, time ) \
	((unsigned) (((blip_pos_t) (time) * (m)->factor + (m)->offset) \
			>> (BLIP_POS_FRAC - BLIP_SUB_BITS)))

void blip_add_delta( blip_t* m, unsigned int time, int delta )
{
	unsigned const pos  = BLIP_LOCATE( m, time );
	int*     const out  = BLIP_SLOTS( m ) + m->avail + (int) (pos >> BLIP_SUB_BITS);
	short const* const k =
			blip_kernel [(pos >> BLIP_INTERP_BITS) & (BLIP_PHASES - 1)];

	/* Split delta between the two neighbouring phases. d1 + d2 == delta
	   exactly, so every delta adds precisely delta << 15 to the buffer. */
	int const blend = (int) (pos & BLIP_INTERP_MASK);
	int const d2    = (delta * blend) >> BLIP_INTERP_BITS;
	int const d1    = delta - d2;

	assert( delta >= -65535 && delta <= 65535 );
	assert( out <= BLIP_SLOTS( m ) + m->size + BLIP_LATE ); /* frame overran buffer */

#if defined(BLIP_USE_SSE2)
	/* When both halves fit in 16 bits (always true for |delta| <= 32767),
	   one pmaddwd yields k[2i]*d1 + k[2i+1]*d2 for four taps at once,
	   exactly as the scalar code computes it. (A 256-bit AVX2 version
	   measured no faster: the loop is bound by store-to-load forwarding
	   between neighbouring deltas, not by arithmetic.) */
	if ( (unsigned) (delta + 32767) <= 65534u )
	{
		int const pair = (int) (((unsigned) d2 << 16) | ((unsigned) d1 & 0xFFFFu));
		__m128i const w  = _mm_set1_epi32( pair );
		__m128i const k0 = BLIP_LOAD_KERNEL( (__m128i const*) (k     ) );
		__m128i const k1 = BLIP_LOAD_KERNEL( (__m128i const*) (k +  8) );
		__m128i const k2 = BLIP_LOAD_KERNEL( (__m128i const*) (k + 16) );
		__m128i const k3 = BLIP_LOAD_KERNEL( (__m128i const*) (k + 24) );
		__m128i const o0 = _mm_loadu_si128( (__m128i const*) (out     ) );
		__m128i const o1 = _mm_loadu_si128( (__m128i const*) (out +  4) );
		__m128i const o2 = _mm_loadu_si128( (__m128i const*) (out +  8) );
		__m128i const o3 = _mm_loadu_si128( (__m128i const*) (out + 12) );
		_mm_storeu_si128( (__m128i*) (out     ), _mm_add_epi32( o0, _mm_madd_epi16( k0, w ) ) );
		_mm_storeu_si128( (__m128i*) (out +  4), _mm_add_epi32( o1, _mm_madd_epi16( k1, w ) ) );
		_mm_storeu_si128( (__m128i*) (out +  8), _mm_add_epi32( o2, _mm_madd_epi16( k2, w ) ) );
		_mm_storeu_si128( (__m128i*) (out + 12), _mm_add_epi32( o3, _mm_madd_epi16( k3, w ) ) );
		return;
	}
#endif

	/* Portable path (also used for very large deltas). Fully unrolled;
	   the contiguous layout lets compilers vectorize it on other targets. */
	#define BLIP_TAP( i ) out [i] += k [2*(i)] * d1 + k [2*(i)+1] * d2
	BLIP_TAP(  0 ); BLIP_TAP(  1 ); BLIP_TAP(  2 ); BLIP_TAP(  3 );
	BLIP_TAP(  4 ); BLIP_TAP(  5 ); BLIP_TAP(  6 ); BLIP_TAP(  7 );
	BLIP_TAP(  8 ); BLIP_TAP(  9 ); BLIP_TAP( 10 ); BLIP_TAP( 11 );
	BLIP_TAP( 12 ); BLIP_TAP( 13 ); BLIP_TAP( 14 ); BLIP_TAP( 15 );
	#undef BLIP_TAP
}

void blip_add_delta_fast( blip_t* m, unsigned int time, int delta )
{
	unsigned const pos = BLIP_LOCATE( m, time );
	int*     const out = BLIP_SLOTS( m ) + m->avail + (int) (pos >> BLIP_SUB_BITS);

	/* Linear interpolation between the two slots around the delta, using
	   the top 15 fraction bits. Same centre as blip_add_delta(). */
	int const frac = (int) ((pos >> (BLIP_SUB_BITS - BLIP_KERNEL_BITS)) &
			((1 << BLIP_KERNEL_BITS) - 1));
	int const late = delta * frac;

	assert( delta >= -65535 && delta <= 65535 );
	assert( out <= BLIP_SLOTS( m ) + m->size + BLIP_LATE );

	out [BLIP_CENTER    ] += delta * (1 << BLIP_KERNEL_BITS) - late;
	out [BLIP_CENTER + 1] += late;
}
