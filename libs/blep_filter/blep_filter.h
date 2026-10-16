/*
  Band-limited resampling sample buffer

  An independent, MIT-licensed implementation of the blip_buf API.
  The function names, signatures and constants are kept identical to the
  classic blip_buf interface so that this file can be used as a drop-in
  replacement. All code, comments and filter tables are original.

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

#ifndef BLIP_BUF_H
#define BLIP_BUF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque buffer object. Every function takes one as its first argument. */
typedef struct blip_t blip_t;

/* Highest supported ratio of input clock rate to output sample rate. */
enum { blip_max_ratio = 1 << 20 };

/* Largest number of output samples a single time frame may produce. */
enum { blip_max_frame = 4000 };

/* Allocates a buffer able to hold up to sample_count output samples.
 * The initial rate is blip_max_ratio clocks per sample.
 * Returns NULL if memory could not be allocated. */
blip_t* blip_new( int sample_count );

/* Configures the conversion so that clock_rate input clocks produce
 * (approximately) sample_rate output samples. clock_rate must not exceed
 * sample_rate * blip_max_ratio. */
void blip_set_rates( blip_t*, double clock_rate, double sample_rate );

/* Discards all buffered samples and pending deltas and resets the filter
 * state. Afterwards blip_samples_avail() returns 0. */
void blip_clear( blip_t* );

/* Adds an amplitude change of 'delta' at input clock 'clock_time',
 * measured from the start of the current time frame. The change is
 * synthesized as a band-limited step. |delta| must not exceed 65535. */
void blip_add_delta( blip_t*, unsigned int clock_time, int delta );

/* Same as blip_add_delta(), but uses cheap linear interpolation instead of
 * full band-limited synthesis. Faster, with more aliasing. */
void blip_add_delta_fast( blip_t*, unsigned int clock_time, int delta );

/* Returns the number of input clocks the current frame must last for
 * sample_count more samples to become available. */
int blip_clocks_needed( const blip_t*, int sample_count );

/* Ends the current time frame after clock_duration clocks, making the
 * samples it covers readable. A new frame starts at that point, so clock 0
 * of the next frame is clock clock_duration of the old one. Deltas may be
 * placed slightly beyond clock_duration (by up to two output samples'
 * worth of clocks). */
void blip_end_frame( blip_t*, unsigned int clock_duration );

/* Returns the number of samples currently available for reading. */
int blip_samples_avail( const blip_t* );

/* Reads and removes up to 'count' samples, writing 16-bit signed output to
 * 'out'. If 'stereo' is non-zero, only every other element of 'out' is
 * written, so two buffers can be interleaved into one stereo stream.
 * Returns the number of samples actually read. */
int blip_read_samples( blip_t*, short out [], int count, int stereo );

/* Releases a buffer. Passing NULL is allowed and does nothing. */
void blip_delete( blip_t* );

/* Older name, kept for source compatibility. */
typedef blip_t blip_buffer_t;

#ifdef __cplusplus
}
#endif

#endif /* BLIP_BUF_H */
