/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    ymfm shared ADPCM engines

    C port of ymfm's adpcm_a_engine and adpcm_b_engine (ymfm_adpcm.h,
    ymfm_adpcm.cpp).
    https://github.com/aaronsgiles/ymfm

    Port notes:
    - ymfm reads the sample memory through its host interface. Here the
      chip gives each engine a pointer to its memory and the size; reads
      outside it return 0, writes outside it are dropped.
    - Muting is up to the chip: it leaves channels out of the ADPCM-A
      chanmask, or skips the ADPCM-B output.
    - Change from ymfm: the ADPCM-A channels are silent while no sample
      memory is set (mem_size 0). ymfm would decode zero bytes into a
      sawtooth (YM2610 songs without ADPCM-A data).

***************************************************************************/

#include <string.h>	// for memset

#include "ymfm_fm.h"	// for bitfield() and clamp()
#include "ymfm_adpcm.h"


//----------------------------------------------------------------------------
// ADPCM-A
//----------------------------------------------------------------------------

// ADPCM-A step sizes
static const UINT16 s_adpcm_a_steps[49] =
{
	 16,  17,   19,   21,   23,   25,   28,
	 31,  34,   37,   41,   45,   50,   55,
	 60,  66,   73,   80,   88,   97,  107,
	118, 130,  143,  157,  173,  190,  209,
	230, 253,  279,  307,  337,  371,  408,
	449, 494,  544,  598,  658,  724,  796,
	876, 963, 1060, 1166, 1282, 1411, 1552
};

// ADPCM-A step index adjustments
static const INT8 s_adpcm_a_step_inc[8] = { -1, -1, -1, -1, 2, 5, 7, 9 };

#define ADPCMA_REG(offset)	(a->regdata[offset])

#define adpcma_total_level()		bitfield(ADPCMA_REG(0x01), 0, 6)
#define adpcma_ch_pan_left(ch)		bitfield(ADPCMA_REG((ch) + 0x08), 7, 1)
#define adpcma_ch_pan_right(ch)		bitfield(ADPCMA_REG((ch) + 0x08), 6, 1)
#define adpcma_ch_instrument_level(ch)	bitfield(ADPCMA_REG((ch) + 0x08), 0, 5)
#define adpcma_ch_start(ch)	((UINT32)(ADPCMA_REG((ch) + 0x10) | (ADPCMA_REG((ch) + 0x18) << 8)))
#define adpcma_ch_end(ch)	((UINT32)(ADPCMA_REG((ch) + 0x20) | (ADPCMA_REG((ch) + 0x28) << 8)))

#define adpcm_a_mem_read(a, address)	(((address) < (a)->mem_size) ? (a)->mem[address] : 0x00)

void adpcm_a_init(ADPCM_A_ENGINE* a, UINT32 shift)
{
	memset(a, 0x00, sizeof(ADPCM_A_ENGINE));
	a->shift = shift;
	return;
}

void adpcm_a_reset(ADPCM_A_ENGINE* a)
{
	memset(a->regdata, 0x00, ADPCM_A_REGISTERS);

	// initialize the pans to on by default, and max instrument volume;
	// some neogeo homebrews (for example ffeast) rely on this
	ADPCMA_REG(0x08) = ADPCMA_REG(0x09) = ADPCMA_REG(0x0a) =
	ADPCMA_REG(0x0b) = ADPCMA_REG(0x0c) = ADPCMA_REG(0x0d) = 0xdf;

	memset(a->channel, 0x00, sizeof(a->channel));
	return;
}

void adpcm_a_set_start_end(ADPCM_A_ENGINE* a, UINT8 chnum, UINT16 start, UINT16 end)
{
	ADPCMA_REG(chnum + 0x10) = (UINT8)start;
	ADPCMA_REG(chnum + 0x18) = (UINT8)(start >> 8);
	ADPCMA_REG(chnum + 0x20) = (UINT8)end;
	ADPCMA_REG(chnum + 0x28) = (UINT8)(end >> 8);
	return;
}

// signal key on/off
static void adpcm_a_keyonoff(ADPCM_A_ENGINE* a, UINT8 chnum, UINT8 on)
{
	ADPCM_A_CHANNEL* ch = &a->channel[chnum];

	ch->playing = on;
	if (ch->playing)
	{
		ch->curaddress = adpcma_ch_start(chnum) << a->shift;
		ch->curnibble = 0;
		ch->curbyte = 0;
		ch->accumulator = 0;
		ch->step_index = 0;
	}
	return;
}

// clock one channel; returns 1 when the sample just ended
static UINT8 adpcm_a_channel_clock(ADPCM_A_ENGINE* a, UINT8 chnum)
{
	ADPCM_A_CHANNEL* ch = &a->channel[chnum];
	UINT32 end;
	UINT8 data;
	INT32 delta;

	// if not playing, just output 0
	if (ch->playing == 0)
	{
		ch->accumulator = 0;
		return 0;
	}

	// if we're about to read nibble 0, fetch the data
	if (ch->curnibble == 0)
	{
		// stop when we hit the end address; apparently only low 20 bits are used for
		// comparison on the YM2610: this affects sample playback in some games, for
		// example twinspri character select screen music will skip some samples if
		// this is not correct
		//
		// note also: end address is inclusive, so wait until we are about to fetch
		// the sample just after the end before stopping; this is needed for nitd's
		// jump sound, for example
		end = (adpcma_ch_end(chnum) + 1) << a->shift;
		if (((ch->curaddress ^ end) & 0xfffff) == 0)
		{
			ch->playing = 0;
			ch->accumulator = 0;
			return 1;
		}

		ch->curbyte = adpcm_a_mem_read(a, ch->curaddress);
		ch->curaddress ++;
		data = (UINT8)(ch->curbyte >> 4);
		ch->curnibble = 1;
	}
	// otherwise just extract from the previosuly-fetched byte
	else
	{
		data = (UINT8)(ch->curbyte & 0xf);
		ch->curnibble = 0;
	}

	// compute the ADPCM delta
	delta = (INT32)(2 * bitfield(data, 0, 3) + 1) * s_adpcm_a_steps[ch->step_index] / 8;
	if (bitfield(data, 3, 1))
		delta = -delta;

	// the 12-bit accumulator wraps on the ym2610 and ym2608 (like the msm5205)
	ch->accumulator = (ch->accumulator + delta) & 0xfff;

	// adjust ADPCM step
	ch->step_index = clamp(ch->step_index + s_adpcm_a_step_inc[bitfield(data, 0, 3)], 0, 48);
	return 0;
}

UINT32 adpcm_a_clock(ADPCM_A_ENGINE* a, UINT32 chanmask)
{
	UINT32 result = 0;
	UINT8 chnum;

	for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1) && adpcm_a_channel_clock(a, chnum))
			result |= 1 << chnum;
	}
	return result;
}

void adpcm_a_output(const ADPCM_A_ENGINE* a, INT32* output, UINT32 chanmask)
{
	UINT8 chnum;
	int vol;
	INT8 mul;
	UINT8 shift;
	INT16 value;

	// without sample memory there is nothing to play
	if (a->mem_size == 0)
		return;

	for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
	{
		if (! bitfield(chanmask, chnum, 1))
			continue;

		// volume combines instrument and total levels
		vol = (int)((adpcma_ch_instrument_level(chnum) ^ 0x1f) + (adpcma_total_level() ^ 0x3f));

		// if combined is maximum, don't add to outputs
		if (vol >= 63)
			continue;

		// convert into a shift and a multiplier
		// QUESTION: verify this from other sources
		mul = (INT8)(15 - (vol & 7));
		shift = (UINT8)(4 + 1 + (vol >> 3));

		// m_accumulator is a 12-bit value; shift up to sign-extend;
		// the downshift is incorporated into 'shift'
		value = (INT16)((((INT16)(a->channel[chnum].accumulator << 4)) * mul >> shift) & ~3);

		// apply to left/right as appropriate
		if (adpcma_ch_pan_left(chnum))
			output[0] += value;
		if (adpcma_ch_pan_right(chnum))
			output[1] += value;
	}
	return;
}

void adpcm_a_write(ADPCM_A_ENGINE* a, UINT32 regnum, UINT8 data)
{
	UINT8 chnum;

	// store the raw value to the register array;
	// most writes are passive, consumed only when needed
	ADPCMA_REG(regnum) = data;

	// actively handle writes to the control register
	if (regnum == 0x00)
	{
		for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
		{
			if (bitfield(data, chnum, 1))
				adpcm_a_keyonoff(a, chnum, (UINT8)bitfield(~data, 7, 1));
		}
	}
	return;
}


//----------------------------------------------------------------------------
// ADPCM-B
//----------------------------------------------------------------------------

#define ADPCM_B_STEP_MIN	127
#define ADPCM_B_STEP_MAX	24576

// ADPCM-B step scale: 0.9, 0.9, 0.9, 0.9, 1.2, 1.6, 2.0, 2.4
static const UINT8 s_adpcm_b_step_scale[8] = { 57, 57, 57, 57, 77, 102, 128, 153 };

#define ADPCM_REG(offset)	(b->regdata[offset])

#define adpcm_execute()		bitfield(ADPCM_REG(0x00), 7, 1)
#define adpcm_record()		bitfield(ADPCM_REG(0x00), 6, 1)
#define adpcm_external()	bitfield(ADPCM_REG(0x00), 5, 1)
#define adpcm_repeat()		bitfield(ADPCM_REG(0x00), 4, 1)
#define adpcm_resetflag()	bitfield(ADPCM_REG(0x00), 0, 1)
#define adpcm_pan_left()	bitfield(ADPCM_REG(0x01), 7, 1)
#define adpcm_pan_right()	bitfield(ADPCM_REG(0x01), 6, 1)
#define adpcm_dram_8bit()	bitfield(ADPCM_REG(0x01), 1, 1)
#define adpcm_rom_ram()		bitfield(ADPCM_REG(0x01), 0, 1)
#define adpcm_start()	((UINT32)(ADPCM_REG(0x02) | (ADPCM_REG(0x03) << 8)))
#define adpcm_end()		((UINT32)(ADPCM_REG(0x04) | (ADPCM_REG(0x05) << 8)))
#define adpcm_cpudata()		ADPCM_REG(0x08)
#define adpcm_delta_n()	((UINT32)(ADPCM_REG(0x09) | (ADPCM_REG(0x0a) << 8)))
#define adpcm_level()		ADPCM_REG(0x0b)
#define adpcm_limit()	((UINT32)(ADPCM_REG(0x0c) | (ADPCM_REG(0x0d) << 8)))

#define adpcm_b_mem_read(b, address)	(((address) < (b)->mem_size) ? (b)->mem[address] : 0x00)

static void adpcm_b_mem_write(ADPCM_B_ENGINE* b, UINT32 address, UINT8 data)
{
	if (address < b->mem_size)
		b->mem[address] = data;
	return;
}

// reset the channel state
static void adpcm_b_channel_reset(ADPCM_B_ENGINE* b)
{
	b->status = ADPCM_B_STATUS_BRDY;
	b->curnibble = 0;
	b->curbyte = 0;
	b->dummy_read = 0;
	b->position = 0;
	b->curaddress = 0;
	b->accumulator = 0;
	b->prev_accum = 0;
	b->adpcm_step = ADPCM_B_STEP_MIN;
	return;
}

void adpcm_b_init(ADPCM_B_ENGINE* b, UINT32 shift)
{
	memset(b, 0x00, sizeof(ADPCM_B_ENGINE));
	b->shift = shift;
	return;
}

void adpcm_b_reset(ADPCM_B_ENGINE* b)
{
	memset(b->regdata, 0x00, ADPCM_B_REGISTERS);

	// default limit to wide open
	ADPCM_REG(0x0c) = ADPCM_REG(0x0d) = 0xff;

	adpcm_b_channel_reset(b);
	return;
}

// compute the current address shift amount based on register settings
static UINT32 adpcm_b_address_shift(const ADPCM_B_ENGINE* b)
{
	// if a constant address shift, just provide that
	if (b->shift != 0)
		return b->shift;

	// if ROM or 8-bit DRAM, shift is 5 bits
	if (adpcm_rom_ram())
		return 5;
	if (adpcm_dram_8bit())
		return 5;

	// otherwise, shift is 2 bits
	return 2;
}

// limit checker; stops at the last byte of the chunk described by address_shift()
#define adpcm_b_at_limit(b)	((b)->curaddress == (((adpcm_limit() + 1) << adpcm_b_address_shift(b)) - 1))

// end checker; stops at the last byte of the chunk described by address_shift()
#define adpcm_b_at_end(b)	((b)->curaddress == (((adpcm_end() + 1) << adpcm_b_address_shift(b)) - 1))

// load the start address and initialize the state
static void adpcm_b_load_start(ADPCM_B_ENGINE* b)
{
	b->status = (b->status & ~ADPCM_B_STATUS_EOS) | ADPCM_B_STATUS_PLAYING;
	b->curaddress = adpcm_external() ? (adpcm_start() << adpcm_b_address_shift(b)) : 0;
	b->curnibble = 0;
	b->curbyte = 0;
	b->position = 0;
	b->accumulator = 0;
	b->prev_accum = 0;
	b->adpcm_step = ADPCM_B_STEP_MIN;
	return;
}

void adpcm_b_clock(ADPCM_B_ENGINE* b)
{
	UINT32 position;
	UINT8 data;
	INT32 delta;

	// only process if active and not recording (which we don't support)
	if (! adpcm_execute() || adpcm_record() || (b->status & ADPCM_B_STATUS_PLAYING) == 0)
	{
		b->status &= ~ADPCM_B_STATUS_PLAYING;
		return;
	}

	// otherwise, advance the step
	position = b->position + adpcm_delta_n();
	b->position = (UINT16)position;
	if (position < 0x10000)
		return;

	// if we're about to process nibble 0, fetch sample
	if (b->curnibble == 0)
	{
		// playing from RAM/ROM
		if (adpcm_external())
			b->curbyte = adpcm_b_mem_read(b, b->curaddress);
	}

	// extract the nibble from our current byte
	data = (UINT8)((UINT8)(b->curbyte << (4 * b->curnibble)) >> 4);
	b->curnibble ^= 1;

	// we just processed the last nibble
	if (b->curnibble == 0)
	{
		// if playing from RAM/ROM, check the end/limit address or advance
		if (adpcm_external())
		{
			// handle the sample end, either repeating or stopping
			if (adpcm_b_at_end(b))
			{
				// if repeating, go back to the start
				if (adpcm_repeat())
				{
					adpcm_b_load_start(b);
				}
				// otherwise, done; set the EOS bit
				else
				{
					b->accumulator = 0;
					b->prev_accum = 0;
					b->status = (b->status & ~ADPCM_B_STATUS_PLAYING) | ADPCM_B_STATUS_EOS;
					return;
				}
			}
			// wrap at the limit address
			else if (adpcm_b_at_limit(b))
			{
				b->curaddress = 0;
			}
			// otherwise, advance the current address
			else
			{
				b->curaddress ++;
				b->curaddress &= 0xffffff;
			}
		}
		// if CPU-driven, copy the next byte and request more
		else
		{
			b->curbyte = adpcm_cpudata();
			b->status |= ADPCM_B_STATUS_BRDY;
		}
	}

	// remember previous value for interpolation
	b->prev_accum = b->accumulator;

	// forecast to next forecast: 1/8, 3/8, 5/8, 7/8, 9/8, 11/8, 13/8, 15/8
	delta = (INT32)(2 * bitfield(data, 0, 3) + 1) * b->adpcm_step / 8;
	if (bitfield(data, 3, 1))
		delta = -delta;

	// add and clamp to 16 bits
	b->accumulator = clamp(b->accumulator + delta, -32768, 32767);

	// scale the ADPCM step
	b->adpcm_step = clamp((b->adpcm_step * s_adpcm_b_step_scale[bitfield(data, 0, 3)]) / 64,
		ADPCM_B_STEP_MIN, ADPCM_B_STEP_MAX);
	return;
}

void adpcm_b_output(const ADPCM_B_ENGINE* b, INT32* output, UINT32 outputs, UINT32 rshift)
{
	INT32 result;

	// do a linear interpolation between samples
	result = (b->prev_accum * (INT32)((b->position ^ 0xffff) + 1) + b->accumulator * (INT32)b->position) >> 16;

	// apply volume (level) in a linear fashion and reduce
	result = (result * (INT32)adpcm_level()) >> (8 + rshift);

	// apply to left/right
	if (outputs == 1 || adpcm_pan_left())
		output[0] += result;
	if (outputs >= 2 && adpcm_pan_right())
		output[1] += result;
	return;
}

UINT8 adpcm_b_read(ADPCM_B_ENGINE* b, UINT32 regnum)
{
	UINT8 result = 0;

	// register 8 reads over the bus under some conditions
	if (regnum == 0x08 && ! adpcm_execute() && ! adpcm_record() && adpcm_external())
	{
		// two dummy reads are consumed first
		if (b->dummy_read != 0)
		{
			adpcm_b_load_start(b);
			b->dummy_read --;
		}
		// read the data
		else
		{
			// read from outside of the chip
			result = adpcm_b_mem_read(b, b->curaddress);
			b->curaddress ++;

			// did we hit the end? if so, signal EOS
			if (adpcm_b_at_end(b))
				b->status = ADPCM_B_STATUS_EOS | ADPCM_B_STATUS_BRDY;
			// signal ready
			else
				b->status = ADPCM_B_STATUS_BRDY;

			// wrap at the limit address
			if (adpcm_b_at_limit(b))
				b->curaddress = 0;
		}
	}
	return result;
}

void adpcm_b_write(ADPCM_B_ENGINE* b, UINT32 regnum, UINT8 value)
{
	// store the raw value to the register array;
	// most writes are passive, consumed only when needed
	ADPCM_REG(regnum) = value;

	// register 0 can do a reset; also use writes here to reset the
	// dummy read counter
	if (regnum == 0x00)
	{
		if (adpcm_execute())
			adpcm_b_load_start(b);
		else
			b->status &= ~ADPCM_B_STATUS_EOS;
		if (adpcm_resetflag())
			adpcm_b_channel_reset(b);
		if (adpcm_external())
			b->dummy_read = 2;
	}
	// register 8 writes over the bus under some conditions
	else if (regnum == 0x08)
	{
		// if writing from the CPU during execute, clear the ready flag
		if (adpcm_execute() && ! adpcm_record() && ! adpcm_external())
		{
			b->status &= ~ADPCM_B_STATUS_BRDY;
		}
		// if writing during "record", pass through as data
		else if (! adpcm_execute() && adpcm_record() && adpcm_external())
		{
			// clear out dummy reads and set start address
			if (b->dummy_read != 0)
			{
				adpcm_b_load_start(b);
				b->dummy_read = 0;
			}

			// did we hit the end? if so, signal EOS
			if (adpcm_b_at_end(b))
			{
				b->status = ADPCM_B_STATUS_EOS | ADPCM_B_STATUS_BRDY;
			}
			// otherwise, write the data and signal ready
			else
			{
				adpcm_b_mem_write(b, b->curaddress, value);
				b->curaddress ++;
				b->status = ADPCM_B_STATUS_BRDY;
			}
		}
	}
	return;
}
