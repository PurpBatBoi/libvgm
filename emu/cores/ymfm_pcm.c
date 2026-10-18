/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    Yamaha OPL4 wavetable engine

    C port of ymfm's PCM engine (ymfm_pcm.h, ymfm_pcm.cpp).
    https://github.com/aaronsgiles/ymfm

    Used by the YMF278B (OPL4) in ymfm_opl.c.

    Port notes:
    - ymfm reads the sample memory through its host interface. The only
      user is the YMF278B, so the memory map is part of the engine: the ROM
      (the YRW801 is 2 MB) at address 0 and the SRAM of the MoonSound at
      0x200000 (after the ROM if that is bigger). The address space wraps
      at 4 MB. The memory type bit of register 2 (SRAM mapped to
      0x380000-0x3FFFFF) is not emulated. The chip owns the memory and sets
      the pointers.
    - Muted channels (mute_mask) are clocked but not added to the outputs.
    - Octave -8 of a channel is not defined by ymfm (negative shift); here
      it shifts the f-number right by one more bit than octave -7.
    - Change from ymfm: ymfm samples the key state once per sample, so a
      key off followed by a key on before the next sample was lost and the
      note did not restart. It also kept a key on that was followed by a
      key off. Here the live key state is kept, and a key off is remembered
      until the next sample, so the channel releases and then attacks
      again.

***************************************************************************/

#include <string.h>	// for memset

#include "ymfm_fm.h"	// for bitfield(), clamp(), attenuation_to_volume() and attenuation_increment()
#include "ymfm_pcm.h"


#define PCM_ALL_CHANNELS	((1 << PCM_CHANNELS) - 1)

// the address space wraps at 4 MB; the RAM starts at 2 MB, or after a bigger ROM
#define ADDRESS_MASK	0x3FFFFF
#define RAM_BASE		0x200000

// "quiet" value, used to optimize when we can skip doing working
#define PCM_EG_QUIET	0x200

// key states
#define KEY_ON			0x01
#define KEY_PENDING_ON	0x02
#define KEY_PENDING		0x04


//----------------------------------------------------------------------------
// tables
//----------------------------------------------------------------------------

// LFO stepping values, how much to add to a running x.18 value for the LFO
static const UINT8 s_lfo_steps[8] = { 1, 12, 19, 25, 31, 35, 37, 42 };

// AM LFO depth values; each has at most 2 bits to make the "multiply" easy
static const UINT8 s_am_depth[8] = { 0, 0x14, 0x20, 0x28, 0x30, 0x40, 0x50, 0x80 };

// PM LFO depth values, converted from cents into f-numbers
static const UINT8 s_pm_depth[8] = { 0, 2, 3, 4, 6, 12, 24, 48 };


//----------------------------------------------------------------------------
// helpers
//----------------------------------------------------------------------------

// sign-extend a 4-bit value
INLINE INT32 sign_extend_4(UINT32 value)
{
	return (value & 0x08) ? (INT32)value - 16 : (INT32)value;
}


//----------------------------------------------------------------------------
// memory
//----------------------------------------------------------------------------

// returns the RAM offset of an address, or (UINT32)-1 if it is not mapped
static UINT32 ram_offset(const PCM_ENGINE* pcm, UINT32 address)
{
	UINT32 base;
	UINT32 offset;

	base = (pcm->rom_size > RAM_BASE) ? pcm->rom_size : RAM_BASE;
	if (address < base)
		return (UINT32)-1;
	offset = address - base;

	// 640 KB are a 512 KB pcm and a 128 KB pcm; the 128 KB pcm is visible
	// four times in the upper part
	if (pcm->ram_size == 640 * 1024 && (offset & 0x080000))
		offset &= ~0x060000;
	return (offset < pcm->ram_size) ? offset : (UINT32)-1;
}

static UINT8 mem_read(const PCM_ENGINE* pcm, UINT32 address)
{
	UINT32 offset;

	address &= ADDRESS_MASK;
	if (address < pcm->rom_size)
		return pcm->rom[address];
	offset = ram_offset(pcm, address);
	if (offset != (UINT32)-1)
		return pcm->ram[offset];
	return 0xFF;
}

static void mem_write(PCM_ENGINE* pcm, UINT32 address, UINT8 data)
{
	UINT32 offset;

	address &= ADDRESS_MASK;
	if (address < pcm->rom_size)
		return;	// can't write to ROM
	offset = ram_offset(pcm, address);
	if (offset != (UINT32)-1)
		pcm->ram[offset] = data;
	return;
}


//----------------------------------------------------------------------------
// registers
//
//      System-wide registers:
//        00-01 xxxxxxxx LSI Test
//           02 -------x Memory access mode (0=sound gen, 1=read/write)
//              ------x- Memory type (0=ROM, 1=ROM+SRAM)
//              ---xxx-- Wave table header
//              xxx----- Device ID (=1 for YMF278B)
//           03 --xxxxxx Memory address high
//           04 xxxxxxxx Memory address mid
//           05 xxxxxxxx Memory address low
//           06 xxxxxxxx Memory data
//           F8 --xxx--- Mix control (FM_R)
//              -----xxx Mix control (FM_L)
//           F9 --xxx--- Mix control (PCM_R)
//              -----xxx Mix control (PCM_L)
//
//      Channel-specific registers:
//        08-1F xxxxxxxx Wave table number low
//        20-37 -------x Wave table number high
//              xxxxxxx- F-number low
//        38-4F -----xxx F-number high
//              ----x--- Pseudo-reverb
//              xxxx---- Octave
//        50-67 xxxxxxx- Total level
//              -------x Level direct
//        68-7F x------- Key on
//              -x------ Damp
//              --x----- LFO reset
//              ---x---- Output channel
//              ----xxxx Panpot
//        80-97 --xxx--- LFO speed
//              -----xxx Vibrato
//        98-AF xxxx---- Attack rate
//              ----xxxx Decay rate
//        B0-C7 xxxx---- Sustain level
//              ----xxxx Sustain rate
//        C8-DF xxxx---- Rate correction
//              ----xxxx Release rate
//        E0-F7 -----xxx AM depth
//----------------------------------------------------------------------------

#define REG(offset)		(pcm->regdata[offset])

// system-wide registers
#define memory_access_mode()	bitfield(REG(0x02), 0, 1)
#define wave_table_header()		bitfield(REG(0x02), 2, 3)
#define memory_address()		((bitfield(REG(0x03), 0, 6) << 16) | (REG(0x04) << 8) | REG(0x05))

// per-channel registers
#define ch_wave_table_num(choffs)	(REG((choffs) + 0x08) | (bitfield(REG((choffs) + 0x20), 0, 1) << 8))
#define ch_fnumber(choffs)			(bitfield(REG((choffs) + 0x20), 1, 7) | (bitfield(REG((choffs) + 0x38), 0, 3) << 7))
#define ch_pseudo_reverb(choffs)	bitfield(REG((choffs) + 0x38), 3, 1)
#define ch_octave(choffs)			bitfield(REG((choffs) + 0x38), 4, 4)
#define ch_total_level(choffs)		bitfield(REG((choffs) + 0x50), 1, 7)
#define ch_level_direct(choffs)		bitfield(REG((choffs) + 0x50), 0, 1)
#define ch_damp(choffs)				bitfield(REG((choffs) + 0x68), 6, 1)
#define ch_lfo_reset(choffs)		bitfield(REG((choffs) + 0x68), 5, 1)
#define ch_output_channel(choffs)	bitfield(REG((choffs) + 0x68), 4, 1)
#define ch_panpot(choffs)			bitfield(REG((choffs) + 0x68), 0, 4)
#define ch_lfo_speed(choffs)		bitfield(REG((choffs) + 0x80), 3, 3)
#define ch_vibrato(choffs)			bitfield(REG((choffs) + 0x80), 0, 3)
#define ch_attack_rate(choffs)		bitfield(REG((choffs) + 0x98), 4, 4)
#define ch_decay_rate(choffs)		bitfield(REG((choffs) + 0x98), 0, 4)
#define ch_sustain_level(choffs)	bitfield(REG((choffs) + 0xb0), 4, 4)
#define ch_sustain_rate(choffs)		bitfield(REG((choffs) + 0xb0), 0, 4)
#define ch_rate_correction(choffs)	bitfield(REG((choffs) + 0xc8), 4, 4)
#define ch_release_rate(choffs)		bitfield(REG((choffs) + 0xc8), 0, 4)
#define ch_am_depth(choffs)			bitfield(REG((choffs) + 0xe0), 0, 3)

// reset the register state
static void regs_reset(PCM_ENGINE* pcm)
{
	memset(pcm->regdata, 0x00, PCM_REGISTERS);
	REG(0xf8) = 0x1b;
	return;
}

// return the memory address and increment it
static UINT32 regs_memory_address_autoinc(PCM_ENGINE* pcm)
{
	UINT32 result;
	UINT32 newval;

	result = memory_address();
	newval = result + 1;
	REG(0x05) = (UINT8)(newval >> 0);
	REG(0x04) = (UINT8)(newval >> 8);
	REG(0x03) = (UINT8)((newval >> 16) & 0x3f);
	return result;
}

// return the effective rate, clamping and applying corrections as needed
static UINT32 effective_rate(UINT32 raw, UINT32 correction)
{
	// raw rates of 0 and 15 just pin to min/max
	if (raw == 0)
		return 0;
	if (raw == 15)
		return 63;

	// otherwise add the correction and clamp to range
	return (UINT32)clamp((INT32)(raw * 4 + correction), 0, 63);
}

// update the cache with data from the registers
static void regs_cache_channel_data(const PCM_ENGINE* pcm, UINT32 choffs, PCM_CACHE* cache)
{
	INT32 octave;
	INT32 shift;
	INT32 panpot;
	INT32 correction;
	UINT32 fnum;

	// compute step from octave and fnumber; the math here implies
	// a .18 fraction but .16 should be perfectly fine
	octave = sign_extend_4(ch_octave(choffs));
	fnum = ch_fnumber(choffs);
	shift = octave + 7;
	if (shift >= 0)
		cache->step = ((0x400 | fnum) << shift) >> 2;
	else
		cache->step = ((0x400 | fnum) >> -shift) >> 2;

	// total level is computed as a .10 value for interpolation
	cache->total_level = ch_total_level(choffs) << 10;

	// compute panning values in terms of envelope attenuation
	panpot = sign_extend_4(ch_panpot(choffs));
	if (panpot >= 0)
	{
		cache->pan_left = (panpot == 7) ? 0x3ff : 0x20 * panpot;
		cache->pan_right = 0;
	}
	else if (panpot >= -7)
	{
		cache->pan_left = 0;
		cache->pan_right = (panpot == -7) ? 0x3ff : -0x20 * panpot;
	}
	else
	{
		cache->pan_left = cache->pan_right = 0x3ff;
	}

	// determine the LFO stepping value
	cache->lfo_step = s_lfo_steps[ch_lfo_speed(choffs)];

	// AM LFO depth values
	cache->am_depth = s_am_depth[ch_am_depth(choffs)];

	// PM LFO depth values
	cache->pm_depth = s_pm_depth[ch_vibrato(choffs)];

	// 4-bit sustain level, but 15 means 31 so effectively 5 bits
	cache->eg_sustain = ch_sustain_level(choffs);
	cache->eg_sustain |= (cache->eg_sustain + 1) & 0x10;
	cache->eg_sustain <<= 5;

	// compute the key scaling correction factor; 15 means don't do any correction
	correction = (INT32)ch_rate_correction(choffs);
	if (correction == 15)
		correction = 0;
	else
		correction = (octave + correction) * 2 + (INT32)bitfield(fnum, 9, 1);

	// compute the envelope generator rates
	cache->eg_rate[PCM_EG_ATTACK] = (UINT8)effective_rate(ch_attack_rate(choffs), correction);
	cache->eg_rate[PCM_EG_DECAY] = (UINT8)effective_rate(ch_decay_rate(choffs), correction);
	cache->eg_rate[PCM_EG_SUSTAIN] = (UINT8)effective_rate(ch_sustain_rate(choffs), correction);
	cache->eg_rate[PCM_EG_RELEASE] = (UINT8)effective_rate(ch_release_rate(choffs), correction);
	cache->eg_rate[PCM_EG_REVERB] = 5;

	// if damping is on, override some things; essentially decay at a hardcoded
	// rate of 48 until -12db (0x80), then at maximum rate for the rest
	if (ch_damp(choffs) != 0)
	{
		cache->eg_rate[PCM_EG_DECAY] = 48;
		cache->eg_rate[PCM_EG_SUSTAIN] = 63;
		cache->eg_rate[PCM_EG_RELEASE] = 63;
		cache->eg_sustain = 0x80;
	}
	return;
}


//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

static void channel_reset(PCM_CHANNEL* ch)
{
	ch->baseaddr = 0;
	ch->endpos = 0;
	ch->looppos = 0;
	ch->curpos = 0;
	ch->nextpos = 0;
	ch->lfo_counter = 0;
	ch->eg_state = PCM_EG_RELEASE;
	ch->env_attenuation = 0x3ff;
	ch->total_level = 0x7f << 10;
	ch->format = 0;
	ch->key_state = 0;
	ch->keyoff_pending = 0;
	return;
}

// start the attack phase
static void channel_start_attack(PCM_ENGINE* pcm, PCM_CHANNEL* ch, UINT32 choffs)
{
	// don't change anything if already in attack state
	if (ch->eg_state == PCM_EG_ATTACK)
		return;
	ch->eg_state = PCM_EG_ATTACK;

	// reset the LFO if requested
	if (ch_lfo_reset(choffs))
		ch->lfo_counter = 0;

	// if the attack rate == 63 then immediately go to max attenuation
	if (ch->cache.eg_rate[PCM_EG_ATTACK] == 63)
		ch->env_attenuation = 0;

	// reset the positions
	ch->curpos = ch->nextpos = 0;
	return;
}

// start the release phase
static void channel_start_release(PCM_CHANNEL* ch)
{
	// don't change anything if already in release or reverb state
	if (ch->eg_state >= PCM_EG_RELEASE)
		return;
	ch->eg_state = PCM_EG_RELEASE;
	return;
}

// prepare for clocking; returns non-zero if the channel is active
static UINT8 channel_prepare(PCM_ENGINE* pcm, PCM_CHANNEL* ch, UINT32 choffs)
{
	UINT8 oldon;
	UINT8 newon;

	// cache the data
	regs_cache_channel_data(pcm, choffs, &ch->cache);

	// clock the key state; the live key is the pending one, and a key off
	// since the last prepare releases the channel before the key on
	if ((ch->key_state & KEY_PENDING) != 0)
	{
		oldon = ch->key_state & KEY_ON;
		newon = (ch->key_state >> 1) & KEY_ON;
		ch->key_state = newon;
		if (ch->keyoff_pending && oldon)
		{
			channel_start_release(ch);
			oldon = 0;
		}
		ch->keyoff_pending = 0;
		if (oldon != newon)
		{
			if (newon)
				channel_start_attack(pcm, ch, choffs);
			else
				channel_start_release(ch);
		}
	}

	// set the total level directly if not interpolating
	if (ch_level_direct(choffs))
		ch->total_level = ch->cache.total_level;

	// we're active until we're quiet after the release
	return (ch->eg_state < PCM_EG_RELEASE || ch->env_attenuation < PCM_EG_QUIET);
}

// clock the envelope generator
static void channel_clock_envelope(const PCM_ENGINE* pcm, PCM_CHANNEL* ch, UINT32 choffs, UINT32 env_counter)
{
	UINT32 rate;
	UINT32 rate_shift;
	UINT32 relevant_bits;
	UINT32 increment;

	// handle attack->decay transitions
	if (ch->eg_state == PCM_EG_ATTACK && ch->env_attenuation == 0)
		ch->eg_state = PCM_EG_DECAY;

	// handle decay->sustain transitions
	if (ch->eg_state == PCM_EG_DECAY && ch->env_attenuation >= ch->cache.eg_sustain)
		ch->eg_state = PCM_EG_SUSTAIN;

	// fetch the appropriate 6-bit rate value from the cache
	rate = ch->cache.eg_rate[ch->eg_state];

	// compute the rate shift value; this is the shift needed to
	// apply to the env_counter such that it becomes a 5.11 fixed
	// point number
	rate_shift = rate >> 2;
	env_counter <<= rate_shift;

	// see if the fractional part is 0; if not, it's not time to clock
	if (bitfield(env_counter, 0, 11) != 0)
		return;

	// determine the increment based on the non-fractional part of env_counter
	relevant_bits = bitfield(env_counter, (rate_shift <= 11) ? 11 : rate_shift, 3);
	increment = attenuation_increment(rate, relevant_bits);

	// attack is the only one that increases
	if (ch->eg_state == PCM_EG_ATTACK)
	{
		ch->env_attenuation = (UINT16)(ch->env_attenuation + ((~(UINT32)ch->env_attenuation * increment) >> 4));
	}
	// all other cases are similar
	else
	{
		// apply the increment
		ch->env_attenuation = (UINT16)(ch->env_attenuation + increment);

		// clamp the final attenuation
		if (ch->env_attenuation >= 0x400)
			ch->env_attenuation = 0x3ff;

		// transition to reverb at -18dB if enabled
		if (ch->env_attenuation >= 0xc0 && ch->eg_state < PCM_EG_REVERB && ch_pseudo_reverb(choffs))
			ch->eg_state = PCM_EG_REVERB;
	}
	return;
}

// master clocking function
static void channel_clock(const PCM_ENGINE* pcm, PCM_CHANNEL* ch, UINT32 choffs, UINT32 env_counter)
{
	UINT32 step;
	UINT32 lfo_shifted;
	INT32 lfo_value;

	// clock the LFO, which is an x.18 value incremented based on the
	// LFO speed value
	ch->lfo_counter += ch->cache.lfo_step;

	// clock the envelope
	channel_clock_envelope(pcm, ch, choffs, env_counter);

	// determine the step after applying vibrato
	step = ch->cache.step;
	if (ch->cache.pm_depth != 0)
	{
		// shift the LFO by 1/4 cycle for PM so that it starts at 0
		lfo_shifted = ch->lfo_counter + (1 << 16);
		lfo_value = (INT32)bitfield(lfo_shifted, 10, 7);
		if (bitfield(lfo_shifted, 17, 1) != 0)
			lfo_value ^= 0x7f;
		lfo_value -= 0x40;
		step += (UINT32)((lfo_value * (INT32)ch->cache.pm_depth) >> 7);
	}

	// advance the sample step and loop as needed
	ch->curpos = ch->nextpos;
	ch->nextpos = ch->curpos + step;
	if (ch->nextpos >= ch->endpos)
		ch->nextpos += ch->looppos - ch->endpos;

	// interpolate total level if needed
	if (ch->total_level != ch->cache.total_level)
	{
		// max->min volume takes 156.4ms, or pretty close to 19/1024 per 44.1kHz sample
		// min->max volume is half that, so advance by 38/1024 per sample
		if (ch->total_level < ch->cache.total_level)
		{
			ch->total_level += 19;
			if ((INT32)ch->total_level > (INT32)ch->cache.total_level)
				ch->total_level = ch->cache.total_level;
		}
		else
		{
			ch->total_level -= 38;
			if ((INT32)ch->total_level < (INT32)ch->cache.total_level)
				ch->total_level = ch->cache.total_level;
		}
	}
	return;
}

// fetch a sample at the current position
static INT16 channel_fetch_sample(const PCM_ENGINE* pcm, const PCM_CHANNEL* ch)
{
	UINT32 addr;
	UINT32 pos;
	INT32 odd;
	INT32 byte0;
	INT32 byte1;
	INT32 byte2;

	addr = ch->baseaddr;
	pos = ch->curpos >> 16;

	// 8-bit PCM: shift up by 8
	if (ch->format == 0)
		return (INT16)(mem_read(pcm, addr + pos) << 8);

	// 16-bit PCM: assemble from 2 halves
	if (ch->format == 2)
	{
		addr += pos * 2;
		return (INT16)((mem_read(pcm, addr) << 8) | mem_read(pcm, addr + 1));
	}

	// 12-bit PCM: assemble out of half of 3 bytes; the even sample takes the
	// first byte and the low nibble of the second, the odd sample the third
	// byte and the high nibble of the second. The parity alternates, so it
	// is selected without a branch (reading the unused byte has no effect)
	addr += (pos / 2) * 3;
	odd = pos & 1;
	byte0 = (INT32)mem_read(pcm, addr + 0);
	byte1 = (INT32)mem_read(pcm, addr + 1);
	byte2 = (INT32)mem_read(pcm, addr + 2);
	return (INT16)(((byte0 ^ ((byte0 ^ byte2) & -odd)) << 8) | (((byte1 << 4) >> (4 * odd)) & 0xf0));
}

// add the computed output value, with panning applied, to the 4 outputs
static void channel_output(const PCM_ENGINE* pcm, const PCM_CHANNEL* ch, UINT32 choffs, INT32* output)
{
	UINT32 envelope;
	UINT32 lfo_value;
	UINT32 lenv;
	UINT32 renv;
	INT32 lvol;
	INT32 rvol;
	INT32 sample;
	UINT32 outnum;

	// early out if the envelope is effectively off
	envelope = ch->env_attenuation;
	if (envelope > PCM_EG_QUIET)
		return;

	// add in LFO AM modulation
	if (ch->cache.am_depth != 0)
	{
		lfo_value = bitfield(ch->lfo_counter, 10, 7);
		if (bitfield(ch->lfo_counter, 17, 1) != 0)
			lfo_value ^= 0x7f;
		envelope += (lfo_value * ch->cache.am_depth) >> 7;
	}

	// add in the current interpolated total level value, which is a .10
	// value shifted left by 2
	envelope += ch->total_level >> 8;

	// add in panning effect and clamp
	lenv = envelope + ch->cache.pan_left;
	if (lenv > 0x3ff)
		lenv = 0x3ff;
	renv = envelope + ch->cache.pan_right;
	if (renv > 0x3ff)
		renv = 0x3ff;

	// convert to volume as a .11 fraction
	lvol = (INT32)attenuation_to_volume(lenv << 2);
	rvol = (INT32)attenuation_to_volume(renv << 2);

	// fetch current sample and add
	sample = channel_fetch_sample(pcm, ch);
	outnum = ch_output_channel(choffs) * 2;
	output[outnum + 0] += (lvol * sample) >> 15;
	output[outnum + 1] += (rvol * sample) >> 15;
	return;
}

// signal key on/off; the key state is only consumed by the next prepare
static void channel_keyonoff(PCM_CHANNEL* ch, UINT32 on)
{
	UINT8 live;

	// the key is live as pending after a write, as the key state otherwise
	live = (ch->key_state & KEY_PENDING) ? ((ch->key_state & KEY_PENDING_ON) != 0) : (ch->key_state & KEY_ON);
	if (! on && live)
		ch->keyoff_pending = 1;

	// mark the key state as pending
	ch->key_state &= ~KEY_PENDING_ON;
	ch->key_state |= KEY_PENDING | (on ? KEY_PENDING_ON : 0);
	return;
}


//----------------------------------------------------------------------------
// engine
//----------------------------------------------------------------------------

void pcm_engine_init(PCM_ENGINE* pcm)
{
	UINT8 curChn;

	pcm->env_counter = 0;
	pcm->modified_channels = PCM_ALL_CHANNELS;
	pcm->active_channels = PCM_ALL_CHANNELS;
	pcm->prepare_count = 0;
	for (curChn = 0; curChn < PCM_CHANNELS; curChn ++)
		channel_reset(&pcm->channel[curChn]);
	return;
}

// reset the engine state
void pcm_engine_reset(PCM_ENGINE* pcm)
{
	UINT8 curChn;

	// reset register state
	regs_reset(pcm);

	// reset each channel
	for (curChn = 0; curChn < PCM_CHANNELS; curChn ++)
		channel_reset(&pcm->channel[curChn]);
	return;
}

// load a wavetable by fetching its data from external memory
static void channel_load_wavetable(PCM_ENGINE* pcm, PCM_CHANNEL* ch, UINT32 choffs)
{
	UINT32 wavnum;
	UINT32 wavheader;
	UINT32 bank;
	UINT8 byte;

	// determine the address of the wave table header
	wavnum = ch_wave_table_num(choffs);
	wavheader = 12 * wavnum;

	// above 384 it may be in a different bank
	if (wavnum >= 384)
	{
		bank = wave_table_header();
		if (bank != 0)
			wavheader = 512 * 1024 * bank + (wavnum - 384) * 12;
	}

	// fetch the 22-bit base address and 2-bit format
	byte = mem_read(pcm, wavheader + 0);
	ch->format = (UINT8)bitfield(byte, 6, 2);
	ch->baseaddr = bitfield(byte, 0, 6) << 16;
	ch->baseaddr |= mem_read(pcm, wavheader + 1) << 8;
	ch->baseaddr |= mem_read(pcm, wavheader + 2) << 0;

	// fetch the 16-bit loop position
	ch->looppos = mem_read(pcm, wavheader + 3) << 8;
	ch->looppos |= mem_read(pcm, wavheader + 4);
	ch->looppos <<= 16;

	// fetch the 16-bit end position, which is stored as a negative value
	// for some reason that is unclear
	ch->endpos = mem_read(pcm, wavheader + 5) << 8;
	ch->endpos |= mem_read(pcm, wavheader + 6);
	ch->endpos = (0 - ch->endpos) << 16;

	// remaining data values set registers
	pcm_engine_write(pcm, 0x80 + choffs, mem_read(pcm, wavheader + 7));
	pcm_engine_write(pcm, 0x98 + choffs, mem_read(pcm, wavheader + 8));
	pcm_engine_write(pcm, 0xb0 + choffs, mem_read(pcm, wavheader + 9));
	pcm_engine_write(pcm, 0xc8 + choffs, mem_read(pcm, wavheader + 10));
	pcm_engine_write(pcm, 0xe0 + choffs, mem_read(pcm, wavheader + 11));

	// reset the envelope so we don't continue playing mid-sample from previous key ons
	ch->env_attenuation = 0x3ff;
	return;
}

// handle writes to the PCM registers
void pcm_engine_write(PCM_ENGINE* pcm, UINT32 regnum, UINT8 data)
{
	// handle writes to the data register
	if (regnum == 0x06 && memory_access_mode() != 0)
	{
		mem_write(pcm, regs_memory_address_autoinc(pcm), data);
		return;
	}

	// for now just mark all channels as modified
	pcm->modified_channels = PCM_ALL_CHANNELS;

	// most writes are passive, consumed only when needed
	REG(regnum) = data;

	// however, process keyons immediately
	if (regnum >= 0x68 && regnum <= 0x7f)
		channel_keyonoff(&pcm->channel[regnum - 0x68], bitfield(data, 7, 1));

	// and also wavetable writes
	else if (regnum >= 0x08 && regnum <= 0x1f)
		channel_load_wavetable(pcm, &pcm->channel[regnum - 0x08], regnum - 0x08);
	return;
}

// handle reads from the PCM registers
UINT8 pcm_engine_read(PCM_ENGINE* pcm, UINT32 regnum)
{
	// handle reads from the data register
	if (regnum == 0x06 && memory_access_mode() != 0)
		return mem_read(pcm, regs_memory_address_autoinc(pcm));

	return REG(regnum);
}

// master clocking function
void pcm_engine_clock(PCM_ENGINE* pcm)
{
	UINT8 chnum;

	// if something was modified, prepare
	// also prepare every 4k samples to catch ending notes
	if (pcm->modified_channels != 0 || pcm->prepare_count++ >= 4096)
	{
		// call each channel to prepare
		pcm->active_channels = 0;
		for (chnum = 0; chnum < PCM_CHANNELS; chnum ++)
		{
			if (channel_prepare(pcm, &pcm->channel[chnum], chnum))
				pcm->active_channels |= 1 << chnum;
		}

		// reset the modified channels and prepare count
		pcm->modified_channels = pcm->prepare_count = 0;
	}

	// increment the envelope counter; the envelope generator
	// only clocks every other sample in order to make the PCM
	// envelopes line up with the FM envelopes (after taking into
	// account the different FM sampling rate)
	pcm->env_counter ++;

	// now update the state of all the channels
	for (chnum = 0; chnum < PCM_CHANNELS; chnum ++)
		channel_clock(pcm, &pcm->channel[chnum], chnum, pcm->env_counter >> 1);
	return;
}

// compute the sum of the channel outputs into the 4 outputs; muted channels
// are skipped here, as their state is kept by the clocking
void pcm_engine_output(const PCM_ENGINE* pcm, INT32* output)
{
	UINT8 chnum;

	for (chnum = 0; chnum < PCM_CHANNELS; chnum ++)
	{
		if ((pcm->mute_mask >> chnum) & 0x01)
			continue;
		channel_output(pcm, &pcm->channel[chnum], chnum, output);
	}
	return;
}
