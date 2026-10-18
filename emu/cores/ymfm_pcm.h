/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles
***************************************************************************/

#ifndef __YMFM_PCM_H__
#define __YMFM_PCM_H__

#include "../../stdtype.h"

#define PCM_CHANNELS	24
#define PCM_REGISTERS	0x100

// envelope states; EG_REVERB is the OPL4 pseudo-reverb state
#define PCM_EG_ATTACK	1
#define PCM_EG_DECAY	2
#define PCM_EG_SUSTAIN	3
#define PCM_EG_RELEASE	4
#define PCM_EG_REVERB	5
#define PCM_EG_STATES	6

// data computed once at the start of clocking that remains static during
// subsequent sound generation
typedef struct _pcm_cache
{
	UINT32 step;			// sample position step, as a .16 value
	UINT32 total_level;		// target total level, as a .10 value
	UINT32 pan_left;		// left panning attenuation
	UINT32 pan_right;		// right panning attenuation
	UINT32 eg_sustain;		// sustain level, shifted up to envelope values
	UINT8 eg_rate[PCM_EG_STATES];	// envelope rate, including KSR
	UINT8 lfo_step;			// stepping value for LFO
	UINT8 am_depth;			// scale value for AM LFO
	UINT8 pm_depth;			// scale value for PM LFO
} PCM_CACHE;

typedef struct _pcm_channel
{
	UINT32 baseaddr;		// base address
	UINT32 endpos;			// ending position
	UINT32 looppos;			// loop position
	UINT32 curpos;			// current position
	UINT32 nextpos;			// next position
	UINT32 lfo_counter;		// LFO counter
	UINT8 eg_state;			// envelope state
	UINT16 env_attenuation;	// computed envelope attenuation
	UINT32 total_level;		// total level with as 7.10 for interp
	UINT8 format;			// sample format
	UINT8 key_state;		// current key state
	UINT8 keyoff_pending;	// the live key went off since the last prepare
	PCM_CACHE cache;		// cached data
} PCM_CHANNEL;

// the OPL4 wavetable engine (ymfm's pcm_engine and pcm_registers)
typedef struct _pcm_engine
{
	UINT8 regdata[PCM_REGISTERS];	// register data
	UINT32 env_counter;		// envelope counter
	UINT32 modified_channels;	// bitmask of modified channels
	UINT32 active_channels;	// bitmask of active channels
	UINT32 prepare_count;	// counter to do periodic prepare sweeps
	UINT32 mute_mask;		// muted channels
	PCM_CHANNEL channel[PCM_CHANNELS];

	// memory, owned by the chip: the ROM at address 0, the RAM after it
	const UINT8* rom;
	UINT32 rom_size;
	UINT8* ram;
	UINT32 ram_size;
} PCM_ENGINE;

// set up the engine; call once, before pcm_engine_reset(); the chip sets the memory
void pcm_engine_init(PCM_ENGINE* pcm);
// reset the registers and the channels
void pcm_engine_reset(PCM_ENGINE* pcm);
// clock all channels forward one sample
void pcm_engine_clock(PCM_ENGINE* pcm);
// add the outputs of the unmuted channels to output[0-3] (2 stereo pairs, by output channel)
void pcm_engine_output(const PCM_ENGINE* pcm, INT32* output);
// read/write the PCM registers
void pcm_engine_write(PCM_ENGINE* pcm, UINT32 regnum, UINT8 data);
UINT8 pcm_engine_read(PCM_ENGINE* pcm, UINT32 regnum);

#endif	// __YMFM_PCM_H__
