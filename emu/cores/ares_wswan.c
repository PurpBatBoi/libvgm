/***************************************************************************
    SPDX-License-Identifier: ISC
    Copyright (c) 2004-2025 ares team, Near et al

    Permission to use, copy, modify, and/or distribute this software for any
    purpose with or without fee is hereby granted, provided that the above
    copyright notice and this permission notice appear in all copies.

    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

    Bandai WonderSwan sound

    C port of the sound unit of ares' WonderSwan (ares/ws/apu/apu.cpp,
    apu.hpp, io.cpp, channel1.cpp ... channel4.cpp, memory/memory.hpp for the
    wave RAM).
    https://github.com/ares-emulator/ares

    Port notes:
    - One update sample is one call of ares' APU::main() with the "Pixel
      Accuracy" option off (ares' default): 128 APU clocks, then the channel
      outputs are added up. The sample rate is clock / 128 (24000 Hz for the
      VGM clock 3072000).
    - The 128 ticks are not run one by one. A channel keeps the ticks to the
      next match of its period with its pitch instead of the period, and steps
      from one match to the next (ws_chn_set_pitch explains why that gives the
      same period and sample offset), channel 3 in two runs when its sweep
      steps in between, channel 4 stepping its noise register once for each
      match.
    - The mix is not added up again for every sample: a channel's level
      changes only when its sample offset moves on, or with a write, a mute
      change or a reset, which mark what they change as stale (mix_stale).
    - With both, a sample takes about 9 ns instead of ares' 360 ns (four
      channels playing), and bit for bit the same.
    - The registers are the ports 0x80 ... 0x9F of the console. The VGM
      command 0xBC and the DAC streams add 0x80 to the register number, so
      this device gets the port number (A8D8). The wave RAM is written
      through the memory interface (A16D8, VGM command 0xC6). The wave RAM
      holds the first 16 KB of the console's internal RAM, which is all the
      sound unit can address: (wave base << 6) + 48 + 15 is 0x3FFF at most.
      Writes above that are ignored, as in ares on the mono console.
    - The channels add up to a 10 bit level per side (at most 930). The
      output is stereo with that level << 5 when the headphone output is
      enabled (port 0x91 bit 3), and mono ((left + right) >> speaker shift,
      8 bit, << 7) when only the speaker output is enabled (bits 0 ... 2),
      as in ares' APU::output(); both are 0 ... 32640 and unsigned.
    - That level has a DC offset of about half its range, and the 24 kHz to
      44.1 kHz resampling of the player overshoots on the edges of the
      waves, so it clips. The output goes through a DC removal filter (see
      the deliberate changes).
    - Muting computes every channel and only leaves the muted ones out of the
      sum.
    - Not ported (no VGM command reaches them, or no supported caller uses
      them):
      - Hyper Voice (channel 5, ports 0x64 ... 0x6B) and the sound DMA
        (ports 0x4A ... 0x52): they are Color-only and below port 0x80,
        which the VGM commands cannot address. Without them the sound unit
        needs no bus.
      - All register reads (the read only ports 0x92 ... 0x9B and the read
        back of the others), the "Pixel Accuracy" per-clock mode, the
        stream filter (see below), the unknown bit of the debug port 0x95
        (it is only read back), the serializer, the scheduler thread and the
        debugger.
    - Deliberate changes from ares:
      - Port 0x9E (master volume, Color only) is ignored and the volume is
        always 100%. VGM rips start by writing 0 to every sound register,
        including 0x9E, which would mute a Color console in ares (129 of
        the 151 test songs). On the console this is the volume button, not
        a game setting, so a rip has no right value for it.
      - ares knows whether headphones are plugged in (a setting, default
        yes) and plays only the output the game enabled for that. A VGM rip
        has no such setting: 33 of the 151 test songs enable only the
        speaker (port 0x91 = 0x07) and would be silent with the headphones
        "plugged in". Here the headphones count as plugged in when the
        song enables them (bit 3), and the speaker is used otherwise.
      - ares puts a one-pole "high-pass filter at 15.915 Hz" on its audio
        stream. nall's one-pole high-pass has a gain of 0.7071 at DC (it does
        not block it: the gain rises from 0.7071 to 1.0 at the Nyquist
        frequency), so ares plays the output at 70.7% with a slight treble
        tilt and with the DC offset. That is replaced by nall's DCRemoval
        (x = 0.999 x + in - y, the filter ares uses for the 32X PWM) on each
        side. Like in the 32X PWM port, its history starts at the first
        non-zero input after a reset, so the first note does not begin with
        the whole DC offset as a step that decays over ~1000 samples.
      - The registers and the wave RAM start as all zero after a reset, as in
        ares' power(), except that the headphone output is enabled (ares
        starts with port 0x91 = 0, no output). On the console the BIOS sets
        port 0x91 before the game runs, and a rip that starts in the middle
        of a game can leave out the write: 3 of the 151 test songs (PCM
        voice tracks of Beatmania) never write it and would be silent. Any
        write to port 0x91 replaces the default.
      - The device always runs at clock / 128; the player resamples.

***************************************************************************/

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "ares_wswan.h"


static void ws_reset(void* info);
static void ws_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_wswan(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_wswan(void* info);
static void ws_write_reg(void* info, UINT8 port, UINT8 data);
static void ws_write_ram(void* info, UINT16 offset, UINT8 data);
static void ws_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ws_write_reg},
	{RWF_MEMORY | RWF_WRITE, DEVRW_A16D8, 0, ws_write_ram},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ws_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"WonderSwan", "ares", FCC_ARES,

	device_start_wswan,
	device_stop_wswan,
	ws_reset,
	ws_update,

	NULL,	// SetOptionBits
	ws_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "WonderSwan";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 4;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_WSwan =
{
	DEVID_WSWAN,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};


//----------------------------------------------------------------------------
// chip state
//----------------------------------------------------------------------------

#define WS_DEFAULT_CLOCK	3072000		// the sound unit runs at 3.072 MHz
#define WS_CLOCKS_PER_SAMPLE	128		// APU clocks per output sample
#define WS_RAM_SIZE		0x4000		// wave RAM: the first 16 KB of the internal RAM
#define WS_PITCH_MASK		0x07FF		// 11 bit pitch and period
#define WS_OFFSET_MASK		0x1F		// 32 samples (4 bit) per wave table
#define WS_LFSR_MASK		0x7FFF		// 15 bit noise register
#define WS_MIX_BASE		0x10		// mix_stale: the base level (bits 0 ... 3: the channel levels)
#define WS_MIX_ALL		0x1F		// mix_stale: everything

// the four wave channels (ares' Channel1 ... Channel4 io.pitch / state.period / state.sampleOffset)
typedef struct _ws_channel
{
	UINT16 pitch;		// 11 bit
	UINT16 counter;		// 1 ... 2048: ticks to the next match of the period with the pitch (ws_chn_advance)
	UINT8 volume_left;	// 4 bit
	UINT8 volume_right;	// 4 bit
	UINT8 sample_offset;	// 5 bit, position in the wave table
	UINT8 enable;
} WS_CHANNEL;

// nall::DSP::IIR::DCRemoval, with the history started at the first non-zero input
typedef struct _ws_dc_filter
{
	double x;
	double y;
	UINT8 primed;
} WS_DC_FILTER;

typedef struct _wsa_state
{
	DEV_DATA _devData;

	UINT32 clock;
	UINT32 rate;
	UINT8 mute;		// bit 0 ... 3 = channel 1 ... 4

	// everything from here on is cleared by a reset
	WS_CHANNEL chn[4];

	// channel 2 voice mode (the volume register is a PCM sample)
	UINT8 voice;
	UINT8 voice_left_half;
	UINT8 voice_left_full;
	UINT8 voice_right_half;
	UINT8 voice_right_full;

	// channel 3 sweep
	INT8 sweep_value;
	UINT8 sweep_time;	// 5 bit
	UINT8 sweep;
	INT32 sweep_counter;
	UINT16 sweep_clock;	// 13 bit, the sweep steps when it wraps

	// channel 4 noise
	UINT8 noise_mode;	// 3 bit
	UINT8 noise_update;
	UINT8 noise;
	UINT8 noise_output;
	UINT16 noise_lfsr;	// 15 bit

	UINT8 wave_base;
	UINT8 speaker_enable;
	UINT8 speaker_shift;	// 2 bit
	UINT8 headphones_enable;

	// DC removal of the left and right output
	WS_DC_FILTER dc_left;
	WS_DC_FILTER dc_right;

	// sequencer debug port 0x95
	UINT8 seq_dbg_hold;
	UINT8 seq_dbg_sweep_clock;
	UINT8 seq_dbg_noise;	// 2 bit
	UINT8 seq_dbg_output_force55;
	UINT8 seq_dbg_ch_force2;
	UINT8 seq_dbg_ch_force4;

	// The mix (the sum of the channel outputs), kept by ws_run(): a channel's level changes only when its sample offset
	// moves on, or with a write, a mute change or a reset, which mark the parts they change as stale.
	UINT8 mix_stale;	// bit 0 ... 3 = channel 1 ... 4 (volume and level), WS_MIX_BASE = base
	UINT8 mix_volume_left[4];	// volume of a heard channel, 0 for one that is off, muted, held or in voice mode
	UINT8 mix_volume_right[4];
	UINT16 mix_level_left[4];	// sample * volume
	UINT16 mix_level_right[4];
	UINT16 mix_base_left;	// APU::sequencerClear() level plus the channel 2 voice
	UINT16 mix_base_right;

	UINT8 ram[WS_RAM_SIZE];
} wsa_state;

static const UINT8 noise_taps[8] = {14, 10, 13, 4, 8, 6, 9, 11};


// ares counts the period (an 11 bit down counter) down by one per tick and, when it equals the pitch, restarts it at 0 and
// moves the sample offset on. So the first match comes ((period - pitch - 1) mod 2048) + 1 ticks from now, and the next ones
// 2048 - pitch ticks after each other (the period restarts at 0). The channel keeps that number of ticks (the counter)
// instead of the period and steps from match to match instead of tick to tick, which gives the same matches as the ticks
// (the registers do not change in between). The period is (counter + pitch) mod 2048.
INLINE void ws_chn_set_pitch(WS_CHANNEL* chn, UINT16 pitch)
{
	UINT16 period = (UINT16)((chn->counter + chn->pitch) & WS_PITCH_MASK);

	chn->pitch = pitch;
	chn->counter = (UINT16)(((period - pitch - 1) & WS_PITCH_MASK) + 1);
}

// ws_chn_advance() when the "ticks" reach the next match: the first match, then one every 2048 - pitch ticks
static UINT32 ws_chn_match(WS_CHANNEL* chn, UINT32 ticks)
{
	UINT32 step = (WS_PITCH_MASK + 1) - chn->pitch;
	UINT32 matches = 1;

	ticks -= chn->counter;	// the ticks after the first match
	if (ticks >= step)
	{
		matches += ticks / step;
		ticks %= step;
	}
	chn->counter = (UINT16)(step - ticks);
	chn->sample_offset = (UINT8)((chn->sample_offset + matches) & WS_OFFSET_MASK);
	return matches;
}

// ChannelN::tick() done "ticks" times. Returns how often the sample offset moved on.
INLINE UINT32 ws_chn_advance(WS_CHANNEL* chn, UINT32 ticks)
{
	if (! chn->enable)
		return 0;
	if (ticks < chn->counter)
	{
		chn->counter = (UINT16)(chn->counter - ticks);
		return 0;
	}
	return ws_chn_match(chn, ticks);
}

// Channel3::sweep()
INLINE void ws_sweep(wsa_state* chip)
{
	WS_CHANNEL* chn = &chip->chn[2];

	if (chip->sweep && --chip->sweep_counter < 0)
	{
		chip->sweep_counter = chip->sweep_time;
		ws_chn_set_pitch(chn, (UINT16)((chn->pitch + chip->sweep_value) & WS_PITCH_MASK));
	}
}

// Channel3::tick() and the sweep clock of APU::main() for "ticks" ticks. The sweep steps after the tick where the 13 bit
// sweep clock wraps to 0 (every tick when the sequencer debug bit says so), so the channel runs up to that tick first.
// With the sweep off, a sweep step does nothing, so without a step (or with the sweep off) the sweep clock only counts.
// Returns the matches.
INLINE UINT32 ws_sweep_advance(wsa_state* chip, UINT32 ticks)
{
	UINT32 matches = 0;
	UINT32 step;

	if (! chip->sweep || (! chip->seq_dbg_sweep_clock && chip->sweep_clock + ticks < 0x2000))
	{
		chip->sweep_clock = (UINT16)((chip->sweep_clock + ticks) & 0x1FFF);
		return ws_chn_advance(&chip->chn[2], ticks);
	}
	while (ticks > 0)
	{
		step = chip->seq_dbg_sweep_clock ? 1 : 0x2000 - chip->sweep_clock;
		if (step > ticks)
			step = ticks;
		matches += ws_chn_advance(&chip->chn[2], step);
		chip->sweep_clock = (UINT16)((chip->sweep_clock + step) & 0x1FFF);
		if (chip->sweep_clock == 0 || chip->seq_dbg_sweep_clock)
			ws_sweep(chip);
		ticks -= step;
	}
	return matches;
}

// Channel4::tick() "ticks" times: every move of the sample offset steps the noise register (the noise registers do not
// change in between, so the steps can follow the whole run). Returns the matches.
INLINE UINT32 ws_noise_advance(wsa_state* chip, UINT32 ticks)
{
	UINT32 matches;
	UINT32 steps;
	UINT8 tap;

	matches = ws_chn_advance(&chip->chn[3], ticks);
	if (! matches || ! chip->noise_update || chip->seq_dbg_noise)
		return matches;
	tap = noise_taps[chip->noise_mode];
	for (steps = matches; steps > 0; steps --)
	{
		chip->noise_output = (UINT8)((1 ^ (chip->noise_lfsr >> 7) ^ (chip->noise_lfsr >> tap)) & 1);
		chip->noise_lfsr = (UINT16)(((chip->noise_lfsr << 1) | chip->noise_output) & WS_LFSR_MASK);
	}
	return matches;
}

// ares' APU::sample() for channel "chn" (0 ... 3) at its sample offset, and the noise output of channel 4
INLINE UINT8 ws_sample(const wsa_state* chip, UINT8 chn)
{
	UINT8 index = chip->chn[chn].sample_offset;
	UINT8 data;

	if (chn == 3 && chip->noise)
		return chip->noise_output ? 0x0F : 0x00;
	if (chip->seq_dbg_ch_force4)
		return 4;
	if (chip->seq_dbg_ch_force2)
		return 2;

	data = chip->ram[(chip->wave_base << 6) + (chn << 4) + (index >> 1)];
	return (index & 1) ? (data >> 4) : (data & 0x0F);
}

// ChannelN::output(): the level of channel "chn" in the mix, after its sample changed
INLINE void ws_mix_level(wsa_state* chip, UINT8 chn)
{
	UINT8 sample = ws_sample(chip, chn);

	chip->mix_level_left[chn] = (UINT16)(sample * chip->mix_volume_left[chn]);
	chip->mix_level_right[chn] = (UINT16)(sample * chip->mix_volume_right[chn]);
}

// the stale parts of the mix: the volume (0 for a channel that is off, muted, held or in voice mode) and level of the
// stale channels, and APU::sequencerClear() plus the channel 2 voice mode (the volume register is the sample, heard even
// when the channel is off) in the base
static void ws_mix_prepare(wsa_state* chip)
{
	UINT8 held;
	UINT8 heard;
	UINT8 volume;
	UINT8 chn;

	held = chip->seq_dbg_hold || chip->seq_dbg_output_force55;
	for (chn = 0; chn < 4; chn ++)
	{
		if (! (chip->mix_stale & (1 << chn)))
			continue;
		heard = ! held && chip->chn[chn].enable && ! (chip->mute & (1 << chn)) && ! (chn == 1 && chip->voice);
		chip->mix_volume_left[chn] = heard ? chip->chn[chn].volume_left : 0;
		chip->mix_volume_right[chn] = heard ? chip->chn[chn].volume_right : 0;
		ws_mix_level(chip, chn);
	}
	if (chip->mix_stale & WS_MIX_BASE)
	{
		chip->mix_base_left = chip->mix_base_right = chip->seq_dbg_output_force55 ? 0x55 : 0;
		if (chip->voice && ! held && ! (chip->mute & 0x02))
		{
			volume = (UINT8)((chip->chn[1].volume_left << 4) | chip->chn[1].volume_right);
			chip->mix_base_left += chip->voice_left_full ? volume : (chip->voice_left_half ? (volume >> 1) : 0);
			chip->mix_base_right += chip->voice_right_full ? volume : (chip->voice_right_half ? (volume >> 1) : 0);
		}
	}
	chip->mix_stale = 0;
}

// APU::main() without "Pixel Accuracy": 128 clocks, then the outputs of the channels, then APU::output()
// (the output level of both sides in outL / outR, before the DC removal)
static void ws_run(wsa_state* chip, INT32* outL, INT32* outR)
{
	UINT32 matches[4];
	UINT16 left;
	UINT16 right;

	// the 128 ticks of the channels (the channels do not depend on each other)
	matches[0] = ws_chn_advance(&chip->chn[0], WS_CLOCKS_PER_SAMPLE);
	matches[1] = ws_chn_advance(&chip->chn[1], WS_CLOCKS_PER_SAMPLE);
	matches[2] = ws_sweep_advance(chip, WS_CLOCKS_PER_SAMPLE);
	matches[3] = ws_noise_advance(chip, WS_CLOCKS_PER_SAMPLE);

	// the outputs of the channels: a channel whose sample moved on has a new level
	if (chip->mix_stale)
		ws_mix_prepare(chip);
	if (matches[0])
		ws_mix_level(chip, 0);
	if (matches[1])
		ws_mix_level(chip, 1);
	if (matches[2])
		ws_mix_level(chip, 2);
	if (matches[3])
		ws_mix_level(chip, 3);
	left = (UINT16)(chip->mix_base_left + chip->mix_level_left[0] + chip->mix_level_left[1] +
		chip->mix_level_left[2] + chip->mix_level_left[3]);
	right = (UINT16)(chip->mix_base_right + chip->mix_level_right[0] + chip->mix_level_right[1] +
		chip->mix_level_right[2] + chip->mix_level_right[3]);

	// APU::output(): the headphones count as connected when the song enables them
	if (chip->headphones_enable)
	{
		*outL = left << 5;
		*outR = right << 5;
	}
	else if (chip->speaker_enable)
	{
		*outL = *outR = (((left + right) >> chip->speaker_shift) & 0xFF) << 7;
	}
	else
	{
		*outL = 0;
		*outR = 0;
	}
}

INLINE DEV_SMPL ws_dc_process(WS_DC_FILTER* filter, INT32 level)
{
	double in = (double)level;

	if (! filter->primed && in != 0.0)
	{
		filter->primed = 1;
		filter->y = in;
	}
	filter->x = 0.999 * filter->x + in - filter->y;
	filter->y = in;
	// rounded half away from 0: x - 0.5 written as x + -0.5 (the same result) lets the compiler select the constant
	// instead of branching on the sign, which a song's output cannot predict
	return (DEV_SMPL)(filter->x + (filter->x < 0.0 ? -0.5 : 0.5));
}

static void ws_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	wsa_state* chip = (wsa_state*)info;
	DEV_SMPL* outL = outputs[0];
	DEV_SMPL* outR = outputs[1];
	UINT32 curSmpl;
	INT32 left;
	INT32 right;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
	{
		ws_run(chip, &left, &right);
		outL[curSmpl] = ws_dc_process(&chip->dc_left, left);
		outR[curSmpl] = ws_dc_process(&chip->dc_right, right);
	}
}

static void ws_reset(void* info)
{
	wsa_state* chip = (wsa_state*)info;
	size_t first = offsetof(wsa_state, chn);
	UINT8 chn;

	memset((UINT8*)chip + first, 0x00, sizeof(wsa_state) - first);
	for (chn = 0; chn < 4; chn ++)
		chip->chn[chn].counter = WS_PITCH_MASK + 1;	// period 0, pitch 0
	chip->headphones_enable = 1;	// see the port notes
	chip->mix_stale = WS_MIX_ALL;
}

static UINT8 device_start_wswan(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	wsa_state* chip;

	chip = (wsa_state*)calloc(1, sizeof(wsa_state));
	if (chip == NULL)
		return 0xFF;

	chip->clock = cfg->clock;
	if (! chip->clock)
		chip->clock = WS_DEFAULT_CLOCK;
	chip->rate = chip->clock / WS_CLOCKS_PER_SAMPLE;
	ws_reset(chip);
	ws_set_mute_mask(chip, 0x00);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, chip->rate, &devDef);
	return 0x00;
}

static void device_stop_wswan(void* info)
{
	wsa_state* chip = (wsa_state*)info;

	free(chip);
}

// APU::writeIO() for the ports 0x80 ... 0x9F that are not read only
static void ws_write_reg(void* info, UINT8 port, UINT8 data)
{
	wsa_state* chip = (wsa_state*)info;
	WS_CHANNEL* chn;

	switch(port)
	{
	case 0x80:	// SND_CH1_PITCH
	case 0x82:	// SND_CH2_PITCH
	case 0x84:	// SND_CH3_PITCH
	case 0x86:	// SND_CH4_PITCH
		chn = &chip->chn[(port - 0x80) >> 1];
		ws_chn_set_pitch(chn, (UINT16)((chn->pitch & 0x0700) | data));
		break;
	case 0x81:
	case 0x83:
	case 0x85:
	case 0x87:
		chn = &chip->chn[(port - 0x80) >> 1];
		ws_chn_set_pitch(chn, (UINT16)((chn->pitch & 0x00FF) | ((data & 0x07) << 8)));
		break;
	case 0x88:	// SND_CH1_VOL
	case 0x89:	// SND_CH2_VOL
	case 0x8A:	// SND_CH3_VOL
	case 0x8B:	// SND_CH4_VOL
		chn = &chip->chn[port - 0x88];
		chn->volume_right = data & 0x0F;
		chn->volume_left = data >> 4;
		// only this channel's volume changes, and the voice (the channel 2 volume) in the base
		chip->mix_stale |= (UINT8)((1 << (port - 0x88)) | (port == 0x89 ? WS_MIX_BASE : 0));
		return;
	case 0x8C:	// SND_SWEEP_VALUE
		chip->sweep_value = (INT8)data;
		break;
	case 0x8D:	// SND_SWEEP_TIME
		chip->sweep_time = data & 0x1F;
		break;
	case 0x8E:	// SND_NOISE
		chip->noise_mode = data & 0x07;
		chip->noise_update = (data >> 4) & 1;
		if (data & 0x08)
			chip->noise_lfsr = 0;
		break;
	case 0x8F:	// SND_WAVE_BASE
		chip->wave_base = data;
		break;
	case 0x90:	// SND_CTRL
		chip->chn[0].enable = data & 1;
		chip->chn[1].enable = (data >> 1) & 1;
		chip->chn[2].enable = (data >> 2) & 1;
		chip->chn[3].enable = (data >> 3) & 1;
		chip->voice = (data >> 5) & 1;
		chip->sweep = (data >> 6) & 1;
		chip->noise = (data >> 7) & 1;
		break;
	case 0x91:	// SND_OUTPUT
		chip->speaker_enable = data & 1;
		chip->speaker_shift = (data >> 1) & 3;
		chip->headphones_enable = (data >> 3) & 1;
		break;
	case 0x94:	// SND_VOICE_CTRL
		chip->voice_right_half = data & 1;
		chip->voice_right_full = (data >> 1) & 1;
		chip->voice_left_half = (data >> 2) & 1;
		chip->voice_left_full = (data >> 3) & 1;
		break;
	case 0x95:	// sequencer debug port
		chip->seq_dbg_hold = data & 1;
		chip->seq_dbg_sweep_clock = (data >> 1) & 1;
		chip->seq_dbg_noise = (data >> 2) & 3;
		chip->seq_dbg_output_force55 = (data >> 5) & 1;
		chip->seq_dbg_ch_force2 = (data >> 6) & 1;
		chip->seq_dbg_ch_force4 = (data >> 7) & 1;
		break;
	// 0x9E (SND_VOLUME) is ignored, see the port notes
	}
	chip->mix_stale = WS_MIX_ALL;
}

static void ws_write_ram(void* info, UINT16 offset, UINT8 data)
{
	wsa_state* chip = (wsa_state*)info;

	if (offset < WS_RAM_SIZE)
	{
		chip->ram[offset] = data;
		chip->mix_stale |= 0x0F;	// the levels of all channels (the wave tables)
	}
}

static void ws_set_mute_mask(void* info, UINT32 MuteMask)
{
	wsa_state* chip = (wsa_state*)info;

	chip->mute = (UINT8)(MuteMask & 0x0F);
	chip->mix_stale = WS_MIX_ALL;
}
