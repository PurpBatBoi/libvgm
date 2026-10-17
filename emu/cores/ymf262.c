// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Yamaha YMF262 (OPL3)

    C port of the OPL3 path of ymfm (ymfm_opl.cpp, ymfm_fm.ipp, ymfm.h).
    https://github.com/aaronsgiles/ymfm

    BSD 3-Clause License

    Copyright (c) 2021, Aaron Giles
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    Port notes:
    - Only the OPL3 parts of the ymfm FM engine are kept: 18 two-operator
      channels in 2 register banks, 4-operator pairing (reg 0x104), the
      rhythm section and 4 outputs. No OPN SSG-EG, OPM noise, OPLL depress
      or OPZ reverb states. OPL3 has no CSM.
    - ymfm leaves timers to its host. Here they count in chip samples
      (clock / 288) and expire before the sample is clocked.
    - One update sample is one chip sample (clock / 288). The 4 outputs are
      mixed to stereo like libvgm's MAME core: left = A + C, right = B + D.
      Each output is clamped to 16 bits first, as ymfm does.
    - Change from ymfm: ymfm samples the key state once per sample, so a
      key off followed by a key on before the next sample was lost and the
      note did not restart. Sound drivers commonly write them back to back.
      Here a key off is remembered until the next sample, so the operator
      releases and then attacks again, as on the real chip.

***************************************************************************/

#include <stdlib.h>
#include <string.h>	// for memset

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"

#include "ymf262.h"


static void ymf262_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ymf262(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ymf262(void* info);
static void device_reset_ymf262(void* info);
static void ymf262_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ymf262_r(void* info, UINT8 offset);
static void ymf262_set_mute_mask(void* info, UINT32 MuteMask);


static DEVDEF_RWFUNC devFunc262[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ymf262_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ymf262_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ymf262_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef262 =
{
	"YMF262", "ymfm", FCC_YMFM,

	device_start_ymf262,
	device_stop_ymf262,
	device_reset_ymf262,
	ymf262_update,

	NULL,	// SetOptionBits
	ymf262_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc262,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "YMF262";
}

#define DEV_CHN_COUNT	23
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT] =
	{
		"1", "2", "3", "4", "5", "6", "7", "8", "9",
		"10", "11", "12", "13", "14", "15", "16", "17", "18",
		"Bass Drum", "Snare Drum", "Tom Tom", "Cymbal", "Hi-Hat",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YMF262 =
{
	DEVID_YMF262,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef262,
		NULL
	}
};


#define CHANNELS		18
#define OPERATORS		(CHANNELS * 2)
#define ALL_CHANNELS	((1 << CHANNELS) - 1)
#define OUTPUTS			4
#define REGISTERS		0x200
#define WAVEFORMS		8
#define WAVEFORM_LENGTH	0x400
#define REG_MODE		0x04
#define RHYTHM_CHANNEL	0xff
#define OP_NONE			0xff	// unused operator slot of a channel

#define STATUS_TIMERA	0x40
#define STATUS_TIMERB	0x20
#define STATUS_IRQ		0x80
#define IRQ_MASK		(STATUS_TIMERA | STATUS_TIMERB)

// mute mask bits beyond the 18 melodic channels
// (a 4-operator channel uses the bit of its first channel)
#define MUTE_BD		18
#define MUTE_SD		19
#define MUTE_TT		20
#define MUTE_CY		21
#define MUTE_HH		22

// envelope states
#define EG_ATTACK	1
#define EG_DECAY	2
#define EG_SUSTAIN	3
#define EG_RELEASE	4
#define EG_STATES	6

// "quiet" value, used to optimize when we can skip doing work
#define EG_QUIET	0x380

// key on sources; actual keyon is an OR over all of these
#define KEYON_NORMAL	0
#define KEYON_RHYTHM	1

// set phase_step to this value to recalculate it each sample; needed
// in the case of PM LFO changes
#define PHASE_STEP_DYNAMIC	1

// data computed once at the start of clocking that remains static during
// subsequent sound generation
typedef struct _opdata_cache
{
	const UINT16* waveform;	// base of sine table
	UINT32 phase_step;		// phase step, or PHASE_STEP_DYNAMIC if PM is active
	UINT32 total_level;		// total level * 8 + KSL
	UINT32 block_freq;		// raw block frequency value (used to compute phase_step)
	UINT32 multiple;		// multiple value (x.1, used to compute phase_step)
	UINT32 eg_sustain;		// sustain level, shifted up to envelope values
	UINT8 eg_rate[EG_STATES];	// envelope rate, including KSR
} OPDATA_CACHE;

typedef struct _fm_operator
{
	UINT32 choffs;			// channel offset in registers
	UINT32 opoffs;			// operator offset in registers
	UINT32 phase;			// current phase value (10.10 format)
	UINT16 env_attenuation;	// computed envelope attenuation (4.6 format)
	UINT8 env_state;		// current envelope state
	UINT8 key_state;		// current key state: on or off (bit 0)
	UINT8 keyon_live;		// live key on state (bit 0 = direct, bit 1 = rhythm)
	OPDATA_CACHE cache;		// cached values for performance
	UINT8 keyoff_pending;	// the live key went off since the last prepare
} FM_OPERATOR;

typedef struct _fm_channel
{
	UINT32 choffs;			// channel offset in registers
	INT16 feedback[2];		// feedback memory for operator 1
	INT16 feedback_in;		// next input value for op 1 feedback (set in output)
	UINT8 op[4];			// operator indices, OP_NONE if unused
} FM_CHANNEL;

typedef struct _ymf262_state
{
	DEV_DATA _devData;

	// OPL registers
	UINT16 lfo_am_counter;	// LFO AM counter
	UINT16 lfo_pm_counter;	// LFO PM counter
	UINT32 noise_lfsr;		// noise LFSR state
	UINT8 lfo_am;			// current LFO AM value
	UINT8 regdata[REGISTERS];	// register data
	UINT16 waveform[WAVEFORMS][WAVEFORM_LENGTH];	// waveforms

	// FM engine
	UINT32 env_counter;		// envelope counter; low 2 bits are sub-counter
	UINT8 status;			// current status register
	UINT8 timer_running[2];	// current timer running state
	UINT8 total_clocks;		// low 8 bits of the total number of clocks processed
	UINT32 active_channels;	// mask of active channels (computed by prepare)
	UINT32 modified_channels;	// mask of channels that have been modified
	UINT32 prepare_count;	// counter to do periodic prepare sweeps
	INT32 timer_count[2];	// samples until the timer expires, 0 = stopped
	FM_CHANNEL channel[CHANNELS];
	FM_OPERATOR op[OPERATORS];

	// chip
	UINT16 address;			// address register

	UINT32 muteMask;
} YMF262_STATE;


//----------------------------------------------------------------------------
// tables
//----------------------------------------------------------------------------

// sine attenuation as 4.8 logarithmic values for 1/4 phase
// this matches the internal format of the OPN chip, extracted from the die
static const UINT16 s_sin_table[256] =
{
	0x859,0x6c3,0x607,0x58b,0x52e,0x4e4,0x4a6,0x471,0x443,0x41a,0x3f5,0x3d3,0x3b5,0x398,0x37e,0x365,
	0x34e,0x339,0x324,0x311,0x2ff,0x2ed,0x2dc,0x2cd,0x2bd,0x2af,0x2a0,0x293,0x286,0x279,0x26d,0x261,
	0x256,0x24b,0x240,0x236,0x22c,0x222,0x218,0x20f,0x206,0x1fd,0x1f5,0x1ec,0x1e4,0x1dc,0x1d4,0x1cd,
	0x1c5,0x1be,0x1b7,0x1b0,0x1a9,0x1a2,0x19b,0x195,0x18f,0x188,0x182,0x17c,0x177,0x171,0x16b,0x166,
	0x160,0x15b,0x155,0x150,0x14b,0x146,0x141,0x13c,0x137,0x133,0x12e,0x129,0x125,0x121,0x11c,0x118,
	0x114,0x10f,0x10b,0x107,0x103,0x0ff,0x0fb,0x0f8,0x0f4,0x0f0,0x0ec,0x0e9,0x0e5,0x0e2,0x0de,0x0db,
	0x0d7,0x0d4,0x0d1,0x0cd,0x0ca,0x0c7,0x0c4,0x0c1,0x0be,0x0bb,0x0b8,0x0b5,0x0b2,0x0af,0x0ac,0x0a9,
	0x0a7,0x0a4,0x0a1,0x09f,0x09c,0x099,0x097,0x094,0x092,0x08f,0x08d,0x08a,0x088,0x086,0x083,0x081,
	0x07f,0x07d,0x07a,0x078,0x076,0x074,0x072,0x070,0x06e,0x06c,0x06a,0x068,0x066,0x064,0x062,0x060,
	0x05e,0x05c,0x05b,0x059,0x057,0x055,0x053,0x052,0x050,0x04e,0x04d,0x04b,0x04a,0x048,0x046,0x045,
	0x043,0x042,0x040,0x03f,0x03e,0x03c,0x03b,0x039,0x038,0x037,0x035,0x034,0x033,0x031,0x030,0x02f,
	0x02e,0x02d,0x02b,0x02a,0x029,0x028,0x027,0x026,0x025,0x024,0x023,0x022,0x021,0x020,0x01f,0x01e,
	0x01d,0x01c,0x01b,0x01a,0x019,0x018,0x017,0x017,0x016,0x015,0x014,0x014,0x013,0x012,0x011,0x011,
	0x010,0x00f,0x00f,0x00e,0x00d,0x00d,0x00c,0x00c,0x00b,0x00a,0x00a,0x009,0x009,0x008,0x008,0x007,
	0x007,0x007,0x006,0x006,0x005,0x005,0x005,0x004,0x004,0x004,0x003,0x003,0x003,0x002,0x002,0x002,
	0x002,0x001,0x001,0x001,0x001,0x001,0x001,0x001,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000
};

// 10-bit mantissas with an implied leading bit, matching the internal format
// of the OPN chip; the implicit 0x400 bit is pre-incorporated, the values are
// left-shifted by 2 and the order is reversed to save a NOT on the input
#define X(a) (((a) | 0x400) << 2)
static const UINT16 s_power_table[256] =
{
	X(0x3fa),X(0x3f5),X(0x3ef),X(0x3ea),X(0x3e4),X(0x3df),X(0x3da),X(0x3d4),
	X(0x3cf),X(0x3c9),X(0x3c4),X(0x3bf),X(0x3b9),X(0x3b4),X(0x3ae),X(0x3a9),
	X(0x3a4),X(0x39f),X(0x399),X(0x394),X(0x38f),X(0x38a),X(0x384),X(0x37f),
	X(0x37a),X(0x375),X(0x370),X(0x36a),X(0x365),X(0x360),X(0x35b),X(0x356),
	X(0x351),X(0x34c),X(0x347),X(0x342),X(0x33d),X(0x338),X(0x333),X(0x32e),
	X(0x329),X(0x324),X(0x31f),X(0x31a),X(0x315),X(0x310),X(0x30b),X(0x306),
	X(0x302),X(0x2fd),X(0x2f8),X(0x2f3),X(0x2ee),X(0x2e9),X(0x2e5),X(0x2e0),
	X(0x2db),X(0x2d6),X(0x2d2),X(0x2cd),X(0x2c8),X(0x2c4),X(0x2bf),X(0x2ba),
	X(0x2b5),X(0x2b1),X(0x2ac),X(0x2a8),X(0x2a3),X(0x29e),X(0x29a),X(0x295),
	X(0x291),X(0x28c),X(0x288),X(0x283),X(0x27f),X(0x27a),X(0x276),X(0x271),
	X(0x26d),X(0x268),X(0x264),X(0x25f),X(0x25b),X(0x257),X(0x252),X(0x24e),
	X(0x249),X(0x245),X(0x241),X(0x23c),X(0x238),X(0x234),X(0x230),X(0x22b),
	X(0x227),X(0x223),X(0x21e),X(0x21a),X(0x216),X(0x212),X(0x20e),X(0x209),
	X(0x205),X(0x201),X(0x1fd),X(0x1f9),X(0x1f5),X(0x1f0),X(0x1ec),X(0x1e8),
	X(0x1e4),X(0x1e0),X(0x1dc),X(0x1d8),X(0x1d4),X(0x1d0),X(0x1cc),X(0x1c8),
	X(0x1c4),X(0x1c0),X(0x1bc),X(0x1b8),X(0x1b4),X(0x1b0),X(0x1ac),X(0x1a8),
	X(0x1a4),X(0x1a0),X(0x19c),X(0x199),X(0x195),X(0x191),X(0x18d),X(0x189),
	X(0x185),X(0x181),X(0x17e),X(0x17a),X(0x176),X(0x172),X(0x16f),X(0x16b),
	X(0x167),X(0x163),X(0x160),X(0x15c),X(0x158),X(0x154),X(0x151),X(0x14d),
	X(0x149),X(0x146),X(0x142),X(0x13e),X(0x13b),X(0x137),X(0x134),X(0x130),
	X(0x12c),X(0x129),X(0x125),X(0x122),X(0x11e),X(0x11b),X(0x117),X(0x114),
	X(0x110),X(0x10c),X(0x109),X(0x106),X(0x102),X(0x0ff),X(0x0fb),X(0x0f8),
	X(0x0f4),X(0x0f1),X(0x0ed),X(0x0ea),X(0x0e7),X(0x0e3),X(0x0e0),X(0x0dc),
	X(0x0d9),X(0x0d6),X(0x0d2),X(0x0cf),X(0x0cc),X(0x0c8),X(0x0c5),X(0x0c2),
	X(0x0be),X(0x0bb),X(0x0b8),X(0x0b5),X(0x0b1),X(0x0ae),X(0x0ab),X(0x0a8),
	X(0x0a4),X(0x0a1),X(0x09e),X(0x09b),X(0x098),X(0x094),X(0x091),X(0x08e),
	X(0x08b),X(0x088),X(0x085),X(0x082),X(0x07e),X(0x07b),X(0x078),X(0x075),
	X(0x072),X(0x06f),X(0x06c),X(0x069),X(0x066),X(0x063),X(0x060),X(0x05d),
	X(0x05a),X(0x057),X(0x054),X(0x051),X(0x04e),X(0x04b),X(0x048),X(0x045),
	X(0x042),X(0x03f),X(0x03c),X(0x039),X(0x036),X(0x033),X(0x030),X(0x02d),
	X(0x02a),X(0x028),X(0x025),X(0x022),X(0x01f),X(0x01c),X(0x019),X(0x016),
	X(0x014),X(0x011),X(0x00e),X(0x00b),X(0x008),X(0x006),X(0x003),X(0x000)
};
#undef X

// 4-bit attenuation increments for a 6-bit ADSR rate and a 3-bit step index
static const UINT32 s_increment_table[64] =
{
	0x00000000, 0x00000000, 0x10101010, 0x10101010,  // 0-3    (0x00-0x03)
	0x10101010, 0x10101010, 0x11101110, 0x11101110,  // 4-7    (0x04-0x07)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 8-11   (0x08-0x0B)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 12-15  (0x0C-0x0F)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 16-19  (0x10-0x13)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 20-23  (0x14-0x17)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 24-27  (0x18-0x1B)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 28-31  (0x1C-0x1F)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 32-35  (0x20-0x23)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 36-39  (0x24-0x27)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 40-43  (0x28-0x2B)
	0x10101010, 0x10111010, 0x11101110, 0x11111110,  // 44-47  (0x2C-0x2F)
	0x11111111, 0x21112111, 0x21212121, 0x22212221,  // 48-51  (0x30-0x33)
	0x22222222, 0x42224222, 0x42424242, 0x44424442,  // 52-55  (0x34-0x37)
	0x44444444, 0x84448444, 0x84848484, 0x88848884,  // 56-59  (0x38-0x3B)
	0x88888888, 0x88888888, 0x88888888, 0x88888888   // 60-63  (0x3C-0x3F)
};

// key scale attenuation for the top 4 bits of FNUM; these are the maximal
// values (for when block == 7), values for other blocks are 8 less per block
static const UINT8 s_fnum_to_atten[16] = { 0,24,32,37,40,43,45,47,48,50,51,52,53,54,55,56 };

// PM LFO steps, as a 1.3 fraction and sign to apply to the upper FNUM bits
static const INT8 s_lfo_pm_scale[8] = { 8, 4, 0, -4, -8, -4, 0, 4 };


//----------------------------------------------------------------------------
// helpers
//----------------------------------------------------------------------------

INLINE UINT32 bitfield(UINT32 value, int start, int length)
{
	return (value >> start) & ((1 << length) - 1);
}

INLINE INT32 clamp(INT32 value, INT32 minval, INT32 maxval)
{
	if (value < minval)
		return minval;
	if (value > maxval)
		return maxval;
	return value;
}

// return the absolute value of sin(input) as a 4.8 attenuation value,
// where the range 0-2*PI is mapped onto 10 bits
static UINT32 abs_sin_attenuation(UINT32 input)
{
	// if the top bit is set, we're in the second half of the curve
	// which is a mirror image, so invert the index
	if (bitfield(input, 8, 1))
		input = ~input;
	return s_sin_table[input & 0xff];
}

// given a 5.8 fixed point logarithmic attenuation value, return a 13-bit
// linear volume
INLINE UINT32 attenuation_to_volume(UINT32 input)
{
	// look up the fractional part, then shift by the whole
	return s_power_table[input & 0xff] >> (input >> 8);
}

// given a 6-bit ADSR rate value and a 3-bit stepping index, return a 4-bit
// increment to the attenuation for this step (or for the attack case, the
// fractional scale factor to decrease by)
INLINE UINT32 attenuation_increment(UINT32 rate, UINT32 index)
{
	return bitfield(s_increment_table[rate], 4 * index, 4);
}

// apply KSR to the raw ADSR rate, ignoring ksr if the raw value is 0, and
// clamping to 63
INLINE UINT32 effective_rate(UINT32 rawrate, UINT32 ksr)
{
	if (rawrate == 0)
		return 0;
	return (rawrate + ksr < 63) ? (rawrate + ksr) : 63;
}

// convert an OPL concatenated block (3 bits) and the top 4 bits of fnum into
// an attenuation offset; values are for 6dB/octave, in 0.75dB units
// (matching total level LSB)
static UINT32 opl_key_scale_atten(UINT32 block, UINT32 fnum_4msb)
{
	INT32 result = s_fnum_to_atten[fnum_4msb] - 8 * (INT32)(block ^ 7);
	return (result > 0) ? (UINT32)result : 0;
}


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

#define REG(offset)		(chip->regdata[offset])

// system-wide registers
#define timer_a_value()		(bitfield(REG(0x02), 0, 8) * 4)	// 8->10 bits
#define timer_b_value()		bitfield(REG(0x03), 0, 8)
#define status_mask()		(bitfield(REG(0x04), 0, 8) & 0x78)
#define irq_reset()			bitfield(REG(0x04), 7, 1)
#define reset_timer_b()		(bitfield(REG(0x04), 7, 1) | bitfield(REG(0x04), 5, 1))
#define reset_timer_a()		(bitfield(REG(0x04), 7, 1) | bitfield(REG(0x04), 6, 1))
#define load_timer_b()		bitfield(REG(0x04), 1, 1)
#define load_timer_a()		bitfield(REG(0x04), 0, 1)
#define note_select()		bitfield(REG(0x08), 6, 1)
#define lfo_am_depth()		bitfield(REG(0xbd), 7, 1)
#define lfo_pm_depth()		bitfield(REG(0xbd), 6, 1)
#define rhythm_enable()		bitfield(REG(0xbd), 5, 1)
#define newflag()			bitfield(REG(0x105), 0, 1)
#define fourop_enable()		bitfield(REG(0x104), 0, 6)

// per-channel registers
#define ch_block_freq(choffs)	((bitfield(REG(0xb0 + (choffs)), 0, 5) << 8) | bitfield(REG(0xa0 + (choffs)), 0, 8))
#define ch_feedback(choffs)		bitfield(REG(0xc0 + (choffs)), 1, 3)
#define ch_algorithm(choffs)	(bitfield(REG(0xc0 + (choffs)), 0, 1) | 8 | (bitfield(REG(0xc3 + (choffs)), 0, 1) << 1))
#define ch_output_any(choffs)	(newflag() ? bitfield(REG(0xc0 + (choffs)), 4, 4) : 1)
#define ch_output_0(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 4, 1) : 1)
#define ch_output_1(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 5, 1) : 1)
#define ch_output_2(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 6, 1) : 0)
#define ch_output_3(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 7, 1) : 0)

// per-operator registers
#define op_lfo_am_enable(opoffs)	bitfield(REG(0x20 + (opoffs)), 7, 1)
#define op_lfo_pm_enable(opoffs)	bitfield(REG(0x20 + (opoffs)), 6, 1)
#define op_eg_sustain(opoffs)	bitfield(REG(0x20 + (opoffs)), 5, 1)
#define op_ksr(opoffs)			bitfield(REG(0x20 + (opoffs)), 4, 1)
#define op_multiple(opoffs)		bitfield(REG(0x20 + (opoffs)), 0, 4)
#define op_ksl(opoffs)			(bitfield(REG(0x40 + (opoffs)), 7, 1) | (bitfield(REG(0x40 + (opoffs)), 6, 1) << 1))
#define op_total_level(opoffs)	bitfield(REG(0x40 + (opoffs)), 0, 6)
#define op_attack_rate(opoffs)	bitfield(REG(0x60 + (opoffs)), 4, 4)
#define op_decay_rate(opoffs)	bitfield(REG(0x60 + (opoffs)), 0, 4)
#define op_sustain_level(opoffs)	bitfield(REG(0x80 + (opoffs)), 4, 4)
#define op_release_rate(opoffs)	bitfield(REG(0x80 + (opoffs)), 0, 4)
#define op_waveform(opoffs)		bitfield(REG(0xe0 + (opoffs)), 0, newflag() ? 3 : 2)

static void regs_init(YMF262_STATE* chip)
{
	UINT32 index;
	UINT16 zeroval;

	chip->lfo_am_counter = 0;
	chip->lfo_pm_counter = 0;
	chip->noise_lfsr = 1;
	chip->lfo_am = 0;

	// create the waveforms
	for (index = 0; index < WAVEFORM_LENGTH; index ++)
		chip->waveform[0][index] = (UINT16)(abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	zeroval = chip->waveform[0][0];
	for (index = 0; index < WAVEFORM_LENGTH; index ++)
	{
		chip->waveform[1][index] = bitfield(index, 9, 1) ? zeroval : chip->waveform[0][index];
		chip->waveform[2][index] = chip->waveform[0][index] & 0x7fff;
		chip->waveform[3][index] = bitfield(index, 8, 1) ? zeroval : (chip->waveform[0][index] & 0x7fff);
		chip->waveform[4][index] = bitfield(index, 9, 1) ? zeroval : chip->waveform[0][index * 2];
		chip->waveform[5][index] = bitfield(index, 9, 1) ? zeroval : chip->waveform[0][(index * 2) & 0x1ff];
		chip->waveform[6][index] = (UINT16)(bitfield(index, 9, 1) << 15);
		chip->waveform[7][index] = (UINT16)((bitfield(index, 9, 1) ? (index ^ 0x13ff) : index) << 3);
	}

	// the operator map is built right away, prior to reset
	REG(0x104) = 0;
	return;
}

static void regs_reset(YMF262_STATE* chip)
{
	memset(chip->regdata, 0x00, REGISTERS);
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 regs_write(YMF262_STATE* chip, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	// writes to the mode register with high bit set ignore the low bits
	if (index == REG_MODE && bitfield(data, 7, 1) != 0)
		REG(index) |= 0x80;
	else
		REG(index) = data;

	// handle writes to the rhythm keyons
	if (index == 0xbd)
	{
		*channel = RHYTHM_CHANNEL;
		*opmask = bitfield(data, 5, 1) ? bitfield(data, 0, 5) : 0;
		return 1;
	}

	// handle writes to the channel keyons
	if ((index & 0xf0) == 0xb0)
	{
		*channel = index & 0x0f;
		if (*channel < 9)
		{
			*channel += 9 * bitfield(index, 8, 1);
			*opmask = bitfield(data, 5, 1) ? 15 : 0;
			return 1;
		}
	}
	return 0;
}

// clock the noise and LFO, returning the LFO PM value
static INT32 clock_noise_and_lfo(YMF262_STATE* chip)
{
	UINT32 am_counter;
	UINT32 pm_counter;
	int shift;

	// OPL has a 23-bit noise generator for the rhythm section, running at
	// a constant rate, used only for percussion input
	chip->noise_lfsr <<= 1;
	chip->noise_lfsr |= bitfield(chip->noise_lfsr, 23, 1) ^ bitfield(chip->noise_lfsr, 9, 1) ^
		bitfield(chip->noise_lfsr, 8, 1) ^ bitfield(chip->noise_lfsr, 1, 1);

	// OPL has two fixed-frequency LFOs, one for AM, one for PM

	// the AM LFO has 210*64 steps; at a nominal 50kHz output,
	// this equates to a period of 50000/(210*64) = 3.72Hz
	am_counter = chip->lfo_am_counter++;
	if (am_counter >= 210*64 - 1)
		chip->lfo_am_counter = 0;

	// low 8 bits are fractional; depth 0 is divided by 2, while depth 1 is times 2
	shift = 9 - 2 * lfo_am_depth();

	// AM value is the upper bits of the value, inverted across the midpoint
	// to produce a triangle
	chip->lfo_am = (UINT8)(((am_counter < 105*64) ? am_counter : (210*64+63 - am_counter)) >> shift);

	// the PM LFO has 8192 steps, or a nominal period of 6.1Hz
	pm_counter = chip->lfo_pm_counter++;

	// PM LFO is broken into 8 chunks, each lasting 1024 steps; the PM value
	// depends on the upper bits of FNUM, so this value is a fraction and
	// sign to apply to that value, as a 1.3 value
	return s_lfo_pm_scale[bitfield(pm_counter, 10, 3)] >> (lfo_pm_depth() ^ 1);
}

// return the current noise state
#define noise_state()	(chip->noise_lfsr >> 23)

// compute the phase step, given a PM value
static UINT32 compute_phase_step(const YMF262_STATE* chip, UINT32 opoffs, const OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	UINT32 fnum;
	UINT32 block;
	UINT32 phase_step;

	// OPL phase calculation has no detuning, but uses FNUMs like
	// the OPN version, and computes PM a bit differently
	if (! op_lfo_pm_enable(opoffs))
		lfo_raw_pm = 0;

	// extract frequency number as a 12-bit fraction
	fnum = bitfield(cache->block_freq, 0, 10) << 2;

	// apply the phase adjustment based on the upper 3 bits
	// of FNUM and the PM depth parameters
	fnum += ((UINT32)lfo_raw_pm * bitfield(cache->block_freq, 7, 3)) >> 1;

	// keep fnum to 12 bits
	fnum &= 0xfff;

	// apply block shift to compute phase step
	block = bitfield(cache->block_freq, 10, 3);
	phase_step = (fnum << block) >> 2;

	// apply frequency multiplier (which is cached as an x.1 value)
	return (phase_step * cache->multiple) >> 1;
}

// fill the operator cache with prefetched data
static void cache_operator_data(const YMF262_STATE* chip, UINT32 choffs, UINT32 opoffs, OPDATA_CACHE* cache)
{
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 multiple;
	UINT32 ksl;
	UINT32 ksrval;

	// set up the easy stuff
	cache->waveform = chip->waveform[op_waveform(opoffs)];

	// get frequency from the channel
	block_freq = cache->block_freq = ch_block_freq(choffs);

	// compute the keycode: block_freq is:
	//
	//     111  |
	//     21098|76543210
	//     BBBFF|FFFFFFFF
	//     ^^^??
	//
	// the 4-bit keycode uses the top 3 bits plus one of the next two bits
	keycode = bitfield(block_freq, 10, 3) << 1;

	// lowest bit is determined by note_select(); note that it is
	// actually reversed from what the manual says, however
	keycode |= bitfield(block_freq, 9 - note_select(), 1);

	// multiple value, as an x.1 value (0 means 0.5)
	// replace the low bit with a table lookup to give 0,1,2,3,4,5,6,7,8,9,10,10,12,12,15,15
	multiple = op_multiple(opoffs);
	cache->multiple = ((multiple & 0xe) | bitfield(0xc2aa, multiple, 1)) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on block_freq
	// and multiple, so compute it after we've done those
	if (op_lfo_pm_enable(opoffs) == 0)
		cache->phase_step = compute_phase_step(chip, opoffs, cache, 0);
	else
		cache->phase_step = PHASE_STEP_DYNAMIC;

	// total level, scaled by 8
	cache->total_level = op_total_level(opoffs) << 3;

	// pre-add key scale level
	ksl = op_ksl(opoffs);
	if (ksl != 0)
		cache->total_level += opl_key_scale_atten(bitfield(block_freq, 10, 3), bitfield(block_freq, 6, 4)) << ksl;

	// 4-bit sustain level, but 15 means 31 so effectively 5 bits
	cache->eg_sustain = op_sustain_level(opoffs);
	cache->eg_sustain |= (cache->eg_sustain + 1) & 0x10;
	cache->eg_sustain <<= 5;

	// determine KSR adjustment for envelope rates
	ksrval = keycode >> (2 * (op_ksr(opoffs) ^ 1));
	cache->eg_rate[EG_ATTACK] = (UINT8)effective_rate(op_attack_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_DECAY] = (UINT8)effective_rate(op_decay_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_SUSTAIN] = op_eg_sustain(opoffs) ? 0 : (UINT8)effective_rate(op_release_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_RELEASE] = (UINT8)effective_rate(op_release_rate(opoffs) * 4, ksrval);
	return;
}


//----------------------------------------------------------------------------
// operator
//----------------------------------------------------------------------------

static void op_reset(FM_OPERATOR* op)
{
	op->phase = 0;
	op->env_attenuation = 0x3ff;
	op->env_state = EG_RELEASE;
	op->key_state = 0;
	op->keyon_live = 0;
	op->keyoff_pending = 0;
	return;
}

// signal key on/off from one source
static void op_keyonoff(FM_OPERATOR* op, UINT32 on, UINT8 type)
{
	UINT8 old_live = op->keyon_live;

	op->keyon_live = (UINT8)((op->keyon_live & ~(1 << type)) | (bitfield(on, 0, 1) << type));
	if (old_live != 0 && op->keyon_live == 0)
		op->keyoff_pending = 1;
	return;
}

// start the attack phase; called when a keyon happens
static void op_start_attack(FM_OPERATOR* op)
{
	// don't change anything if already in attack state
	if (op->env_state == EG_ATTACK)
		return;
	op->env_state = EG_ATTACK;

	// reset the phase when we start an attack due to a key on
	op->phase = 0;

	// if the attack rate >= 62 then immediately go to max attenuation
	if (op->cache.eg_rate[EG_ATTACK] >= 62)
		op->env_attenuation = 0;
	return;
}

// start the release phase; called when a keyoff happens
static void op_start_release(FM_OPERATOR* op)
{
	// don't change anything if already in release state
	if (op->env_state >= EG_RELEASE)
		return;
	op->env_state = EG_RELEASE;
	return;
}

// clock the keystate to match the incoming keystate
static void op_clock_keystate(FM_OPERATOR* op, UINT32 keystate)
{
	// has the key changed?
	if ((keystate ^ op->key_state) != 0)
	{
		op->key_state = (UINT8)keystate;

		// if the key has turned on, start the attack
		if (keystate != 0)
			op_start_attack(op);
		// otherwise, start the release
		else
			op_start_release(op);
	}
	return;
}

// prepare prior to clocking; returns non-zero while the operator is audible
static UINT8 op_prepare(const YMF262_STATE* chip, FM_OPERATOR* op)
{
	// cache the data
	cache_operator_data(chip, op->choffs, op->opoffs, &op->cache);

	// clock the key state; a key off since the last prepare releases first,
	// so a key off + key on within one sample restarts the note
	if (op->keyoff_pending)
	{
		op_clock_keystate(op, 0);
		op->keyoff_pending = 0;
	}
	op_clock_keystate(op, (UINT32)(op->keyon_live != 0));

	// we're active until we're quiet after the release
	return (op->env_state != EG_RELEASE || op->env_attenuation < EG_QUIET);
}

// clock the envelope state according to the given count
static void op_clock_envelope(FM_OPERATOR* op, UINT32 env_counter)
{
	UINT32 rate;
	UINT32 rate_shift;
	UINT32 relevant_bits;
	UINT32 increment;

	// handle attack->decay transitions
	if (op->env_state == EG_ATTACK && op->env_attenuation == 0)
		op->env_state = EG_DECAY;

	// handle decay->sustain transitions; it is important to do this immediately
	// after the attack->decay transition above in the event that the sustain level
	// is set to 0 (in which case we will skip right to sustain without doing any
	// decay); as an example where this can be heard, check the cymbals sound
	// in channel 0 of shinobi's test mode sound #5
	if (op->env_state == EG_DECAY && op->env_attenuation >= op->cache.eg_sustain)
		op->env_state = EG_SUSTAIN;

	// fetch the appropriate 6-bit rate value from the cache
	rate = op->cache.eg_rate[op->env_state];

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
	if (op->env_state == EG_ATTACK)
	{
		// glitch means that attack rates of 62/63 don't increment if
		// changed after the initial key on (where they are handled
		// specially); nukeykt confirms this happens on OPM, OPN, OPL/OPLL
		// at least so assuming it is true for everyone
		if (rate < 62)
			op->env_attenuation = (UINT16)(op->env_attenuation + ((~(UINT32)op->env_attenuation * increment) >> 4));
	}
	// all other cases are similar
	else
	{
		// apply the increment
		op->env_attenuation = (UINT16)(op->env_attenuation + increment);

		// clamp the final attenuation
		if (op->env_attenuation >= 0x400)
			op->env_attenuation = 0x3ff;
	}
	return;
}

// clock the 10.10 phase value
static void op_clock_phase(const YMF262_STATE* chip, FM_OPERATOR* op, INT32 lfo_raw_pm)
{
	// read from the cache, or recalculate if PM active
	UINT32 phase_step = op->cache.phase_step;
	if (phase_step == PHASE_STEP_DYNAMIC)
		phase_step = compute_phase_step(chip, op->opoffs, &op->cache, lfo_raw_pm);

	// finally apply the step to the current phase value
	op->phase += phase_step;
	return;
}

// master clocking function
static void op_clock(const YMF262_STATE* chip, FM_OPERATOR* op, UINT32 env_counter, INT32 lfo_raw_pm)
{
	// clock the envelope if on an envelope cycle; env_counter is a x.2 value
	if (bitfield(env_counter, 0, 2) == 0)
		op_clock_envelope(op, env_counter >> 2);

	// clock the phase
	op_clock_phase(chip, op, lfo_raw_pm);
	return;
}

// return the effective attenuation of the envelope
static UINT32 op_envelope_attenuation(const YMF262_STATE* chip, const FM_OPERATOR* op, UINT32 am_offset)
{
	UINT32 result = op->env_attenuation;

	// add in LFO AM modulation
	if (op_lfo_am_enable(op->opoffs))
		result += am_offset;

	// add in total level and KSL from the cache
	result += op->cache.total_level;

	// clamp to max and return
	return (result < 0x3ff) ? result : 0x3ff;
}

// compute the 14-bit signed volume of this operator, given a phase
// modulation and an AM LFO offset
static INT32 op_compute_volume(const YMF262_STATE* chip, const FM_OPERATOR* op, UINT32 phase, UINT32 am_offset)
{
	UINT32 sin_attenuation;
	UINT32 env_attenuation;
	INT32 result;

	// early out if the envelope is effectively off
	if (op->env_attenuation > EG_QUIET)
		return 0;

	// get the absolute value of the sin, as attenuation, as a 4.8 fixed point value
	sin_attenuation = op->cache.waveform[phase & (WAVEFORM_LENGTH - 1)];

	// get the attenuation from the evelope generator as a 4.6 value, shifted up to 4.8
	env_attenuation = op_envelope_attenuation(chip, op, am_offset) << 2;

	// combine into a 5.8 value, then convert from attenuation to 13-bit linear volume
	result = (INT32)attenuation_to_volume((sin_attenuation & 0x7fff) + env_attenuation);

	// negate if in the negative part of the sin wave (sign bit gives 14 bits)
	return bitfield(sin_attenuation, 15, 1) ? -result : result;
}

// return the current phase value
#define op_phase(op)	((op)->phase >> 10)


//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

// assign an operator to a slot of the channel; OP_NONE leaves it empty
static void channel_assign(YMF262_STATE* chip, FM_CHANNEL* ch, UINT8 index, UINT8 opnum)
{
	ch->op[index] = opnum;
	if (opnum != OP_NONE)
		chip->op[opnum].choffs = ch->choffs;
	return;
}

// signal key on/off to the channel's operators
static void channel_keyonoff(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 states, UINT8 type)
{
	UINT8 opnum;

	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (ch->op[opnum] != OP_NONE)
			op_keyonoff(&chip->op[ch->op[opnum]], bitfield(states, opnum, 1), type);
	}
	return;
}

// prepare for clocking; returns non-zero if any operator is active
static UINT8 channel_prepare(YMF262_STATE* chip, FM_CHANNEL* ch)
{
	UINT32 active_mask = 0;
	UINT8 opnum;

	// prepare all operators and determine if they are active
	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (ch->op[opnum] != OP_NONE && op_prepare(chip, &chip->op[ch->op[opnum]]))
			active_mask |= 1 << opnum;
	}
	return (active_mask != 0);
}

// master clock of all operators
static void channel_clock(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 env_counter, INT32 lfo_raw_pm)
{
	UINT8 opnum;

	// clock the feedback through
	ch->feedback[0] = ch->feedback[1];
	ch->feedback[1] = ch->feedback_in;

	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (ch->op[opnum] != OP_NONE)
			op_clock(chip, &chip->op[ch->op[opnum]], env_counter, lfo_raw_pm);
	}
	return;
}

// add a channel's value to the outputs it is enabled on
static void channel_add_to_output(const YMF262_STATE* chip, UINT32 choffs, INT32* output, INT32 value)
{
	if (ch_output_0(choffs))
		output[0] += value;
	if (ch_output_1(choffs))
		output[1] += value;
	if (ch_output_2(choffs))
		output[2] += value;
	if (ch_output_3(choffs))
		output[3] += value;
	return;
}

// combine 2 operators according to the specified algorithm, returning a sum
// according to the rshift and clipmax parameters
static INT32 channel_output_2op(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = &chip->op[ch->op[0]];
	const FM_OPERATOR* op2 = &chip->op[ch->op[1]];
	UINT32 am_offset;
	INT32 opmod;
	UINT32 feedback;
	INT32 op1value;
	INT32 result;

	// AM amount is the same across all operators
	am_offset = chip->lfo_am;

	// operator 1 has optional self-feedback
	opmod = 0;
	feedback = ch_feedback(ch->choffs);
	if (feedback != 0)
		opmod = (ch->feedback[0] + ch->feedback[1]) >> (10 - feedback);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	op1value = ch->feedback_in = (INT16)op_compute_volume(chip, op1, op_phase(op1) + opmod, am_offset);

	// now that the feedback has been computed, skip the rest if all volumes
	// are clear; no need to do all this work for nothing
	if (ch_output_any(ch->choffs) == 0)
		return 0;

	// Algorithms for two-operator case:
	//    0: O1 -> O2 -> out
	//    1: (O1 + O2) -> out
	// OPL3 uses the current sample of operator 1 for modulation
	if (bitfield(ch_algorithm(ch->choffs), 0, 1) == 0)
	{
		opmod = op1value >> 1;
		result = op_compute_volume(chip, op2, op_phase(op2) + opmod, am_offset) >> rshift;
	}
	else
	{
		result = op1value >> rshift;
		result += op_compute_volume(chip, op2, op_phase(op2), am_offset) >> rshift;
		result = clamp(result, -clipmax - 1, clipmax);
	}
	return result;
}

// combine 4 operators according to the specified algorithm, returning a sum
// according to the rshift and clipmax parameters
static INT32 channel_output_4op(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 rshift, INT32 clipmax)
{
	// OPL3 4-operator algorithms are 8-11 of ymfm's table; the operators are
	// computed in order, with the inputs pulled from an array of values
	// (opout) that is populated as we go:
	//    0 = 0
	//    1 = O1
	//    2 = O2
	//    3 = O3
	//    4 = (O4)
	//    5 = O1+O2
	//    6 = O1+O3
	//    7 = O2+O3
	//
	// The s_algorithm_ops table describes the inputs and outputs of each
	// algorithm as follows:
	//
	//      ---------x use opout[x] as operator 2 input
	//      ------xxx- use opout[x] as operator 3 input
	//      ---xxx---- use opout[x] as operator 4 input
	//      --x------- include opout[1] in final sum
	//      -x-------- include opout[2] in final sum
	//      x--------- include opout[3] in final sum
#define ALGORITHM(op2in, op3in, op4in, op1out, op2out, op3out) \
	((op2in) | ((op3in) << 1) | ((op4in) << 4) | ((op1out) << 7) | ((op2out) << 8) | ((op3out) << 9))
	static const UINT16 s_algorithm_ops[4] =
	{
		ALGORITHM(1,2,3, 0,0,0),	//  8: O1 -> O2 -> O3 -> O4 -> out (O4)
		ALGORITHM(0,2,3, 1,0,0),	//  9: (O1 + (O2 -> O3 -> O4)) -> out (O1+O4)
		ALGORITHM(1,0,3, 0,1,0),	// 10: ((O1 -> O2) + (O3 -> O4)) -> out (O2+O4)
		ALGORITHM(0,2,0, 1,0,1)		// 11: (O1 + (O2 -> O3) + O4) -> out (O1+O3+O4)
	};
#undef ALGORITHM
	UINT32 am_offset;
	INT32 opmod;
	UINT32 feedback;
	UINT32 algorithm_ops;
	INT16 opout[8];
	INT32 result;
	INT32 clipmin;

	// AM amount is the same across all operators
	am_offset = chip->lfo_am;

	// operator 1 has optional self-feedback
	opmod = 0;
	feedback = ch_feedback(ch->choffs);
	if (feedback != 0)
		opmod = (ch->feedback[0] + ch->feedback[1]) >> (10 - feedback);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	opout[1] = ch->feedback_in = (INT16)op_compute_volume(chip, &chip->op[ch->op[0]], op_phase(&chip->op[ch->op[0]]) + opmod, am_offset);

	// now that the feedback has been computed, skip the rest if all volumes
	// are clear; no need to do all this work for nothing
	if (ch_output_any(ch->choffs) == 0)
		return 0;

	algorithm_ops = s_algorithm_ops[ch_algorithm(ch->choffs) - 8];

	// populate the opout table
	opout[0] = 0;

	// compute the 14-bit volume/value of operator 2
	opmod = opout[bitfield(algorithm_ops, 0, 1)] >> 1;
	opout[2] = (INT16)op_compute_volume(chip, &chip->op[ch->op[1]], op_phase(&chip->op[ch->op[1]]) + opmod, am_offset);
	opout[5] = opout[1] + opout[2];

	// compute the 14-bit volume/value of operator 3
	opmod = opout[bitfield(algorithm_ops, 1, 3)] >> 1;
	opout[3] = (INT16)op_compute_volume(chip, &chip->op[ch->op[2]], op_phase(&chip->op[ch->op[2]]) + opmod, am_offset);
	opout[6] = opout[1] + opout[3];
	opout[7] = opout[2] + opout[3];

	// compute the 14-bit volume/value of operator 4; all algorithms
	// consume OP4 output at a minimum
	opmod = opout[bitfield(algorithm_ops, 4, 3)] >> 1;
	result = op_compute_volume(chip, &chip->op[ch->op[3]], op_phase(&chip->op[ch->op[3]]) + opmod, am_offset);
	result >>= rshift;

	// optionally add OP1, OP2, OP3
	clipmin = -clipmax - 1;
	if (bitfield(algorithm_ops, 7, 1) != 0)
		result = clamp(result + (opout[1] >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 8, 1) != 0)
		result = clamp(result + (opout[2] >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 9, 1) != 0)
		result = clamp(result + (opout[3] >> rshift), clipmin, clipmax);
	return result;
}

// channel 6 in rhythm mode: Bass Drum
static INT32 channel_output_rhythm_ch6(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = &chip->op[ch->op[0]];
	const FM_OPERATOR* op2 = &chip->op[ch->op[1]];
	UINT32 am_offset;
	INT32 opmod;
	UINT32 feedback;
	INT32 opout1;
	INT32 result;

	// AM amount is the same across all operators
	am_offset = chip->lfo_am;

	// Bass Drum: this uses operators 12 and 15 (i.e., channel 6)
	// in an almost-normal way, except that if the algorithm is 1,
	// the first operator is ignored instead of added in

	// operator 1 has optional self-feedback
	opmod = 0;
	feedback = ch_feedback(ch->choffs);
	if (feedback != 0)
		opmod = (ch->feedback[0] + ch->feedback[1]) >> (10 - feedback);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	opout1 = ch->feedback_in = (INT16)op_compute_volume(chip, op1, op_phase(op1) + opmod, am_offset);

	// compute the 14-bit volume/value of operator 2, which is the result
	opmod = bitfield(ch_algorithm(ch->choffs), 0, 1) ? 0 : (opout1 >> 1);
	result = op_compute_volume(chip, op2, op_phase(op2) + opmod, am_offset) >> rshift;
	if ((chip->muteMask >> MUTE_BD) & 0x01)
		result = 0;
	return result * 2;
}

// channel 7 in rhythm mode: High Hat and Snare Drum
static INT32 channel_output_rhythm_ch7(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 phase_select, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = &chip->op[ch->op[0]];
	const FM_OPERATOR* op2 = &chip->op[ch->op[1]];
	UINT32 am_offset;
	UINT32 noise;
	UINT32 phase;
	INT32 hihat;
	INT32 snare;

	// AM amount is the same across all operators
	am_offset = chip->lfo_am;
	noise = bitfield(noise_state(), 0, 1);

	// High Hat: this uses the envelope from operator 13 (channel 7),
	// and a combination of noise and the operator 13/17 phase select
	// to compute the phase
	phase = (phase_select << 9) | (0xd0 >> (2 * (noise ^ phase_select)));
	hihat = op_compute_volume(chip, op1, phase, am_offset) >> rshift;
	if ((chip->muteMask >> MUTE_HH) & 0x01)
		hihat = 0;

	// Snare Drum: this uses the envelope from operator 16 (channel 7),
	// and a combination of noise and operator 13 phase to pick a phase
	phase = (0x100 << bitfield(op_phase(op1), 8, 1)) ^ (noise << 8);
	snare = op_compute_volume(chip, op2, phase, am_offset) >> rshift;
	if ((chip->muteMask >> MUTE_SD) & 0x01)
		snare = 0;

	return clamp(hihat + snare, -clipmax - 1, clipmax) * 2;
}

// channel 8 in rhythm mode: Tom Tom and Top Cymbal
static INT32 channel_output_rhythm_ch8(YMF262_STATE* chip, FM_CHANNEL* ch, UINT32 phase_select, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = &chip->op[ch->op[0]];
	const FM_OPERATOR* op2 = &chip->op[ch->op[1]];
	UINT32 am_offset;
	UINT32 phase;
	INT32 tomtom;
	INT32 cymbal;

	// AM amount is the same across all operators
	am_offset = chip->lfo_am;

	// Tom Tom: this is just a single operator processed normally
	tomtom = op_compute_volume(chip, op1, op_phase(op1), am_offset) >> rshift;
	if ((chip->muteMask >> MUTE_TT) & 0x01)
		tomtom = 0;

	// Top Cymbal: this uses the envelope from operator 17 (channel 8),
	// and the operator 13/17 phase select to compute the phase
	phase = 0x100 | (phase_select << 9);
	cymbal = op_compute_volume(chip, op2, phase, am_offset) >> rshift;
	if ((chip->muteMask >> MUTE_CY) & 0x01)
		cymbal = 0;

	return clamp(tomtom + cymbal, -clipmax - 1, clipmax) * 2;
}


//----------------------------------------------------------------------------
// FM engine
//----------------------------------------------------------------------------

// update the IRQ status bit
static void engine_check_interrupts(YMF262_STATE* chip)
{
	if ((chip->status & IRQ_MASK & ~status_mask()) != 0)
		chip->status |= STATUS_IRQ;
	else
		chip->status &= ~STATUS_IRQ;
	return;
}

// set/reset bits in the status register, updating the IRQ status
static void engine_set_reset_status(YMF262_STATE* chip, UINT8 set, UINT8 reset)
{
	chip->status = (chip->status | set) & ~reset;
	engine_check_interrupts(chip);
	return;
}

// return the current status
#define engine_status()	((UINT8)(chip->status & ~status_mask()))

// update the state of the given timer
static void engine_update_timer(YMF262_STATE* chip, UINT32 tnum, UINT32 enable, INT32 delta_clocks)
{
	// if the timer is live, but not currently enabled, set the timer
	if (enable && ! chip->timer_running[tnum])
	{
		// period comes from the registers, and is different for each
		UINT32 period = (tnum == 0) ? (1024 - timer_a_value()) : 16 * (256 - timer_b_value());

		// caller can also specify a delta to account for other effects
		period += delta_clocks;

		// reset it; one period unit is one sample (OPERATORS * prescale clocks)
		chip->timer_count[tnum] = (INT32)period;
		chip->timer_running[tnum] = 1;
	}
	// if the timer is not live, ensure it is not enabled
	else if (! enable)
	{
		chip->timer_count[tnum] = 0;
		chip->timer_running[tnum] = 0;
	}
	return;
}

// timer has expired - signal status
static void engine_timer_expired(YMF262_STATE* chip, UINT32 tnum)
{
	// update status; both timers are always enabled on OPL
	if (tnum == 0)
		engine_set_reset_status(chip, STATUS_TIMERA, 0);
	else
		engine_set_reset_status(chip, STATUS_TIMERB, 0);

	// reset
	chip->timer_running[tnum] = 0;
	engine_update_timer(chip, tnum, 1, 0);
	return;
}

// handle a mode register write
static void engine_mode_write(YMF262_STATE* chip, UINT8 data)
{
	UINT32 dummy1;
	UINT32 dummy2;
	UINT8 reset_mask;

	// mark all channels as modified
	chip->modified_channels = ALL_CHANNELS;

	// actually write the mode register now
	regs_write(chip, REG_MODE, data, &dummy1, &dummy2);

	// reset IRQ status -- when written, all other bits are ignored
	if (irq_reset())
	{
		engine_set_reset_status(chip, 0, 0x78);
	}
	else
	{
		// reset timer status
		reset_mask = 0;
		if (reset_timer_b())
			reset_mask |= STATUS_TIMERB;
		if (reset_timer_a())
			reset_mask |= STATUS_TIMERA;
		engine_set_reset_status(chip, 0, reset_mask);

		// load timers; note that timer B gets a small negative adjustment because
		// the *16 multiplier is free-running, so the first tick of the clock
		// is a bit shorter
		engine_update_timer(chip, 1, load_timer_b(), -(INT32)(chip->total_clocks & 15));
		engine_update_timer(chip, 0, load_timer_a(), 0);
	}
	return;
}

// handle writes to the OPL registers
static void engine_write(YMF262_STATE* chip, UINT16 regnum, UINT8 data)
{
	UINT32 keyon_channel;
	UINT32 keyon_opmask;

	// special case: writes to the mode register can impact IRQs
	if (regnum == REG_MODE)
	{
		engine_mode_write(chip, data);
		return;
	}

	// for now just mark all channels as modified
	chip->modified_channels = ALL_CHANNELS;

	// most writes are passive, consumed only when needed
	if (regs_write(chip, regnum, data, &keyon_channel, &keyon_opmask))
	{
		// handle writes to the keyon register(s)
		if (keyon_channel < CHANNELS)
		{
			// normal channel on/off
			channel_keyonoff(chip, &chip->channel[keyon_channel], keyon_opmask, KEYON_NORMAL);
		}
		else if (keyon_channel == RHYTHM_CHANNEL)
		{
			// special case for the OPL rhythm channels
			channel_keyonoff(chip, &chip->channel[6], bitfield(keyon_opmask, 4, 1) ? 3 : 0, KEYON_RHYTHM);
			channel_keyonoff(chip, &chip->channel[7], bitfield(keyon_opmask, 0, 1) | (bitfield(keyon_opmask, 3, 1) << 1), KEYON_RHYTHM);
			channel_keyonoff(chip, &chip->channel[8], bitfield(keyon_opmask, 2, 1) | (bitfield(keyon_opmask, 1, 1) << 1), KEYON_RHYTHM);
		}
	}
	return;
}

// map the operators to the channels from the 4-operator enables (reg 0x104):
// enable bit n (n = 0-2 in bank 0, 3-5 in bank 1) gives channel n of the bank
// the operators of channel n + 3, which then has none
static void engine_assign_operators(YMF262_STATE* chip)
{
	// 2-operator pairs of each channel in one bank: carrier/modulator pairs
	// 3 apart, in groups of 6
	static const UINT8 s_channel_ops[9][2] =
	{
		{  0,  3 }, {  1,  4 }, {  2,  5 },
		{  6,  9 }, {  7, 10 }, {  8, 11 },
		{ 12, 15 }, { 13, 16 }, { 14, 17 }
	};
	UINT32 fourop;
	UINT8 chnum;
	UINT8 base;
	UINT8 opbase;
	UINT8 index;
	UINT8 ops[4];
	FM_CHANNEL* ch;

	fourop = fourop_enable();
	for (chnum = 0; chnum < CHANNELS; chnum ++)
	{
		ch = &chip->channel[chnum];
		base = chnum % 9;
		opbase = 18 * (chnum / 9);
		ops[0] = ops[1] = ops[2] = ops[3] = OP_NONE;
		if (base < 3 && bitfield(fourop, base + 3 * (chnum / 9), 1))
		{
			ops[0] = opbase + base;
			ops[1] = opbase + base + 3;
			ops[2] = opbase + base + 6;
			ops[3] = opbase + base + 9;
		}
		else if (base >= 3 && base < 6 && bitfield(fourop, base - 3 + 3 * (chnum / 9), 1))
		{
			// operators belong to the 4-operator channel base - 3
		}
		else
		{
			ops[0] = opbase + s_channel_ops[base][0];
			ops[1] = opbase + s_channel_ops[base][1];
		}
		for (index = 0; index < 4; index ++)
			channel_assign(chip, ch, index, ops[index]);
	}
	return;
}

// iterate over all channels, clocking them forward one step
static void engine_clock(YMF262_STATE* chip)
{
	UINT8 chnum;
	INT32 lfo_raw_pm;

	// update the clock counter
	chip->total_clocks ++;

	// if something was modified, prepare
	// also prepare every 4k samples to catch ending notes
	if (chip->modified_channels != 0 || chip->prepare_count++ >= 4096)
	{
		// reassign operators to channels
		engine_assign_operators(chip);

		// call each channel to prepare
		chip->active_channels = 0;
		for (chnum = 0; chnum < CHANNELS; chnum ++)
		{
			if (channel_prepare(chip, &chip->channel[chnum]))
				chip->active_channels |= 1 << chnum;
		}

		// reset the modified channels and prepare count
		chip->modified_channels = chip->prepare_count = 0;
	}

	// the envelope clock divider is 1, so just increment by 4
	chip->env_counter += 4;

	// clock the noise generator and LFO
	lfo_raw_pm = clock_noise_and_lfo(chip);

	// now update the state of all the channels and operators
	for (chnum = 0; chnum < CHANNELS; chnum ++)
		channel_clock(chip, &chip->channel[chnum], chip->env_counter, lfo_raw_pm);
	return;
}

// compute a sum over all active channels into the 4 outputs; muted channels
// are computed (so their feedback state stays live) but not added
static void engine_output(YMF262_STATE* chip, INT32* output, UINT32 rshift, INT32 clipmax)
{
	UINT32 chanmask;
	UINT32 phase_select;
	UINT32 op13phase;
	UINT32 op17phase;
	INT32 result;
	UINT8 rhythm;
	UINT8 chnum;
	FM_CHANNEL* ch;

	// mask out inactive channels
	chanmask = ALL_CHANNELS & chip->active_channels;

	// handle the rhythm case, where some of the operators are dedicated
	// to percussion
	rhythm = (UINT8)rhythm_enable();
	phase_select = 0;
	if (rhythm)
	{
		// precompute the operator 13+17 phase selection value
		op13phase = op_phase(&chip->op[13]);
		op17phase = op_phase(&chip->op[17]);
		phase_select = (bitfield(op13phase, 2, 1) ^ bitfield(op13phase, 7, 1)) | bitfield(op13phase, 3, 1) |
			(bitfield(op17phase, 5, 1) ^ bitfield(op17phase, 3, 1));
	}

	// sum over all the desired channels
	for (chnum = 0; chnum < CHANNELS; chnum ++)
	{
		if (! bitfield(chanmask, chnum, 1))
			continue;
		ch = &chip->channel[chnum];
		if (rhythm && chnum == 6)
		{
			result = channel_output_rhythm_ch6(chip, ch, rshift, clipmax);
		}
		else if (rhythm && chnum == 7)
		{
			result = channel_output_rhythm_ch7(chip, ch, phase_select, rshift, clipmax);
		}
		else if (rhythm && chnum == 8)
		{
			result = channel_output_rhythm_ch8(chip, ch, phase_select, rshift, clipmax);
		}
		else
		{
			if (ch->op[2] != OP_NONE)
				result = channel_output_4op(chip, ch, rshift, clipmax);
			else
				result = channel_output_2op(chip, ch, rshift, clipmax);
			if ((chip->muteMask >> chnum) & 0x01)
				result = 0;
		}
		channel_add_to_output(chip, ch->choffs, output, result);
	}
	return;
}

static void engine_reset(YMF262_STATE* chip)
{
	UINT8 curChn;
	UINT8 curOp;

	// reset all status bits
	engine_set_reset_status(chip, 0, 0xff);

	// register type-specific initialization
	regs_reset(chip);

	// explicitly write to the mode register since it has side-effects
	engine_write(chip, REG_MODE, 0);

	// reset the channels
	for (curChn = 0; curChn < CHANNELS; curChn ++)
	{
		chip->channel[curChn].feedback[0] = chip->channel[curChn].feedback[1] = 0;
		chip->channel[curChn].feedback_in = 0;
	}

	// reset the operators
	for (curOp = 0; curOp < OPERATORS; curOp ++)
		op_reset(&chip->op[curOp]);
	return;
}

static void engine_init(YMF262_STATE* chip)
{
	UINT8 curChn;
	UINT8 curOp;
	UINT32 opnum;

	regs_init(chip);

	chip->active_channels = ALL_CHANNELS;
	chip->modified_channels = ALL_CHANNELS;

	// create the channels; bank 1 channels are at register offset 0x100
	for (curChn = 0; curChn < CHANNELS; curChn ++)
		chip->channel[curChn].choffs = (curChn % 9) + 0x100 * (curChn / 9);

	// create the operators
	for (curOp = 0; curOp < OPERATORS; curOp ++)
	{
		opnum = curOp % 18;
		chip->op[curOp].opoffs = opnum + 2 * (opnum / 6) + 0x100 * (curOp / 18);
		op_reset(&chip->op[curOp]);
	}

	// do the initial operator assignment
	engine_assign_operators(chip);
	return;
}


//----------------------------------------------------------------------------
// YMF262
//----------------------------------------------------------------------------

// generate one sample of sound
static void ymf262_generate(YMF262_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[OUTPUTS];
	UINT8 tnum;
	UINT8 outnum;

	// expire timers that run out on this sample
	for (tnum = 0; tnum < 2; tnum ++)
	{
		if (chip->timer_count[tnum] > 0 && --chip->timer_count[tnum] == 0)
			engine_timer_expired(chip, tnum);
	}

	// clock the system
	engine_clock(chip);

	// update the FM content; mixing details need verification
	for (outnum = 0; outnum < OUTPUTS; outnum ++)
		output[outnum] = 0;
	engine_output(chip, output, 0, 32767);

	// YMF262 output is 16-bit offset serial via YAC512 DAC
	for (outnum = 0; outnum < OUTPUTS; outnum ++)
		output[outnum] = clamp(output[outnum], -32768, 32767);

	*outL = output[0] + output[2];
	*outR = output[1] + output[3];
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

static UINT8 device_start_ymf262(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YMF262_STATE* chip;

	chip = (YMF262_STATE*)calloc(1, sizeof(YMF262_STATE));
	if (chip == NULL)
		return 0xFF;

	engine_init(chip);
	engine_reset(chip);
	chip->muteMask = 0x00;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / 288, &devDef262);
	return 0x00;
}

static void device_stop_ymf262(void* info)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;

	free(chip);
	return;
}

static void device_reset_ymf262(void* info)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;

	// ymfm's reset leaves the address register alone; a device reset clears it too
	chip->address = 0;
	engine_reset(chip);
	return;
}

static void ymf262_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ymf262_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ymf262_w(void* info, UINT8 offset, UINT8 data)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;

	switch (offset & 3)
	{
	case 0:	// address port, bank 0
		chip->address = data;
		break;
	case 2:	// address port, bank 1
		chip->address = data | 0x100;

		// tests reveal that in compatibility mode, upper bit is masked
		// except for register 0x105
		if (newflag() == 0 && chip->address != 0x105)
			chip->address &= 0xff;
		break;
	case 1:	// data ports
	case 3:
		engine_write(chip, chip->address, data);
		break;
	}
	return;
}

static UINT8 ymf262_r(void* info, UINT8 offset)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;

	// status port; the other offsets are not readable
	if ((offset & 3) != 0)
		return 0xFF;
	return engine_status();
}

static void ymf262_set_mute_mask(void* info, UINT32 MuteMask)
{
	YMF262_STATE* chip = (YMF262_STATE*)info;

	chip->muteMask = MuteMask;
	return;
}
