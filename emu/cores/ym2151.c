// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Yamaha YM2151 (OPM)

    C port of the YM2151 path of ymfm (ymfm_opm.cpp, ymfm_fm.ipp, ymfm.h).
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
    - Only the OPM (YM2151) parts of the ymfm FM engine are kept: no OPN
      SSG-EG, OPL rhythm, OPLL depress or OPZ reverb states.
    - ymfm leaves timers to its host. Here they count in chip samples
      (clock / 64) and expire before the sample is clocked, so Timer A
      also drives CSM key-on.
    - One update sample is one chip sample (clock / 64).
    - The busy flag is not emulated. The CT1/CT2 output lines (register
      0x1B bits 6-7) are stored but drive nothing.
    - The YM2164 (OPP) variant is not included.
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

#include "ym2151.h"


static void ym2151_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym2151(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ym2151(void* info);
static void device_reset_ym2151(void* info);
static void ym2151_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2151_r(void* info, UINT8 offset);
static void ym2151_set_mute_mask(void* info, UINT32 MuteMask);


static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2151_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2151_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ym2151_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"YM2151", "ymfm", FCC_YMFM,

	device_start_ym2151,
	device_stop_ym2151,
	device_reset_ym2151,
	ym2151_update,

	NULL,	// SetOptionBits
	ym2151_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "YM2151";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 8;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YM2151 =
{
	DEVID_YM2151,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};


#define CHANNELS		8
#define OPERATORS		(CHANNELS * 4)
#define ALL_CHANNELS	((1 << CHANNELS) - 1)
#define REGISTERS		0x100
#define WAVEFORM_LENGTH	0x400
#define LFO_WAVEFORM_LENGTH	0x100
#define REG_MODE		0x14
#define CSM_TRIGGER_MASK	ALL_CHANNELS

#define STATUS_TIMERA	0x01
#define STATUS_TIMERB	0x02
#define STATUS_BUSY		0x80

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
#define KEYON_CSM		2

// set phase_step to this value to recalculate it each sample; needed
// in the case of PM LFO changes
#define PHASE_STEP_DYNAMIC	1

// data computed once at the start of clocking that remains static during
// subsequent sound generation
typedef struct _opdata_cache
{
	UINT32 phase_step;		// phase step, or PHASE_STEP_DYNAMIC if PM is active
	UINT32 total_level;		// total level * 8
	UINT32 block_freq;		// raw block frequency value (used to compute phase_step)
	INT32 detune;			// detuning value (used to compute phase_step)
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
	UINT8 keyon_live;		// live key on state (bit 0 = direct, bit 2 = CSM)
	UINT8 keyoff_pending;	// the live key went off since the last prepare
	OPDATA_CACHE cache;		// cached values for performance
} FM_OPERATOR;

typedef struct _fm_channel
{
	UINT32 choffs;			// channel offset in registers
	INT16 feedback[2];		// feedback memory for operator 1
	INT16 feedback_in;		// next input value for op 1 feedback (set in output)
	UINT8 op[4];			// operator indices
} FM_CHANNEL;

typedef struct _ym2151_state
{
	DEV_DATA _devData;

	// OPM registers
	UINT32 lfo_counter;		// LFO counter
	UINT32 noise_lfsr;		// noise LFSR state
	UINT8 noise_counter;	// noise counter
	UINT8 noise_state;		// latched noise state
	UINT8 noise_lfo;		// latched LFO noise value
	UINT8 lfo_am;			// current LFO AM value
	UINT8 regdata[REGISTERS];	// register data
	INT16 lfo_waveform[4][LFO_WAVEFORM_LENGTH];	// LFO waveforms; AM in low 8, PM in upper 8
	UINT16 waveform[WAVEFORM_LENGTH];	// sine waveform

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

	// YM2151
	UINT8 address;			// address register

	UINT32 muteMask;
} YM2151_STATE;


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

// 6-bit phase displacement for a 5-bit key code and a 2-bit detune value
static const UINT8 s_detune_adjustment[32][4] =
{
	{ 0,  0,  1,  2 },  { 0,  0,  1,  2 },  { 0,  0,  1,  2 },  { 0,  0,  1,  2 },
	{ 0,  1,  2,  2 },  { 0,  1,  2,  3 },  { 0,  1,  2,  3 },  { 0,  1,  2,  3 },
	{ 0,  1,  2,  4 },  { 0,  1,  3,  4 },  { 0,  1,  3,  4 },  { 0,  1,  3,  5 },
	{ 0,  2,  4,  5 },  { 0,  2,  4,  6 },  { 0,  2,  4,  6 },  { 0,  2,  5,  7 },
	{ 0,  2,  5,  8 },  { 0,  3,  6,  8 },  { 0,  3,  6,  9 },  { 0,  3,  7, 10 },
	{ 0,  4,  8, 11 },  { 0,  4,  8, 12 },  { 0,  4,  9, 13 },  { 0,  5, 10, 14 },
	{ 0,  5, 11, 16 },  { 0,  6, 12, 17 },  { 0,  6, 13, 19 },  { 0,  7, 14, 20 },
	{ 0,  8, 16, 22 },  { 0,  8, 16, 22 },  { 0,  8, 16, 22 },  { 0,  8, 16, 22 }
};

// coarse detune (DT2) delta; uses the cents values from the manual,
// converted into 1/64ths
static const INT16 s_detune2_delta[4] =
{
	0, (600*64+50)/100, (781*64+50)/100, (950*64+50)/100
};

// The phase step is essentially the fnum in OPN-speak. To compute this table,
// we used the standard formula for computing the frequency of a note, and
// then converted that frequency to fnum using the formula documented in the
// YM2608 manual.
//
// However, the YM2608 manual describes everything in terms of a nominal 8MHz
// clock, which produces an FM clock of:
//
//    8000000 / 24(operators) / 6(prescale) = 55555Hz FM clock
//
// Whereas the descriptions for the YM2151 use a nominal 3.579545MHz clock:
//
//    3579545 / 32(operators) / 2(prescale) = 55930Hz FM clock
//
// To correct for this, the YM2608 formula was adjusted to use a clock of
// 8053920Hz, giving this equation for the fnum:
//
//    fnum = (double(144) * freq * (1 << 20)) / double(8053920) / 4;
//
// Unfortunately, the computed table differs in a few spots from the data
// verified from an actual chip. The table below comes from David Viens'
// analysis, used with his permission.
static const UINT32 s_phase_step[12*64] =
{
	41568,41600,41632,41664,41696,41728,41760,41792,41856,41888,41920,41952,42016,42048,42080,42112,
	42176,42208,42240,42272,42304,42336,42368,42400,42464,42496,42528,42560,42624,42656,42688,42720,
	42784,42816,42848,42880,42912,42944,42976,43008,43072,43104,43136,43168,43232,43264,43296,43328,
	43392,43424,43456,43488,43552,43584,43616,43648,43712,43744,43776,43808,43872,43904,43936,43968,
	44032,44064,44096,44128,44192,44224,44256,44288,44352,44384,44416,44448,44512,44544,44576,44608,
	44672,44704,44736,44768,44832,44864,44896,44928,44992,45024,45056,45088,45152,45184,45216,45248,
	45312,45344,45376,45408,45472,45504,45536,45568,45632,45664,45728,45760,45792,45824,45888,45920,
	45984,46016,46048,46080,46144,46176,46208,46240,46304,46336,46368,46400,46464,46496,46528,46560,
	46656,46688,46720,46752,46816,46848,46880,46912,46976,47008,47072,47104,47136,47168,47232,47264,
	47328,47360,47392,47424,47488,47520,47552,47584,47648,47680,47744,47776,47808,47840,47904,47936,
	48032,48064,48096,48128,48192,48224,48288,48320,48384,48416,48448,48480,48544,48576,48640,48672,
	48736,48768,48800,48832,48896,48928,48992,49024,49088,49120,49152,49184,49248,49280,49344,49376,
	49440,49472,49504,49536,49600,49632,49696,49728,49792,49824,49856,49888,49952,49984,50048,50080,
	50144,50176,50208,50240,50304,50336,50400,50432,50496,50528,50560,50592,50656,50688,50752,50784,
	50880,50912,50944,50976,51040,51072,51136,51168,51232,51264,51328,51360,51424,51456,51488,51520,
	51616,51648,51680,51712,51776,51808,51872,51904,51968,52000,52064,52096,52160,52192,52224,52256,
	52384,52416,52448,52480,52544,52576,52640,52672,52736,52768,52832,52864,52928,52960,52992,53024,
	53120,53152,53216,53248,53312,53344,53408,53440,53504,53536,53600,53632,53696,53728,53792,53824,
	53920,53952,54016,54048,54112,54144,54208,54240,54304,54336,54400,54432,54496,54528,54592,54624,
	54688,54720,54784,54816,54880,54912,54976,55008,55072,55104,55168,55200,55264,55296,55360,55392,
	55488,55520,55584,55616,55680,55712,55776,55808,55872,55936,55968,56032,56064,56128,56160,56224,
	56288,56320,56384,56416,56480,56512,56576,56608,56672,56736,56768,56832,56864,56928,56960,57024,
	57120,57152,57216,57248,57312,57376,57408,57472,57536,57568,57632,57664,57728,57792,57824,57888,
	57952,57984,58048,58080,58144,58208,58240,58304,58368,58400,58464,58496,58560,58624,58656,58720,
	58784,58816,58880,58912,58976,59040,59072,59136,59200,59232,59296,59328,59392,59456,59488,59552,
	59648,59680,59744,59776,59840,59904,59936,60000,60064,60128,60160,60224,60288,60320,60384,60416,
	60512,60544,60608,60640,60704,60768,60800,60864,60928,60992,61024,61088,61152,61184,61248,61280,
	61376,61408,61472,61536,61600,61632,61696,61760,61824,61856,61920,61984,62048,62080,62144,62208,
	62272,62304,62368,62432,62496,62528,62592,62656,62720,62752,62816,62880,62944,62976,63040,63104,
	63200,63232,63296,63360,63424,63456,63520,63584,63648,63680,63744,63808,63872,63904,63968,64032,
	64096,64128,64192,64256,64320,64352,64416,64480,64544,64608,64672,64704,64768,64832,64896,64928,
	65024,65056,65120,65184,65248,65312,65376,65408,65504,65536,65600,65664,65728,65792,65856,65888,
	65984,66016,66080,66144,66208,66272,66336,66368,66464,66496,66560,66624,66688,66752,66816,66848,
	66944,66976,67040,67104,67168,67232,67296,67328,67424,67456,67520,67584,67648,67712,67776,67808,
	67904,67936,68000,68064,68128,68192,68256,68288,68384,68448,68512,68544,68640,68672,68736,68800,
	68896,68928,68992,69056,69120,69184,69248,69280,69376,69440,69504,69536,69632,69664,69728,69792,
	69920,69952,70016,70080,70144,70208,70272,70304,70400,70464,70528,70560,70656,70688,70752,70816,
	70912,70976,71040,71104,71136,71232,71264,71360,71424,71488,71552,71616,71648,71744,71776,71872,
	71968,72032,72096,72160,72192,72288,72320,72416,72480,72544,72608,72672,72704,72800,72832,72928,
	72992,73056,73120,73184,73216,73312,73344,73440,73504,73568,73632,73696,73728,73824,73856,73952,
	74080,74144,74208,74272,74304,74400,74432,74528,74592,74656,74720,74784,74816,74912,74944,75040,
	75136,75200,75264,75328,75360,75456,75488,75584,75648,75712,75776,75840,75872,75968,76000,76096,
	76224,76288,76352,76416,76448,76544,76576,76672,76736,76800,76864,76928,77024,77120,77152,77248,
	77344,77408,77472,77536,77568,77664,77696,77792,77856,77920,77984,78048,78144,78240,78272,78368,
	78464,78528,78592,78656,78688,78784,78816,78912,78976,79040,79104,79168,79264,79360,79392,79488,
	79616,79680,79744,79808,79840,79936,79968,80064,80128,80192,80256,80320,80416,80512,80544,80640,
	80768,80832,80896,80960,80992,81088,81120,81216,81280,81344,81408,81472,81568,81664,81696,81792,
	81952,82016,82080,82144,82176,82272,82304,82400,82464,82528,82592,82656,82752,82848,82880,82976,
	
};

// inputs and outputs of each 4-operator algorithm; the inputs come from opout:
//    0 = 0, 1 = O1, 2 = O2, 3 = O3, 4 = (O4), 5 = O1+O2, 6 = O1+O3, 7 = O2+O3
//      ---------x use opout[x] as operator 2 input
//      ------xxx- use opout[x] as operator 3 input
//      ---xxx---- use opout[x] as operator 4 input
//      --x------- include opout[1] in final sum
//      -x-------- include opout[2] in final sum
//      x--------- include opout[3] in final sum
#define ALGORITHM(op2in, op3in, op4in, op1out, op2out, op3out) \
	((op2in) | ((op3in) << 1) | ((op4in) << 4) | ((op1out) << 7) | ((op2out) << 8) | ((op3out) << 9))
static const UINT16 s_algorithm_ops[8] =
{
	ALGORITHM(1,2,3, 0,0,0),    //  0: O1 -> O2 -> O3 -> O4 -> out (O4)
	ALGORITHM(0,5,3, 0,0,0),    //  1: (O1 + O2) -> O3 -> O4 -> out (O4)
	ALGORITHM(0,2,6, 0,0,0),    //  2: (O1 + (O2 -> O3)) -> O4 -> out (O4)
	ALGORITHM(1,0,7, 0,0,0),    //  3: ((O1 -> O2) + O3) -> O4 -> out (O4)
	ALGORITHM(1,0,3, 0,1,0),    //  4: ((O1 -> O2) + (O3 -> O4)) -> out (O2+O4)
	ALGORITHM(1,1,1, 0,1,1),    //  5: ((O1 -> O2) + (O1 -> O3) + (O1 -> O4)) -> out (O2+O3+O4)
	ALGORITHM(1,0,0, 0,1,1),    //  6: ((O1 -> O2) + O3 + O4) -> out (O2+O3+O4)
	ALGORITHM(0,0,0, 1,1,1),    //  7: (O1 + O2 + O3 + O4) -> out (O1+O2+O3+O4)
};
#undef ALGORITHM


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

// return the number of leading zeros in a 32-bit value
static UINT8 count_leading_zeros(UINT32 value)
{
	UINT8 count;

	if (value == 0)
		return 32;
	for (count = 0; ! (value & 0x80000000); count ++)
		value <<= 1;
	return count;
}

// compute the result of a round trip through the 3.10 floating-point format
// sent to the DAC: values are clamped to 16 bits and bits below the mantissa
// are zeroed
static INT32 roundtrip_fp(INT32 value)
{
	INT32 scanvalue;
	int exponent;
	INT32 mask;

	// handle overflows first
	if (value < -32768)
		return -32768;
	if (value > 32767)
		return 32767;

	// we need to count the number of leading sign bits after the sign
	// we can use count_leading_zeros if we invert negative values
	scanvalue = value ^ (value >> 31);

	// exponent is related to the number of leading bits starting from bit 14
	exponent = 7 - count_leading_zeros((UINT32)scanvalue << 17);

	// smallest exponent value allowed is 1
	if (exponent < 1)
		exponent = 1;

	// apply the shift back and forth to zero out bits that are lost
	exponent -= 1;
	mask = (1 << exponent) - 1;
	return value & ~mask;
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

// given a 5-bit key code value and a 3-bit detune parameter, return a
// 6-bit signed phase displacement
static INT32 detune_adjustment(UINT32 detune, UINT32 keycode)
{
	INT32 result = s_detune_adjustment[keycode][detune & 3];
	return bitfield(detune, 2, 1) ? -result : result;
}

// convert an OPM concatenated block (3 bits), keycode (4 bits) and key
// fraction (6 bits) to a 0.10 phase step, after applying the given delta
static UINT32 opm_key_code_to_phase_step(UINT32 block_freq, INT32 delta)
{
	UINT32 block;
	UINT32 adjusted_code;
	INT32 eff_freq;

	// extract the block (octave) first
	block = bitfield(block_freq, 10, 3);

	// the keycode (bits 6-9) is "gappy", mapping 12 values over 16 in each
	// octave; to correct for this, we multiply the 4-bit value by 3/4 (or
	// rather subtract 1/4); note that a (invalid) value of 15 will bleed into
	// the next octave -- this is confirmed
	adjusted_code = bitfield(block_freq, 6, 4) - bitfield(block_freq, 8, 2);

	// now re-insert the 6-bit fraction
	eff_freq = (INT32)((adjusted_code << 6) | bitfield(block_freq, 0, 6));

	// now that the gaps are removed, add the delta
	eff_freq += delta;

	// handle over/underflow by adjusting the block:
	if ((UINT32)eff_freq >= 768)
	{
		// minimum delta is -512 (PM), so we can only underflow by 1 octave
		if (eff_freq < 0)
		{
			eff_freq += 768;
			if (block-- == 0)
				return s_phase_step[0] >> 7;
		}
		// maximum delta is +512+608 (PM+detune), so we can overflow by up to 2 octaves
		else
		{
			eff_freq -= 768;
			if (eff_freq >= 768)
			{
				block ++;
				eff_freq -= 768;
			}
			if (block++ >= 7)
				return s_phase_step[767];
		}
	}

	// look up the phase shift for the key code, then shift by octave
	return s_phase_step[eff_freq] >> (block ^ 7);
}

// apply KSR to the raw ADSR rate, ignoring ksr if the raw value is 0, and
// clamping to 63
INLINE UINT32 effective_rate(UINT32 rawrate, UINT32 ksr)
{
	if (rawrate == 0)
		return 0;
	return (rawrate + ksr < 63) ? (rawrate + ksr) : 63;
}


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

#define REG(offset)		(chip->regdata[offset])

// system-wide registers
#define lfo_reset()			bitfield(REG(0x01), 1, 1)
#define noise_frequency()	(bitfield(REG(0x0f), 0, 5) ^ 0x1f)
#define noise_enable()		bitfield(REG(0x0f), 7, 1)
#define timer_a_value()		((bitfield(REG(0x10), 0, 8) << 2) | bitfield(REG(0x11), 0, 2))
#define timer_b_value()		bitfield(REG(0x12), 0, 8)
#define csm()				bitfield(REG(0x14), 7, 1)
#define reset_timer_b()		bitfield(REG(0x14), 5, 1)
#define reset_timer_a()		bitfield(REG(0x14), 4, 1)
#define enable_timer_b()	bitfield(REG(0x14), 3, 1)
#define enable_timer_a()	bitfield(REG(0x14), 2, 1)
#define load_timer_b()		bitfield(REG(0x14), 1, 1)
#define load_timer_a()		bitfield(REG(0x14), 0, 1)
#define lfo_rate()			bitfield(REG(0x18), 0, 8)
#define lfo_am_depth()		bitfield(REG(0x19), 0, 7)
#define lfo_pm_depth()		bitfield(REG(0x1a), 0, 7)
#define lfo_waveform()		bitfield(REG(0x1b), 0, 2)

// per-channel registers
#define ch_output_any(choffs)	bitfield(REG(0x20 + (choffs)), 6, 2)
#define ch_output_0(choffs)		bitfield(REG(0x20 + (choffs)), 6, 1)
#define ch_output_1(choffs)		bitfield(REG(0x20 + (choffs)), 7, 1)
#define ch_feedback(choffs)		bitfield(REG(0x20 + (choffs)), 3, 3)
#define ch_algorithm(choffs)	bitfield(REG(0x20 + (choffs)), 0, 3)
#define ch_block_freq(choffs)	((bitfield(REG(0x28 + (choffs)), 0, 7) << 6) | bitfield(REG(0x30 + (choffs)), 2, 6))
#define ch_lfo_pm_sens(choffs)	bitfield(REG(0x38 + (choffs)), 4, 3)
#define ch_lfo_am_sens(choffs)	bitfield(REG(0x38 + (choffs)), 0, 2)

// per-operator registers
#define op_detune(opoffs)		bitfield(REG(0x40 + (opoffs)), 4, 3)
#define op_multiple(opoffs)		bitfield(REG(0x40 + (opoffs)), 0, 4)
#define op_total_level(opoffs)	bitfield(REG(0x60 + (opoffs)), 0, 7)
#define op_ksr(opoffs)			bitfield(REG(0x80 + (opoffs)), 6, 2)
#define op_attack_rate(opoffs)	bitfield(REG(0x80 + (opoffs)), 0, 5)
#define op_lfo_am_enable(opoffs)	bitfield(REG(0xa0 + (opoffs)), 7, 1)
#define op_decay_rate(opoffs)	bitfield(REG(0xa0 + (opoffs)), 0, 5)
#define op_detune2(opoffs)		bitfield(REG(0xc0 + (opoffs)), 6, 2)
#define op_sustain_rate(opoffs)	bitfield(REG(0xc0 + (opoffs)), 0, 5)
#define op_sustain_level(opoffs)	bitfield(REG(0xe0 + (opoffs)), 4, 4)
#define op_release_rate(opoffs)	bitfield(REG(0xe0 + (opoffs)), 0, 4)

static void regs_init(YM2151_STATE* chip)
{
	UINT32 index;
	UINT8 am;
	UINT8 pm;

	chip->lfo_counter = 0;
	chip->noise_lfsr = 1;
	chip->noise_counter = 0;
	chip->noise_state = 0;
	chip->noise_lfo = 0;
	chip->lfo_am = 0;

	// create the waveforms
	for (index = 0; index < WAVEFORM_LENGTH; index ++)
		chip->waveform[index] = (UINT16)(abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	// create the LFO waveforms; AM in the low 8 bits, PM in the upper 8
	// waveforms are adjusted to match the pictures in the application manual
	for (index = 0; index < LFO_WAVEFORM_LENGTH; index ++)
	{
		// waveform 0 is a sawtooth
		am = (UINT8)(index ^ 0xff);
		pm = (UINT8)index;
		chip->lfo_waveform[0][index] = (INT16)(am | (pm << 8));

		// waveform 1 is a square wave
		am = bitfield(index, 7, 1) ? 0 : 0xff;
		pm = am ^ 0x80;
		chip->lfo_waveform[1][index] = (INT16)(am | (pm << 8));

		// waveform 2 is a triangle wave
		am = (UINT8)(bitfield(index, 7, 1) ? (index << 1) : ((index ^ 0xff) << 1));
		pm = (UINT8)(bitfield(index, 6, 1) ? am : ~am);
		chip->lfo_waveform[2][index] = (INT16)(am | (pm << 8));

		// waveform 3 is noise; it is filled in dynamically
		chip->lfo_waveform[3][index] = 0;
	}
	return;
}

static void regs_reset(YM2151_STATE* chip)
{
	memset(chip->regdata, 0x00, REGISTERS);
	// enable output on both channels by default
	REG(0x20) = REG(0x21) = REG(0x22) = REG(0x23) = 0xc0;
	REG(0x24) = REG(0x25) = REG(0x26) = REG(0x27) = 0xc0;
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 regs_write(YM2151_STATE* chip, UINT8 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	// LFO AM/PM depth are written to the same register (0x19);
	// redirect the PM depth to an unused neighbor (0x1a)
	if (index == 0x19)
		REG(index + bitfield(data, 7, 1)) = data;
	else if (index != 0x1a)
		REG(index) = data;

	// handle writes to the key on index
	if (index == 0x08)
	{
		*channel = bitfield(data, 0, 3);
		*opmask = bitfield(data, 3, 4);
		return 1;
	}
	return 0;
}

// clock the noise and LFO, returning the LFO PM value
static INT32 clock_noise_and_lfo(YM2151_STATE* chip)
{
	UINT32 freq;
	UINT32 rate;
	UINT32 lfo;
	UINT32 lfo_noise;
	INT32 ampm;
	int rep;

	// base noise frequency is measured at 2x 1/2 FM frequency; this
	// means each tick counts as two steps against the noise counter
	freq = noise_frequency();
	for (rep = 0; rep < 2; rep ++)
	{
		// evidence seems to suggest the LFSR is clocked continually and just
		// sampled at the noise frequency for output purposes; note that the
		// low 8 bits are the most recent 8 bits of history while bits 8-24
		// contain the 17 bit LFSR state
		chip->noise_lfsr <<= 1;
		chip->noise_lfsr |= bitfield(chip->noise_lfsr, 17, 1) ^ bitfield(chip->noise_lfsr, 14, 1) ^ 1;

		// compare against the frequency and latch when we exceed it
		if (chip->noise_counter++ >= freq)
		{
			chip->noise_counter = 0;
			chip->noise_state = (UINT8)bitfield(chip->noise_lfsr, 17, 1);
		}
	}

	// treat the rate as a 4.4 floating-point step value with implied
	// leading 1; this matches exactly the frequencies in the application
	// manual, though it might not be implemented exactly this way on chip
	rate = lfo_rate();
	chip->lfo_counter += (0x10 | bitfield(rate, 0, 4)) << bitfield(rate, 4, 4);

	// bit 1 of the test register is officially undocumented but has been
	// discovered to hold the LFO in reset while active
	if (lfo_reset())
		chip->lfo_counter = 0;

	// now pull out the non-fractional LFO value
	lfo = bitfield(chip->lfo_counter, 22, 8);

	// fill in the noise entry 1 ahead of our current position; this
	// ensures the current value remains stable for a full LFO clock
	// and effectively latches the running value when the LFO advances
	lfo_noise = bitfield(chip->noise_lfsr, 17, 8);
	chip->lfo_waveform[3][(lfo + 1) & 0xff] = (INT16)(lfo_noise | (lfo_noise << 8));

	// fetch the AM/PM values based on the waveform; AM is unsigned and
	// encoded in the low 8 bits, while PM signed and encoded in the upper
	// 8 bits
	ampm = chip->lfo_waveform[lfo_waveform()][lfo];

	// apply depth to the AM value and store for later
	chip->lfo_am = (UINT8)(((ampm & 0xff) * lfo_am_depth()) >> 7);

	// apply depth to the PM value and return it
	return ((ampm >> 8) * (INT32)lfo_pm_depth()) >> 7;
}

// return the AM offset from LFO for the given channel
static UINT32 lfo_am_offset(const YM2151_STATE* chip, UINT32 choffs)
{
	// shift value for AM sensitivity is [*, 0, 1, 2],
	// mapping to values of [0, 23.9, 47.8, and 95.6dB]
	UINT32 am_sensitivity = ch_lfo_am_sens(choffs);
	if (am_sensitivity == 0)
		return 0;

	// raw LFO AM value on OPM is 0-FF, which is already a factor of 2
	// larger than the OPN, putting our staring point at 2x theirs;
	// this works out since our minimum is 2x their maximum
	return (UINT32)chip->lfo_am << (am_sensitivity - 1);
}

// compute the phase step, given a PM value
static UINT32 compute_phase_step(const YM2151_STATE* chip, UINT32 choffs, UINT32 opoffs, const OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	INT32 delta;
	UINT32 pm_sensitivity;
	UINT32 phase_step;

	// OPM logic is rather unique here, due to extra detune
	// and the use of key codes (not to be confused with keycode)

	// start with coarse detune delta
	delta = s_detune2_delta[op_detune2(opoffs)];

	// add in the PM delta
	pm_sensitivity = ch_lfo_pm_sens(choffs);
	if (pm_sensitivity != 0)
	{
		// raw PM value is -127..128 which is +/- 200 cents
		// manual gives these magnitudes in cents:
		//    0, +/-5, +/-10, +/-20, +/-50, +/-100, +/-400, +/-700
		// this roughly corresponds to shifting the 200-cent value:
		//    0  >> 5,  >> 4,  >> 3,  >> 2,  >> 1,   << 1,   << 2
		if (pm_sensitivity < 6)
			delta += lfo_raw_pm >> (6 - pm_sensitivity);
		else
			delta += (INT32)((UINT32)lfo_raw_pm << (pm_sensitivity - 5));
	}

	// apply delta and convert to a frequency number
	phase_step = opm_key_code_to_phase_step(cache->block_freq, delta);

	// apply detune based on the keycode
	phase_step += cache->detune;

	// apply frequency multiplier (which is cached as an x.1 value)
	return (phase_step * cache->multiple) >> 1;
}

// fill the operator cache with prefetched data
static void cache_operator_data(const YM2151_STATE* chip, UINT32 choffs, UINT32 opoffs, OPDATA_CACHE* cache)
{
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 ksrval;

	// get frequency from the channel
	block_freq = cache->block_freq = ch_block_freq(choffs);

	// compute the keycode: block_freq is:
	//
	//     BBBCCCCFFFFFF
	//     ^^^^^
	//
	// the 5-bit keycode is just the top 5 bits (block + top 2 bits
	// of the key code)
	keycode = bitfield(block_freq, 8, 5);

	// detune adjustment
	cache->detune = detune_adjustment(op_detune(opoffs), keycode);

	// multiple value, as an x.1 value (0 means 0.5)
	cache->multiple = op_multiple(opoffs) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on
	// block_freq, detune, and multiple, so compute it after we've done those
	if (lfo_pm_depth() == 0 || ch_lfo_pm_sens(choffs) == 0)
		cache->phase_step = compute_phase_step(chip, choffs, opoffs, cache, 0);
	else
		cache->phase_step = PHASE_STEP_DYNAMIC;

	// total level, scaled by 8
	cache->total_level = op_total_level(opoffs) << 3;

	// 4-bit sustain level, but 15 means 31 so effectively 5 bits
	cache->eg_sustain = op_sustain_level(opoffs);
	cache->eg_sustain |= (cache->eg_sustain + 1) & 0x10;
	cache->eg_sustain <<= 5;

	// determine KSR adjustment for envelope rates
	ksrval = keycode >> (op_ksr(opoffs) ^ 3);
	cache->eg_rate[EG_ATTACK] = (UINT8)effective_rate(op_attack_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_DECAY] = (UINT8)effective_rate(op_decay_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_SUSTAIN] = (UINT8)effective_rate(op_sustain_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_RELEASE] = (UINT8)effective_rate(op_release_rate(opoffs) * 4 + 2, ksrval);
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
static UINT8 op_prepare(const YM2151_STATE* chip, FM_OPERATOR* op)
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
	op->keyon_live &= ~(1 << KEYON_CSM);

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
static void op_clock_phase(const YM2151_STATE* chip, FM_OPERATOR* op, INT32 lfo_raw_pm)
{
	// read from the cache, or recalculate if PM active
	UINT32 phase_step = op->cache.phase_step;
	if (phase_step == PHASE_STEP_DYNAMIC)
		phase_step = compute_phase_step(chip, op->choffs, op->opoffs, &op->cache, lfo_raw_pm);

	// finally apply the step to the current phase value
	op->phase += phase_step;
	return;
}

// master clocking function
static void op_clock(const YM2151_STATE* chip, FM_OPERATOR* op, UINT32 env_counter, INT32 lfo_raw_pm)
{
	// clock the envelope if on an envelope cycle; env_counter is a x.2 value
	if (bitfield(env_counter, 0, 2) == 0)
		op_clock_envelope(op, env_counter >> 2);

	// clock the phase
	op_clock_phase(chip, op, lfo_raw_pm);
	return;
}

// return the effective attenuation of the envelope
static UINT32 op_envelope_attenuation(const YM2151_STATE* chip, const FM_OPERATOR* op, UINT32 am_offset)
{
	UINT32 result = op->env_attenuation;

	// add in LFO AM modulation
	if (op_lfo_am_enable(op->opoffs))
		result += am_offset;

	// add in total level from the cache
	result += op->cache.total_level;

	// clamp to max and return
	return (result < 0x3ff) ? result : 0x3ff;
}

// compute the 14-bit signed volume of this operator, given a phase
// modulation and an AM LFO offset
static INT32 op_compute_volume(const YM2151_STATE* chip, const FM_OPERATOR* op, UINT32 phase, UINT32 am_offset)
{
	UINT32 sin_attenuation;
	UINT32 env_attenuation;
	INT32 result;

	// early out if the envelope is effectively off
	if (op->env_attenuation > EG_QUIET)
		return 0;

	// get the absolute value of the sin, as attenuation, as a 4.8 fixed point value
	sin_attenuation = chip->waveform[phase & (WAVEFORM_LENGTH - 1)];

	// get the attenuation from the evelope generator as a 4.6 value, shifted up to 4.8
	env_attenuation = op_envelope_attenuation(chip, op, am_offset) << 2;

	// combine into a 5.8 value, then convert from attenuation to 13-bit linear volume
	result = (INT32)attenuation_to_volume((sin_attenuation & 0x7fff) + env_attenuation);

	// negate if in the negative part of the sin wave (sign bit gives 14 bits)
	return bitfield(sin_attenuation, 15, 1) ? -result : result;
}

// compute the 14-bit signed noise volume of this operator, given an AM offset
static INT32 op_compute_noise_volume(const YM2151_STATE* chip, const FM_OPERATOR* op, UINT32 am_offset)
{
	// application manual says the logarithmic transform is not applied here, so we
	// just use the raw envelope attenuation, inverted (since 0 attenuation should be
	// maximum), and shift it up from a 10-bit value to an 11-bit value
	INT32 result = (INT32)((op_envelope_attenuation(chip, op, am_offset) ^ 0x3ff) << 1);

	// negate based on the noise state
	return bitfield(chip->noise_state, 0, 1) ? -result : result;
}

// return the current phase value
#define op_phase(op)	((op)->phase >> 10)


//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

// signal key on/off to the channel's operators
static void channel_keyonoff(YM2151_STATE* chip, FM_CHANNEL* ch, UINT32 states, UINT8 type)
{
	UINT8 opnum;
	UINT8 old_live;

	for (opnum = 0; opnum < 4; opnum ++)
	{
		FM_OPERATOR* op = &chip->op[ch->op[opnum]];
		old_live = op->keyon_live;
		op->keyon_live = (UINT8)((op->keyon_live & ~(1 << type)) | (bitfield(states, opnum, 1) << type));
		if (old_live != 0 && op->keyon_live == 0)
			op->keyoff_pending = 1;
	}
	return;
}

// prepare for clocking; returns non-zero if any operator is active
static UINT8 channel_prepare(YM2151_STATE* chip, FM_CHANNEL* ch)
{
	UINT32 active_mask = 0;
	UINT8 opnum;

	// prepare all operators and determine if they are active
	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (op_prepare(chip, &chip->op[ch->op[opnum]]))
			active_mask |= 1 << opnum;
	}
	return (active_mask != 0);
}

// master clock of all operators
static void channel_clock(YM2151_STATE* chip, FM_CHANNEL* ch, UINT32 env_counter, INT32 lfo_raw_pm)
{
	UINT8 opnum;

	// clock the feedback through
	ch->feedback[0] = ch->feedback[1];
	ch->feedback[1] = ch->feedback_in;

	for (opnum = 0; opnum < 4; opnum ++)
		op_clock(chip, &chip->op[ch->op[opnum]], env_counter, lfo_raw_pm);
	return;
}

// combine 4 operators according to the specified algorithm, returning a sum
// according to the rshift and clipmax parameters
static void channel_output_4op(YM2151_STATE* chip, FM_CHANNEL* ch, INT32* output, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = &chip->op[ch->op[0]];
	const FM_OPERATOR* op2 = &chip->op[ch->op[1]];
	const FM_OPERATOR* op3 = &chip->op[ch->op[2]];
	const FM_OPERATOR* op4 = &chip->op[ch->op[3]];
	UINT32 am_offset;
	INT32 opmod;
	UINT32 feedback;
	INT32 op1value;
	UINT32 algorithm_ops;
	INT16 opout[8];
	INT32 result;
	INT32 clipmin;

	// AM amount is the same across all operators; compute it once
	am_offset = lfo_am_offset(chip, ch->choffs);

	// operator 1 has optional self-feedback
	opmod = 0;
	feedback = ch_feedback(ch->choffs);
	if (feedback != 0)
		opmod = (ch->feedback[0] + ch->feedback[1]) >> (10 - feedback);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	ch->feedback_in = (INT16)op_compute_volume(chip, op1, op_phase(op1) + opmod, am_offset);
	op1value = ch->feedback_in;

	// now that the feedback has been computed, skip the rest if all volumes
	// are clear; no need to do all this work for nothing
	if (ch_output_any(ch->choffs) == 0)
		return;

	algorithm_ops = s_algorithm_ops[ch_algorithm(ch->choffs)];

	// populate the opout table
	opout[0] = 0;
	opout[1] = (INT16)op1value;

	// compute the 14-bit volume/value of operator 2
	opmod = opout[bitfield(algorithm_ops, 0, 1)] >> 1;
	opout[2] = (INT16)op_compute_volume(chip, op2, op_phase(op2) + opmod, am_offset);
	opout[5] = opout[1] + opout[2];

	// compute the 14-bit volume/value of operator 3
	opmod = opout[bitfield(algorithm_ops, 1, 3)] >> 1;
	opout[3] = (INT16)op_compute_volume(chip, op3, op_phase(op3) + opmod, am_offset);
	opout[6] = opout[1] + opout[3];
	opout[7] = opout[2] + opout[3];

	// compute the 14-bit volume/value of operator 4; this could be a noise
	// value on channel 7; all algorithms consume OP4 output at a minimum
	if (noise_enable() && ch->choffs == 7)
	{
		result = op_compute_noise_volume(chip, op4, am_offset);
	}
	else
	{
		opmod = opout[bitfield(algorithm_ops, 4, 3)] >> 1;
		result = op_compute_volume(chip, op4, op_phase(op4) + opmod, am_offset);
	}
	result >>= rshift;

	// optionally add OP1, OP2, OP3
	clipmin = -clipmax - 1;
	if (bitfield(algorithm_ops, 7, 1) != 0)
		result = clamp(result + (opout[1] >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 8, 1) != 0)
		result = clamp(result + (opout[2] >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 9, 1) != 0)
		result = clamp(result + (opout[3] >> rshift), clipmin, clipmax);

	// add to the output
	if (ch_output_0(ch->choffs))
		output[0] += result;
	if (ch_output_1(ch->choffs))
		output[1] += result;
	return;
}


//----------------------------------------------------------------------------
// FM engine
//----------------------------------------------------------------------------

// set/reset bits in the status register
static void engine_set_reset_status(YM2151_STATE* chip, UINT8 set, UINT8 reset)
{
	chip->status = (chip->status | set) & ~(reset | STATUS_BUSY);
	return;
}

// update the state of the given timer
static void engine_update_timer(YM2151_STATE* chip, UINT32 tnum, UINT32 enable, INT32 delta_clocks)
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

// timer has expired - signal status and CSM key on
static void engine_timer_expired(YM2151_STATE* chip, UINT32 tnum)
{
	UINT32 chnum;

	// update status
	if (tnum == 0 && enable_timer_a())
		engine_set_reset_status(chip, STATUS_TIMERA, 0);
	else if (tnum == 1 && enable_timer_b())
		engine_set_reset_status(chip, STATUS_TIMERB, 0);

	// if timer A fired in CSM mode, trigger CSM on all relevant channels
	if (tnum == 0 && csm())
	{
		for (chnum = 0; chnum < CHANNELS; chnum ++)
		{
			if (bitfield(CSM_TRIGGER_MASK, chnum, 1))
			{
				channel_keyonoff(chip, &chip->channel[chnum], 0xf, KEYON_CSM);
				chip->modified_channels |= 1 << chnum;
			}
		}
	}

	// reset
	chip->timer_running[tnum] = 0;
	engine_update_timer(chip, tnum, 1, 0);
	return;
}

// handle a mode register write
static void engine_mode_write(YM2151_STATE* chip, UINT8 data)
{
	UINT8 reset_mask = 0;

	// mark all channels as modified
	chip->modified_channels = ALL_CHANNELS;

	// actually write the mode register now
	REG(REG_MODE) = data;

	// reset timer status
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
	return;
}

// handle writes to the OPM registers
static void engine_write(YM2151_STATE* chip, UINT8 regnum, UINT8 data)
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
			channel_keyonoff(chip, &chip->channel[keyon_channel], keyon_opmask, KEYON_NORMAL);
	}
	return;
}

// iterate over all channels, clocking them forward one step
static void engine_clock(YM2151_STATE* chip)
{
	UINT8 chnum;
	INT32 lfo_raw_pm;

	// update the clock counter
	chip->total_clocks ++;

	// if something was modified, prepare
	// also prepare every 4k samples to catch ending notes
	if (chip->modified_channels != 0 || chip->prepare_count++ >= 4096)
	{
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

	// increment by 1 and manually wrap when we reach the divide count (3)
	if (bitfield(++chip->env_counter, 0, 2) == 3)
		chip->env_counter += 4 - 3;

	// clock the noise generator and LFO
	lfo_raw_pm = clock_noise_and_lfo(chip);

	// now update the state of all the channels and operators
	for (chnum = 0; chnum < CHANNELS; chnum ++)
		channel_clock(chip, &chip->channel[chnum], chip->env_counter, lfo_raw_pm);
	return;
}

// compute a sum over the relevant channels
static void engine_output(YM2151_STATE* chip, INT32* output, UINT32 rshift, INT32 clipmax, UINT32 chanmask)
{
	UINT8 chnum;

	// mask out inactive channels
	chanmask &= chip->active_channels;

	// sum over all the desired channels
	for (chnum = 0; chnum < CHANNELS; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1))
			channel_output_4op(chip, &chip->channel[chnum], output, rshift, clipmax);
	}
	return;
}

static void engine_reset(YM2151_STATE* chip)
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

static void engine_init(YM2151_STATE* chip)
{
	UINT8 curChn;
	UINT8 curOp;

	regs_init(chip);

	chip->active_channels = ALL_CHANNELS;
	chip->modified_channels = ALL_CHANNELS;

	// create the operators; on OPM the operator offset is the operator number
	for (curOp = 0; curOp < OPERATORS; curOp ++)
	{
		chip->op[curOp].opoffs = curOp;
		op_reset(&chip->op[curOp]);
	}

	// create the channels and assign their operators; the channel index order is
	// 0,2,1,3, because the order in the map is: carrier 1, carrier 2, modulator 1, modulator 2
	for (curChn = 0; curChn < CHANNELS; curChn ++)
	{
		FM_CHANNEL* ch = &chip->channel[curChn];

		ch->choffs = curChn;
		ch->op[0] = curChn + 0;
		ch->op[1] = curChn + 16;
		ch->op[2] = curChn + 8;
		ch->op[3] = curChn + 24;
		for (curOp = 0; curOp < 4; curOp ++)
			chip->op[ch->op[curOp]].choffs = ch->choffs;
	}
	return;
}


//----------------------------------------------------------------------------
// YM2151
//----------------------------------------------------------------------------

// generate one sample of sound
static void ym2151_generate(YM2151_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	INT32 temp[2];
	UINT8 chan;
	UINT8 tnum;

	// expire timers that run out on this sample
	for (tnum = 0; tnum < 2; tnum ++)
	{
		if (chip->timer_count[tnum] > 0 && --chip->timer_count[tnum] == 0)
			engine_timer_expired(chip, tnum);
	}

	// clock the system
	engine_clock(chip);

	// update the FM content; OPM is full 14-bit with no intermediate clipping;
	// muted channels are still computed so their feedback state stays live
	output[0] = output[1] = 0;
	for (chan = 0; chan < CHANNELS; chan ++)
	{
		temp[0] = temp[1] = 0;
		engine_output(chip, temp, 0, 32767, 1 << chan);
		if ((chip->muteMask >> chan) & 0x01)
			continue;
		output[0] += temp[0];
		output[1] += temp[1];
	}

	// YM2151 uses an external DAC (YM3012) with mantissa/exponent format
	// convert to 10.3 floating point value and back to simulate truncation
	*outL = roundtrip_fp(output[0]);
	*outR = roundtrip_fp(output[1]);
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

static UINT8 device_start_ym2151(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YM2151_STATE* chip;

	chip = (YM2151_STATE*)calloc(1, sizeof(YM2151_STATE));
	if (chip == NULL)
		return 0xFF;

	engine_init(chip);
	engine_reset(chip);
	chip->muteMask = 0x00;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / 64, &devDef);
	return 0x00;
}

static void device_stop_ym2151(void* info)
{
	free(info);
	return;
}

static void device_reset_ym2151(void* info)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	// ymfm's reset leaves the address register alone; a device reset clears it too
	chip->address = 0;
	engine_reset(chip);
	return;
}

static void ym2151_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2151_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2151_w(void* info, UINT8 offset, UINT8 data)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	if (! (offset & 1))
		chip->address = data;	// address port
	else
		engine_write(chip, chip->address, data);	// data port
	return;
}

static UINT8 ym2151_r(void* info, UINT8 offset)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	// status port, YM2203 compatible; the data port is unused
	if (! (offset & 1))
		return 0xFF;
	return chip->status & ~STATUS_BUSY;
}

static void ym2151_set_mute_mask(void* info, UINT32 MuteMask)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	chip->muteMask = MuteMask;
	return;
}
