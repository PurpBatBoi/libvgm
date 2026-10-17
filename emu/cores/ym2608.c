// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Yamaha YM2203 (OPN), YM2608 (OPNA), YM2610/YM2610B (OPNB)

    C port of the YM2203, YM2608 and YM2610 paths of ymfm (ymfm_opn.cpp,
    ymfm_fm.ipp, ymfm_adpcm.cpp, ymfm.h).
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
    - Only the OPN/OPNA parts of the ymfm FM engine are kept: no OPL
      rhythm, OPM noise, OPLL depress or OPZ reverb states.
    - ymfm's C++ templates on the register type become a runtime flag
      (is_opna: 6 channels, stereo, LFO; YM2608 and YM2610).
    - The SSG part is not emulated here. As in libvgm's other OPN cores,
      it is a linked AY8910 device: SSG register accesses are forwarded
      to it, and prescaler changes update its clock.
    - One update sample is one FM sample (clock / (12 * prescale) on the
      YM2203, clock / (24 * prescale) on the YM2608, clock / 144 on the
      YM2610). A prescaler change changes the sample rate and calls the
      sample rate change callback.
    - ymfm leaves timers to its host. Here they count in FM samples and
      expire before the sample is clocked, so Timer A also drives CSM
      key-on.
    - The busy flag and the IRQ line are not emulated.
    - YM2608/YM2610: ymfm mixes FM and ADPCM at 13 bits, 6 dB below the
      YM2203. The output is doubled after ymfm's mix, so the level and the
      balance with the linked SSG match libvgm's other OPN cores.
    - YM2610: the ADPCM-A and ADPCM-B sample memories are filled by the
      player (DEVRW_MEMSIZE, DEVRW_BLOCK with 'A'/'B'). YM2608: the ADPCM-B
      memory is filled the same way ('B'). Reads outside a memory return 0.
    - YM2608: Yamaha's rhythm (ADPCM-A) ROM is not included. The rhythm
      channels play the Tetorica OPNA rhythm sounds instead (BSD-3, see
      ym2608_rhythm.h): the same layout and encoding, but different sounds.
      The player can replace them with another 8 KB rhythm ROM
      (DEVRW_BLOCK with 0x524F, 'RO', as for the YMF278B ROM).
    - YM2610: the ADPCM-A channels are silent while no ADPCM-A memory is
      loaded. (ymfm would decode zero bytes into a sawtooth.)
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
#include "../SoundEmu.h"	// for SndEmu_GetDeviceFunc
#include "ayintf.h"	// for AY8910_CFG and the AYTYPE constants

#include "ym2608.h"
#include "ym2608_rhythm.h"


static void ym2203_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void ym2608_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void ym2610_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym2203(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static UINT8 device_start_ym2608(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static UINT8 device_start_ym2610(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_opn(void* info);
static void device_reset_opn(void* info);
static void ym2203_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2203_r(void* info, UINT8 offset);
static void ym2608_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2608_r(void* info, UINT8 offset);
static void ym2610_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2610_r(void* info, UINT8 offset);
static void opn_alloc_pcmroma(void* info, UINT32 memsize);
static void opn_write_pcmroma(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void opn_alloc_pcmromb(void* info, UINT32 memsize);
static void opn_write_pcmromb(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void ym2608_write_rhythmrom(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void opn_set_mute_mask(void* info, UINT32 MuteMask);
static void opn_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr);
static UINT8 opn_link_device(void* info, UINT8 linkID, const DEV_INFO* devInfLink);


static DEVDEF_RWFUNC devFunc2203[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2203_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2203_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opn_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef2203 =
{
	"YM2203", "ymfm", FCC_YMFM,

	device_start_ym2203,
	device_stop_opn,
	device_reset_opn,
	ym2203_update,

	NULL,	// SetOptionBits
	opn_set_mute_mask,
	NULL,	// SetPanning
	opn_set_srchg_cb,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	opn_link_device,	// LinkDevice

	devFunc2203,	// rwFuncs
};

static DEVDEF_RWFUNC devFunc2608[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2608_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2608_r},
	// the ADPCM-B memory comes first: players look memory functions up with user 0
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 'B', opn_write_pcmromb},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 'B', opn_alloc_pcmromb},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0x524F, ym2608_write_rhythmrom},	// 0x524F = 'RO' (rhythm ROM)
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opn_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef2608 =
{
	"YM2608", "ymfm", FCC_YMFM,

	device_start_ym2608,
	device_stop_opn,
	device_reset_opn,
	ym2608_update,

	NULL,	// SetOptionBits
	opn_set_mute_mask,
	NULL,	// SetPanning
	opn_set_srchg_cb,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	opn_link_device,	// LinkDevice

	devFunc2608,	// rwFuncs
};

static DEVDEF_RWFUNC devFunc2610[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2610_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2610_r},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 'A', opn_write_pcmroma},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 'A', opn_alloc_pcmroma},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 'B', opn_write_pcmromb},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 'B', opn_alloc_pcmromb},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opn_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef2610 =
{
	"YM2610", "ymfm", FCC_YMFM,

	device_start_ym2610,
	device_stop_opn,
	device_reset_opn,
	ym2610_update,

	NULL,	// SetOptionBits
	opn_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback (the YM2610 has no prescaler)
	NULL,	// SetLoggingCallback
	opn_link_device,	// LinkDevice

	devFunc2610,	// rwFuncs
};
static DEV_DEF devDef2610B =
{
	"YM2610B", "ymfm", FCC_YMFM,

	device_start_ym2610,
	device_stop_opn,
	device_reset_opn,
	ym2610_update,

	NULL,	// SetOptionBits
	opn_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback (the YM2610 has no prescaler)
	NULL,	// SetLoggingCallback
	opn_link_device,	// LinkDevice

	devFunc2610,	// rwFuncs
};

static const char* DeviceName_YM2203(const DEV_GEN_CFG* devCfg)
{
	return "YM2203";
}

static const char* DeviceName_YM2608(const DEV_GEN_CFG* devCfg)
{
	return "YM2608";
}

static const char* DeviceName_YM2610(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && devCfg->flags)
		return "YM2610B";
	return "YM2610";
}

#define DEV_CHN_COUNT_YM2203	3
static UINT16 DeviceChannels_YM2203(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_YM2203;
}

static const char** DeviceChannelNames_YM2203(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_YM2203] =
	{
		"FM 1", "FM 2", "FM 3",
	};
	return names;
}

// the YM2610 lists 6 FM channels too, because the mute mask uses the
// same bits for both variants (FM 1 and FM 4 don't exist on the YM2610)
#define DEV_CHN_COUNT_OPNA	13
static UINT16 DeviceChannels_OPNA(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_OPNA;
}

static const char** DeviceChannelNames_OPNA(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_OPNA] =
	{
		"FM 1", "FM 2", "FM 3", "FM 4", "FM 5", "FM 6",
		"ADPCM-A 1", "ADPCM-A 2", "ADPCM-A 3", "ADPCM-A 4", "ADPCM-A 5", "ADPCM-A 6",
		"ADPCM-B",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	static const DEVLINK_IDS dlIDs = {1, {DEVID_AY8910}};
	return &dlIDs;
}

const DEV_DECL sndDev_YM2203 =
{
	DEVID_YM2203,
	DeviceName_YM2203,
	DeviceChannels_YM2203,
	DeviceChannelNames_YM2203,
	DeviceLinkIDs,
	{	// cores
		&devDef2203,
		NULL
	}
};

const DEV_DECL sndDev_YM2608 =
{
	DEVID_YM2608,
	DeviceName_YM2608,
	DeviceChannels_OPNA,
	DeviceChannelNames_OPNA,
	DeviceLinkIDs,
	{	// cores
		&devDef2608,
		NULL
	}
};

const DEV_DECL sndDev_YM2610 =
{
	DEVID_YM2610,
	DeviceName_YM2610,
	DeviceChannels_OPNA,
	DeviceChannelNames_OPNA,
	DeviceLinkIDs,
	{	// cores
		&devDef2610,
		NULL
	}
};


#define CHIP_YM2203		0
#define CHIP_YM2608		1
#define CHIP_YM2610		2

#define MAX_CHANNELS	6
#define MAX_OPERATORS	(MAX_CHANNELS * 4)
#define REGISTERS		0x200
#define WAVEFORM_LENGTH	0x400
#define REG_MODE		0x27
#define CSM_TRIGGER_MASK	(1 << 2)
#define DEFAULT_PRESCALE	6

#define STATUS_TIMERA	0x01
#define STATUS_TIMERB	0x02
#define STATUS_BUSY		0x80

// YM2608 extended status bits
#define STATUS_ADPCM_B_EOS		0x04
#define STATUS_ADPCM_B_BRDY		0x08
#define STATUS_ADPCM_B_ZERO		0x10
#define STATUS_ADPCM_B_PLAYING	0x20

// YM2610 end-of-sample flags
#define EOS_FLAGS_MASK	0xbf

// mute mask bits beyond the 6 FM channels
#define MUTE_ADPCM_A	6
#define MUTE_ADPCM_B	12

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

// ADPCM-A channels
#define ADPCM_A_CHANNELS	6
#define ADPCM_A_REGISTERS	0x30

// ADPCM-B channel
#define ADPCM_B_REGISTERS	0x11
#define ADPCM_B_STEP_MIN	127
#define ADPCM_B_STEP_MAX	24576
#define ADPCM_B_STATUS_EOS		0x01
#define ADPCM_B_STATUS_BRDY		0x02
#define ADPCM_B_STATUS_PLAYING	0x04

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
	UINT8 ssg_inverted;		// non-zero if the output should be inverted (bit 0)
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

typedef struct _adpcm_a_channel
{
	UINT32 playing;			// currently playing?
	UINT32 curnibble;		// index of the current nibble
	UINT32 curbyte;			// current byte of data
	UINT32 curaddress;		// current address
	INT32 accumulator;		// accumulator
	INT32 step_index;		// index in the stepping table
} ADPCM_A_CHANNEL;

typedef struct _adpcm_b_channel
{
	UINT8 regdata[ADPCM_B_REGISTERS];	// register data
	UINT32 status;			// currently playing?
	UINT32 curnibble;		// index of the current nibble
	UINT32 curbyte;			// current byte of data
	UINT32 dummy_read;		// dummy read tracker
	UINT32 position;		// current fractional position
	UINT32 curaddress;		// current address
	INT32 accumulator;		// accumulator
	INT32 prev_accum;		// previous accumulator (for linear interp)
	INT32 adpcm_step;		// next forecast
} ADPCM_B_CHANNEL;

// functions of the linked SSG (AY8910) device
typedef struct _ssg_funcs
{
	DEVFUNC_WRITE_A8D8 write;
	DEVFUNC_READ_A8D8 read;
	DEVFUNC_WRITE_CLOCK set_clock;
	DEVFUNC_CTRL reset;
	void* param;
} SSG_FUNCS;

typedef struct _opn_state
{
	DEV_DATA _devData;

	UINT8 type;				// CHIP_YM2203, CHIP_YM2608 or CHIP_YM2610
	UINT8 is_opna;			// OPNA register layout (YM2608, YM2610)
	UINT8 channels;			// number of FM channels (3 or 6)
	UINT32 all_channels;	// mask of all FM channels
	UINT32 fm_mask;			// mask of FM channels that are clocked
	UINT32 clock;			// input clock
	UINT8 prescale;			// clock prescale (6, 3 or 2)

	// OPN(A) registers
	UINT32 lfo_counter;		// LFO counter
	UINT8 lfo_am;			// current LFO AM value
	UINT8 regdata[REGISTERS];	// register data
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
	FM_CHANNEL channel[MAX_CHANNELS];
	FM_OPERATOR op[MAX_OPERATORS];

	// chip
	UINT16 address;			// address register
	UINT8 irq_enable;		// YM2608: IRQ enable register
	UINT8 flag_control;		// YM2608: flag control register
	UINT8 eos_status;		// YM2610: end-of-sample signals
	UINT8 flag_mask;		// YM2610: flag mask control

	// ADPCM-A
	UINT8 adpcm_a_regs[ADPCM_A_REGISTERS];
	ADPCM_A_CHANNEL adpcm_a[ADPCM_A_CHANNELS];
	UINT32 adpcm_a_shift;	// address bits shift-left
	const UINT8* adpcm_a_data;	// ADPCM-A samples: the rhythm ROM or adpcm_a_mem
	UINT32 adpcm_a_size;
	UINT8* adpcm_a_mem;		// YM2610: ADPCM-A sample memory
	UINT32 adpcm_a_mem_size;

	// ADPCM-B
	ADPCM_B_CHANNEL adpcm_b;
	UINT32 adpcm_b_shift;	// fixed address shift, 0 = from the registers
	UINT8* adpcm_b_mem;		// ADPCM-B sample memory
	UINT32 adpcm_b_mem_size;

	SSG_FUNCS ssg;
	DEVCB_SRATE_CHG smpRateFunc;
	void* smpRateData;

	UINT32 muteMask;
} OPN_STATE;


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

// 2 shift values to apply to the top 7 bits of fnum, per PM depth and step;
// effectively a cheap multiply by a constant value containing 0-2 bits
static const UINT8 s_lfo_pm_shifts[8][8] =
{
	{ 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 },
	{ 0x77, 0x77, 0x77, 0x77, 0x72, 0x72, 0x72, 0x72 },
	{ 0x77, 0x77, 0x77, 0x72, 0x72, 0x72, 0x17, 0x17 },
	{ 0x77, 0x77, 0x72, 0x72, 0x17, 0x17, 0x12, 0x12 },
	{ 0x77, 0x77, 0x72, 0x17, 0x17, 0x17, 0x12, 0x07 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 }
};

// LFO clock dividers, converted from the frequencies in the application manual
static const UINT8 s_lfo_max_count[8] = { 109, 78, 72, 68, 63, 45, 9, 6 };

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

// given the 7 most significant frequency number bits, plus a 3-bit PM depth
// value and a signed 5-bit raw PM value, return a signed PM adjustment to the
// frequency; algorithm written to match Nuked behavior
static INT32 opn_lfo_pm_phase_adjustment(UINT32 fnum_bits, UINT32 pm_sensitivity, INT32 lfo_raw_pm)
{
	INT32 abs_pm;
	UINT32 shifts;
	INT32 adjust;

	// look up the relevant shifts
	abs_pm = (lfo_raw_pm < 0) ? -lfo_raw_pm : lfo_raw_pm;
	shifts = s_lfo_pm_shifts[pm_sensitivity][bitfield(abs_pm, 0, 3)];

	// compute the adjustment
	adjust = (fnum_bits >> bitfield(shifts, 0, 4)) + (fnum_bits >> bitfield(shifts, 4, 4));
	if (pm_sensitivity > 5)
		adjust <<= pm_sensitivity - 5;
	adjust >>= 2;

	// every 16 cycles it inverts sign
	return (lfo_raw_pm < 0) ? -adjust : adjust;
}

// apply KSR to the raw ADSR rate, ignoring ksr if the raw value is 0, and
// clamping to 63
INLINE UINT32 effective_rate(UINT32 rawrate, UINT32 ksr)
{
	if (rawrate == 0)
		return 0;
	return (rawrate + ksr < 63) ? (rawrate + ksr) : 63;
}

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

// ADPCM-B step scale: 0.9, 0.9, 0.9, 0.9, 1.2, 1.6, 2.0, 2.4
static const UINT8 s_adpcm_b_step_scale[8] = { 57, 57, 57, 57, 77, 102, 128, 153 };

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


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

#define REG(offset)		(chip->regdata[offset])

// system-wide registers; the YM2203 (OPN) has no LFO
#define lfo_enable()		(chip->is_opna ? bitfield(REG(0x22), 3, 1) : 0)
#define lfo_rate()			bitfield(REG(0x22), 0, 3)
#define timer_a_value()		((bitfield(REG(0x24), 0, 8) << 2) | bitfield(REG(0x25), 0, 2))
#define timer_b_value()		bitfield(REG(0x26), 0, 8)
#define csm()				(bitfield(REG(0x27), 6, 2) == 2)
#define multi_freq()		(bitfield(REG(0x27), 6, 2) != 0)
#define reset_timer_b()		bitfield(REG(0x27), 5, 1)
#define reset_timer_a()		bitfield(REG(0x27), 4, 1)
#define enable_timer_b()	bitfield(REG(0x27), 3, 1)
#define enable_timer_a()	bitfield(REG(0x27), 2, 1)
#define load_timer_b()		bitfield(REG(0x27), 1, 1)
#define load_timer_a()		bitfield(REG(0x27), 0, 1)
#define multi_block_freq(num)	((bitfield(REG(0xac + (num)), 0, 6) << 8) | bitfield(REG(0xa8 + (num)), 0, 8))

// per-channel registers; the YM2203 (OPN) is mono and has no LFO
#define ch_block_freq(choffs)	((bitfield(REG(0xa4 + (choffs)), 0, 6) << 8) | bitfield(REG(0xa0 + (choffs)), 0, 8))
#define ch_feedback(choffs)		bitfield(REG(0xb0 + (choffs)), 3, 3)
#define ch_algorithm(choffs)	bitfield(REG(0xb0 + (choffs)), 0, 3)
#define ch_output_any(choffs)	(chip->is_opna ? bitfield(REG(0xb4 + (choffs)), 6, 2) : 1)
#define ch_output_0(choffs)		(chip->is_opna ? bitfield(REG(0xb4 + (choffs)), 7, 1) : 1)
#define ch_output_1(choffs)		(chip->is_opna ? bitfield(REG(0xb4 + (choffs)), 6, 1) : 0)
#define ch_lfo_am_sens(choffs)	(chip->is_opna ? bitfield(REG(0xb4 + (choffs)), 4, 2) : 0)
#define ch_lfo_pm_sens(choffs)	(chip->is_opna ? bitfield(REG(0xb4 + (choffs)), 0, 3) : 0)

// per-operator registers
#define op_detune(opoffs)		bitfield(REG(0x30 + (opoffs)), 4, 3)
#define op_multiple(opoffs)		bitfield(REG(0x30 + (opoffs)), 0, 4)
#define op_total_level(opoffs)	bitfield(REG(0x40 + (opoffs)), 0, 7)
#define op_ksr(opoffs)			bitfield(REG(0x50 + (opoffs)), 6, 2)
#define op_attack_rate(opoffs)	bitfield(REG(0x50 + (opoffs)), 0, 5)
#define op_decay_rate(opoffs)	bitfield(REG(0x60 + (opoffs)), 0, 5)
#define op_lfo_am_enable(opoffs)	(chip->is_opna ? bitfield(REG(0x60 + (opoffs)), 7, 1) : 0)
#define op_sustain_rate(opoffs)	bitfield(REG(0x70 + (opoffs)), 0, 5)
#define op_sustain_level(opoffs)	bitfield(REG(0x80 + (opoffs)), 4, 4)
#define op_release_rate(opoffs)	bitfield(REG(0x80 + (opoffs)), 0, 4)
#define op_ssg_eg_enable(opoffs)	bitfield(REG(0x90 + (opoffs)), 3, 1)
#define op_ssg_eg_mode(opoffs)	bitfield(REG(0x90 + (opoffs)), 0, 3)

static void regs_reset(OPN_STATE* chip)
{
	memset(chip->regdata, 0x00, REGISTERS);
	if (chip->is_opna)
	{
		// enable output on both channels by default
		REG(0xb4) = REG(0xb5) = REG(0xb6) = 0xc0;
		REG(0x1b4) = REG(0x1b5) = REG(0x1b6) = 0xc0;
	}
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 regs_write(OPN_STATE* chip, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	// writes in the 0xa0-af/0x1a0-af region are handled as latched pairs
	// borrow unused registers 0xb8-bf as temporary holding locations
	if ((index & 0xf0) == 0xa0)
	{
		UINT32 latchindex;

		if (bitfield(index, 0, 2) == 3)
			return 0;

		latchindex = 0xb8 | bitfield(index, 3, 1);

		// writes to the upper half just latch (only low 6 bits matter)
		if (bitfield(index, 2, 1))
		{
			REG(latchindex) = data & 0x3f;
		}
		// writes to the lower half also apply said latch
		else
		{
			REG(index) = data;
			REG(index | 4) = REG(latchindex);
		}
		return 0;
	}
	else if ((index & 0xf8) == 0xb8)
	{
		// registers 0xb8-0xbf are used internally
		return 0;
	}

	// everything else is normal
	REG(index) = data;

	// handle writes to the key on index
	if (index == 0x28)
	{
		*channel = bitfield(data, 0, 2);
		if (*channel == 3)
			return 0;
		if (chip->is_opna)
			*channel += bitfield(data, 2, 1) * 3;
		*opmask = bitfield(data, 4, 4);
		return 1;
	}
	return 0;
}

// clock the LFO, returning the LFO PM value
static INT32 clock_noise_and_lfo(OPN_STATE* chip)
{
	UINT32 subcount;
	INT32 pm;

	// if LFO not enabled (not present on OPN), quick exit with 0s
	if (! lfo_enable())
	{
		chip->lfo_counter = 0;

		// special case: if LFO is disabled on OPNA, it basically just keeps the counter
		// at 0; since position 0 gives an AM value of 0x3f, it is important to reflect
		// that here; for example, MegaDrive Venom plays some notes with LFO globally
		// disabled but enabling LFO on the operators, and it expects this added attenutation
		chip->lfo_am = chip->is_opna ? 0x3f : 0x00;
		return 0;
	}

	subcount = (UINT8)(chip->lfo_counter ++);

	// when we cross the divider count, add enough to zero it and cause an
	// increment at bit 8; the 7-bit value lives from bits 8-14
	if (subcount >= s_lfo_max_count[lfo_rate()])
	{
		// note: to match the published values this should be 0x100 - subcount;
		// however, tests on the hardware and nuked bear out an off-by-one
		// error exists that causes the max LFO rate to be faster than published
		chip->lfo_counter += 0x101 - subcount;
	}

	// AM value is 7 bits, staring at bit 8; grab the low 6 directly
	chip->lfo_am = (UINT8)bitfield(chip->lfo_counter, 8, 6);

	// first half of the AM period (bit 6 == 0) is inverted
	if (bitfield(chip->lfo_counter, 8+6, 1) == 0)
		chip->lfo_am ^= 0x3f;

	// PM value is 5 bits, starting at bit 10; grab the low 3 directly
	pm = (INT32)bitfield(chip->lfo_counter, 10, 3);

	// PM is reflected based on bit 3
	if (bitfield(chip->lfo_counter, 10+3, 1))
		pm ^= 7;

	// PM is negated based on bit 4
	return bitfield(chip->lfo_counter, 10+4, 1) ? -pm : pm;
}

// return the AM offset from LFO for the given channel
static UINT32 lfo_am_offset(const OPN_STATE* chip, UINT32 choffs)
{
	// shift value for AM sensitivity is [7, 3, 1, 0],
	// mapping to values of [0, 1.4, 5.9, and 11.8dB]
	UINT32 am_shift = (1 << (ch_lfo_am_sens(choffs) ^ 3)) - 1;

	// raw LFO AM value on OPN is 0-3F, scale that up by a factor of 2
	// (giving 7 bits) before applying the final shift
	return ((UINT32)chip->lfo_am << 1) >> am_shift;
}

// compute the phase step, given a PM value
static UINT32 compute_phase_step(const OPN_STATE* chip, UINT32 choffs, const OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	UINT32 fnum;
	UINT32 pm_sensitivity;
	UINT32 block;
	UINT32 phase_step;

	// extract frequency number (low 11 bits of block_freq)
	fnum = bitfield(cache->block_freq, 0, 11) << 1;

	// if there's a non-zero PM sensitivity, compute the adjustment
	pm_sensitivity = ch_lfo_pm_sens(choffs);
	if (pm_sensitivity != 0)
	{
		// apply the phase adjustment based on the upper 7 bits
		// of FNUM and the PM depth parameters
		fnum += opn_lfo_pm_phase_adjustment(bitfield(cache->block_freq, 4, 7), pm_sensitivity, lfo_raw_pm);

		// keep fnum to 12 bits
		fnum &= 0xfff;
	}

	// apply block shift to compute phase step
	block = bitfield(cache->block_freq, 11, 3);
	phase_step = (fnum << block) >> 2;

	// apply detune based on the keycode
	phase_step += cache->detune;

	// clamp to 17 bits in case detune overflows
	phase_step &= 0x1ffff;

	// apply frequency multiplier (which is cached as an x.1 value)
	return (phase_step * cache->multiple) >> 1;
}

// fill the operator cache with prefetched data
static void cache_operator_data(const OPN_STATE* chip, UINT32 choffs, UINT32 opoffs, OPDATA_CACHE* cache)
{
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 ksrval;

	// get frequency from the channel
	block_freq = cache->block_freq = ch_block_freq(choffs);

	// if multi-frequency mode is enabled and this is channel 2,
	// fetch one of the special frequencies
	if (multi_freq() && choffs == 2)
	{
		if (opoffs == 2)
			block_freq = cache->block_freq = multi_block_freq(1);
		else if (opoffs == 10)
			block_freq = cache->block_freq = multi_block_freq(2);
		else if (opoffs == 6)
			block_freq = cache->block_freq = multi_block_freq(0);
	}

	// compute the keycode: block_freq is:
	//
	//     BBBFFFFFFFFFFF
	//     ^^^^???
	//
	// the 5-bit keycode uses the top 4 bits plus a magic formula
	// for the final bit
	keycode = bitfield(block_freq, 10, 4) << 1;

	// lowest bit is determined by a mix of next lower FNUM bits
	// according to this equation from the YM2608 manual:
	//
	//   (F11 & (F10 | F9 | F8)) | (!F11 & F10 & F9 & F8)
	//
	// for speed, we just look it up in a 16-bit constant
	keycode |= bitfield(0xfe80, bitfield(block_freq, 7, 4), 1);

	// detune adjustment
	cache->detune = detune_adjustment(op_detune(opoffs), keycode);

	// multiple value, as an x.1 value (0 means 0.5)
	cache->multiple = op_multiple(opoffs) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on
	// block_freq, detune, and multiple, so compute it after we've done those
	if (lfo_enable() == 0 || ch_lfo_pm_sens(choffs) == 0)
		cache->phase_step = compute_phase_step(chip, choffs, cache, 0);
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
	op->ssg_inverted = 0;
	op->key_state = 0;
	op->keyon_live = 0;
	op->keyoff_pending = 0;
	return;
}

// start the attack phase; called when a keyon happens or when an SSG-EG
// cycle is complete and restarts
static void op_start_attack(const OPN_STATE* chip, FM_OPERATOR* op, UINT8 is_restart)
{
	// don't change anything if already in attack state
	if (op->env_state == EG_ATTACK)
		return;
	op->env_state = EG_ATTACK;

	// generally not inverted at start, except if SSG-EG is enabled and
	// one of the inverted modes is specified; leave this alone on a
	// restart, as it is managed by the clock_ssg_eg_state() code
	if (! is_restart)
		op->ssg_inverted = (UINT8)(op_ssg_eg_enable(op->opoffs) & bitfield(op_ssg_eg_mode(op->opoffs), 2, 1));

	// reset the phase when we start an attack due to a key on
	// (but not when due to an SSG-EG restart except in certain cases
	// managed directly by the SSG-EG code)
	if (! is_restart)
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

	// if attenuation if inverted due to SSG-EG, snap the inverted attenuation
	// as the starting point
	if (op->ssg_inverted)
	{
		op->env_attenuation = (0x200 - op->env_attenuation) & 0x3ff;
		op->ssg_inverted = 0;
	}
	return;
}

// clock the keystate to match the incoming keystate
static void op_clock_keystate(const OPN_STATE* chip, FM_OPERATOR* op, UINT32 keystate)
{
	// has the key changed?
	if ((keystate ^ op->key_state) != 0)
	{
		op->key_state = (UINT8)keystate;

		// if the key has turned on, start the attack
		if (keystate != 0)
			op_start_attack(chip, op, 0);
		// otherwise, start the release
		else
			op_start_release(op);
	}
	return;
}

// prepare prior to clocking; returns non-zero while the operator is audible
static UINT8 op_prepare(const OPN_STATE* chip, FM_OPERATOR* op)
{
	// cache the data
	cache_operator_data(chip, op->choffs, op->opoffs, &op->cache);

	// clock the key state; a key off since the last prepare releases first,
	// so a key off + key on within one sample restarts the note
	if (op->keyoff_pending)
	{
		op_clock_keystate(chip, op, 0);
		op->keyoff_pending = 0;
	}
	op_clock_keystate(chip, op, (UINT32)(op->keyon_live != 0));
	op->keyon_live &= ~(1 << KEYON_CSM);

	// we're active until we're quiet after the release
	return (op->env_state != EG_RELEASE || op->env_attenuation < EG_QUIET);
}

// clock the SSG-EG state; should only be called if SSG-EG is enabled
static void op_clock_ssg_eg_state(const OPN_STATE* chip, FM_OPERATOR* op)
{
	UINT32 mode;

	// work only happens once the attenuation crosses above 0x200
	if (! bitfield(op->env_attenuation, 9, 1))
		return;

	// 8 SSG-EG modes:
	//    000: repeat normally
	//    001: run once, hold low
	//    010: repeat, alternating between inverted/non-inverted
	//    011: run once, hold high
	//    100: inverted repeat normally
	//    101: inverted run once, hold low
	//    110: inverted repeat, alternating between inverted/non-inverted
	//    111: inverted run once, hold high
	mode = op_ssg_eg_mode(op->opoffs);

	// hold modes (1/3/5/7)
	if (bitfield(mode, 0, 1))
	{
		// set the inverted flag to the end state (0 for modes 1/7, 1 for modes 3/5)
		op->ssg_inverted = (UINT8)(bitfield(mode, 2, 1) ^ bitfield(mode, 1, 1));

		// if holding, force the attenuation to the expected value once we're
		// past the attack phase
		if (op->env_state != EG_ATTACK)
			op->env_attenuation = op->ssg_inverted ? 0x200 : 0x3ff;
	}
	// continuous modes (0/2/4/6)
	else
	{
		// toggle invert in alternating mode (even in attack state)
		op->ssg_inverted ^= bitfield(mode, 1, 1);

		// restart attack if in decay/sustain states
		if (op->env_state == EG_DECAY || op->env_state == EG_SUSTAIN)
			op_start_attack(chip, op, 1);

		// phase is reset to 0 in modes 0/4
		if (bitfield(mode, 1, 1) == 0)
			op->phase = 0;
	}

	// in all modes, once we hit release state, attenuation is forced to maximum
	if (op->env_state == EG_RELEASE)
		op->env_attenuation = 0x3ff;
	return;
}

// clock the envelope state according to the given count
static void op_clock_envelope(const OPN_STATE* chip, FM_OPERATOR* op, UINT32 env_counter)
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
		// non-SSG-EG cases just apply the increment
		if (! op_ssg_eg_enable(op->opoffs))
			op->env_attenuation = (UINT16)(op->env_attenuation + increment);
		// SSG-EG only applies if less than mid-point, and then at 4x
		else if (op->env_attenuation < 0x200)
			op->env_attenuation = (UINT16)(op->env_attenuation + 4 * increment);

		// clamp the final attenuation
		if (op->env_attenuation >= 0x400)
			op->env_attenuation = 0x3ff;
	}
	return;
}

// clock the 10.10 phase value
static void op_clock_phase(const OPN_STATE* chip, FM_OPERATOR* op, INT32 lfo_raw_pm)
{
	// read from the cache, or recalculate if PM active
	UINT32 phase_step = op->cache.phase_step;
	if (phase_step == PHASE_STEP_DYNAMIC)
		phase_step = compute_phase_step(chip, op->choffs, &op->cache, lfo_raw_pm);

	// finally apply the step to the current phase value
	op->phase += phase_step;
	return;
}

// master clocking function
static void op_clock(const OPN_STATE* chip, FM_OPERATOR* op, UINT32 env_counter, INT32 lfo_raw_pm)
{
	// clock the SSG-EG state
	if (op_ssg_eg_enable(op->opoffs))
		op_clock_ssg_eg_state(chip, op);
	else
		op->ssg_inverted = 0;

	// clock the envelope if on an envelope cycle; env_counter is a x.2 value
	if (bitfield(env_counter, 0, 2) == 0)
		op_clock_envelope(chip, op, env_counter >> 2);

	// clock the phase
	op_clock_phase(chip, op, lfo_raw_pm);
	return;
}

// return the effective attenuation of the envelope
static UINT32 op_envelope_attenuation(const OPN_STATE* chip, const FM_OPERATOR* op, UINT32 am_offset)
{
	UINT32 result = op->env_attenuation;

	// invert if necessary due to SSG-EG
	if (op->ssg_inverted)
		result = (0x200 - result) & 0x3ff;

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
static INT32 op_compute_volume(const OPN_STATE* chip, const FM_OPERATOR* op, UINT32 phase, UINT32 am_offset)
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

// return the current phase value
#define op_phase(op)	((op)->phase >> 10)


//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

// signal key on/off to the channel's operators
static void channel_keyonoff(OPN_STATE* chip, FM_CHANNEL* ch, UINT32 states, UINT8 type)
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
static UINT8 channel_prepare(OPN_STATE* chip, FM_CHANNEL* ch)
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
static void channel_clock(OPN_STATE* chip, FM_CHANNEL* ch, UINT32 env_counter, INT32 lfo_raw_pm)
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
static void channel_output_4op(OPN_STATE* chip, FM_CHANNEL* ch, INT32* output, UINT32 rshift, INT32 clipmax)
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

	// compute the 14-bit volume/value of operator 4; all algorithms consume OP4 output
	opmod = opout[bitfield(algorithm_ops, 4, 3)] >> 1;
	result = op_compute_volume(chip, op4, op_phase(op4) + opmod, am_offset);
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
static void engine_set_reset_status(OPN_STATE* chip, UINT8 set, UINT8 reset)
{
	chip->status = (chip->status | set) & ~(reset | STATUS_BUSY);
	return;
}

// update the state of the given timer
static void engine_update_timer(OPN_STATE* chip, UINT32 tnum, UINT32 enable, INT32 delta_clocks)
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
static void engine_timer_expired(OPN_STATE* chip, UINT32 tnum)
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
		for (chnum = 0; chnum < chip->channels; chnum ++)
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

// expire timers that run out on this sample
static void engine_clock_timers(OPN_STATE* chip)
{
	UINT8 tnum;

	for (tnum = 0; tnum < 2; tnum ++)
	{
		if (chip->timer_count[tnum] > 0 && --chip->timer_count[tnum] == 0)
			engine_timer_expired(chip, tnum);
	}
	return;
}

// handle a mode register write
static void engine_mode_write(OPN_STATE* chip, UINT8 data)
{
	UINT8 reset_mask = 0;

	// mark all channels as modified
	chip->modified_channels = chip->all_channels;

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

// handle writes to the OPN registers
static void engine_write(OPN_STATE* chip, UINT16 regnum, UINT8 data)
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
	chip->modified_channels = chip->all_channels;

	// most writes are passive, consumed only when needed
	if (regs_write(chip, regnum, data, &keyon_channel, &keyon_opmask))
	{
		// handle writes to the keyon register(s)
		if (keyon_channel < chip->channels)
			channel_keyonoff(chip, &chip->channel[keyon_channel], keyon_opmask, KEYON_NORMAL);
	}
	return;
}

// iterate over the given channels, clocking them forward one step;
// returns the envelope counter, which also clocks ADPCM-A
static UINT32 engine_clock(OPN_STATE* chip, UINT32 chanmask)
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
		for (chnum = 0; chnum < chip->channels; chnum ++)
		{
			if (bitfield(chanmask, chnum, 1))
			{
				if (channel_prepare(chip, &chip->channel[chnum]))
					chip->active_channels |= 1 << chnum;
			}
		}

		// reset the modified channels and prepare count
		chip->modified_channels = chip->prepare_count = 0;
	}

	// increment by 1 and manually wrap when we reach the divide count (3)
	if (bitfield(++chip->env_counter, 0, 2) == 3)
		chip->env_counter += 4 - 3;

	// clock the LFO
	lfo_raw_pm = clock_noise_and_lfo(chip);

	// now update the state of all the channels and operators
	for (chnum = 0; chnum < chip->channels; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1))
			channel_clock(chip, &chip->channel[chnum], chip->env_counter, lfo_raw_pm);
	}
	return chip->env_counter;
}

// compute a sum over the given channels; every active channel is computed
// so that its feedback stays current, but muted ones are not added
static void engine_output(OPN_STATE* chip, INT32* output, UINT32 rshift, INT32 clipmax, UINT32 chanmask)
{
	INT32 temp[2];
	UINT8 chnum;

	// mask out inactive channels
	chanmask &= chip->active_channels;

	// sum over all the desired channels
	for (chnum = 0; chnum < chip->channels; chnum ++)
	{
		if (! bitfield(chanmask, chnum, 1))
			continue;
		temp[0] = temp[1] = 0;
		channel_output_4op(chip, &chip->channel[chnum], temp, rshift, clipmax);
		if ((chip->muteMask >> chnum) & 0x01)
			continue;
		output[0] += temp[0];
		output[1] += temp[1];
	}
	return;
}

static void engine_reset(OPN_STATE* chip)
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
	for (curChn = 0; curChn < chip->channels; curChn ++)
	{
		chip->channel[curChn].feedback[0] = chip->channel[curChn].feedback[1] = 0;
		chip->channel[curChn].feedback_in = 0;
	}

	// reset the operators
	for (curOp = 0; curOp < chip->channels * 4; curOp ++)
		op_reset(&chip->op[curOp]);
	return;
}

static void engine_init(OPN_STATE* chip)
{
	UINT32 index;
	UINT8 curChn;
	UINT8 curOp;

	// create the waveforms
	for (index = 0; index < WAVEFORM_LENGTH; index ++)
		chip->waveform[index] = (UINT16)(abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	chip->all_channels = (1 << chip->channels) - 1;
	chip->active_channels = chip->all_channels;
	chip->modified_channels = chip->all_channels;

	// create the operators; for the first 3 channels the OPN and OPNA
	// offsets are the same
	for (curOp = 0; curOp < chip->channels * 4; curOp ++)
	{
		chip->op[curOp].opoffs = (curOp % 12) + ((curOp % 12) / 3) + 0x100 * (curOp / 12);
		op_reset(&chip->op[curOp]);
	}

	// create the channels and assign their operators; the channel index order is
	// 0,2,1,3, because the order in the map is: carrier 1, carrier 2, modulator 1, modulator 2
	for (curChn = 0; curChn < chip->channels; curChn ++)
	{
		FM_CHANNEL* ch = &chip->channel[curChn];
		UINT8 base = (curChn / 3) * 12 + (curChn % 3);

		ch->choffs = (curChn % 3) + 0x100 * (curChn / 3);
		ch->op[0] = base + 0;
		ch->op[1] = base + 6;
		ch->op[2] = base + 3;
		ch->op[3] = base + 9;
		for (curOp = 0; curOp < 4; curOp ++)
			chip->op[ch->op[curOp]].choffs = ch->choffs;
	}
	return;
}


//----------------------------------------------------------------------------
// ADPCM-A
//----------------------------------------------------------------------------

#define ADPCMA_REG(offset)	(chip->adpcm_a_regs[offset])

#define adpcma_total_level()		bitfield(ADPCMA_REG(0x01), 0, 6)
#define adpcma_ch_pan_left(ch)		bitfield(ADPCMA_REG((ch) + 0x08), 7, 1)
#define adpcma_ch_pan_right(ch)		bitfield(ADPCMA_REG((ch) + 0x08), 6, 1)
#define adpcma_ch_instrument_level(ch)	bitfield(ADPCMA_REG((ch) + 0x08), 0, 5)
#define adpcma_ch_start(ch)	((UINT32)(ADPCMA_REG((ch) + 0x10) | (ADPCMA_REG((ch) + 0x18) << 8)))
#define adpcma_ch_end(ch)	((UINT32)(ADPCMA_REG((ch) + 0x20) | (ADPCMA_REG((ch) + 0x28) << 8)))

static UINT8 adpcm_a_mem_read(const OPN_STATE* chip, UINT32 address)
{
	if (address < chip->adpcm_a_size)
		return chip->adpcm_a_data[address];
	return 0x00;
}

static void adpcm_a_set_start_end(OPN_STATE* chip, UINT8 chnum, UINT16 start, UINT16 end)
{
	ADPCMA_REG(chnum + 0x10) = (UINT8)start;
	ADPCMA_REG(chnum + 0x18) = (UINT8)(start >> 8);
	ADPCMA_REG(chnum + 0x20) = (UINT8)end;
	ADPCMA_REG(chnum + 0x28) = (UINT8)(end >> 8);
	return;
}

static void adpcm_a_reset(OPN_STATE* chip)
{
	UINT8 chnum;

	memset(chip->adpcm_a_regs, 0x00, ADPCM_A_REGISTERS);

	// initialize the pans to on by default, and max instrument volume;
	// some neogeo homebrews (for example ffeast) rely on this
	ADPCMA_REG(0x08) = ADPCMA_REG(0x09) = ADPCMA_REG(0x0a) =
	ADPCMA_REG(0x0b) = ADPCMA_REG(0x0c) = ADPCMA_REG(0x0d) = 0xdf;

	for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
		memset(&chip->adpcm_a[chnum], 0x00, sizeof(ADPCM_A_CHANNEL));
	return;
}

// signal key on/off
static void adpcm_a_keyonoff(OPN_STATE* chip, UINT8 chnum, UINT8 on)
{
	ADPCM_A_CHANNEL* ch = &chip->adpcm_a[chnum];

	ch->playing = on;
	if (ch->playing)
	{
		ch->curaddress = adpcma_ch_start(chnum) << chip->adpcm_a_shift;
		ch->curnibble = 0;
		ch->curbyte = 0;
		ch->accumulator = 0;
		ch->step_index = 0;
	}
	return;
}

// clock one channel; returns 1 when the sample just ended
static UINT8 adpcm_a_channel_clock(OPN_STATE* chip, UINT8 chnum)
{
	ADPCM_A_CHANNEL* ch = &chip->adpcm_a[chnum];
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
		UINT32 end = (adpcma_ch_end(chnum) + 1) << chip->adpcm_a_shift;
		if (((ch->curaddress ^ end) & 0xfffff) == 0)
		{
			ch->playing = 0;
			ch->accumulator = 0;
			return 1;
		}

		ch->curbyte = adpcm_a_mem_read(chip, ch->curaddress++);
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

// clock the given channels; returns a bitmask of the samples that ended
static UINT32 adpcm_a_clock(OPN_STATE* chip, UINT32 chanmask)
{
	UINT32 result = 0;
	UINT8 chnum;

	for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1))
		{
			if (adpcm_a_channel_clock(chip, chnum))
				result |= 1 << chnum;
		}
	}
	return result;
}

// add the output of all channels, with panning applied
static void adpcm_a_output(OPN_STATE* chip, INT32* output)
{
	UINT8 chnum;
	int vol;
	INT8 mul;
	UINT8 shift;
	INT16 value;

	// without sample memory there is nothing to play
	if (chip->adpcm_a_size == 0)
		return;

	for (chnum = 0; chnum < ADPCM_A_CHANNELS; chnum ++)
	{
		if ((chip->muteMask >> (MUTE_ADPCM_A + chnum)) & 0x01)
			continue;

		// volume combines instrument and total levels
		vol = (int)((adpcma_ch_instrument_level(chnum) ^ 0x1f) + (adpcma_total_level() ^ 0x3f));

		// if combined is maximum, don't add to outputs
		if (vol >= 63)
			continue;

		// convert into a shift and a multiplier
		mul = (INT8)(15 - (vol & 7));
		shift = (UINT8)(4 + 1 + (vol >> 3));

		// accumulator is a 12-bit value; shift up to sign-extend;
		// the downshift is incorporated into 'shift'
		value = (INT16)((((INT16)(chip->adpcm_a[chnum].accumulator << 4)) * mul >> shift) & ~3);

		// apply to left/right as appropriate
		if (adpcma_ch_pan_left(chnum))
			output[0] += value;
		if (adpcma_ch_pan_right(chnum))
			output[1] += value;
	}
	return;
}

// handle writes to the ADPCM-A registers
static void adpcm_a_write(OPN_STATE* chip, UINT32 regnum, UINT8 data)
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
				adpcm_a_keyonoff(chip, chnum, (UINT8)bitfield(~data, 7, 1));
		}
	}
	return;
}


//----------------------------------------------------------------------------
// ADPCM-B
//----------------------------------------------------------------------------

#define ADPCM_REG(offset)	(chip->adpcm_b.regdata[offset])

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

static UINT8 adpcm_b_mem_read(const OPN_STATE* chip, UINT32 address)
{
	if (address < chip->adpcm_b_mem_size)
		return chip->adpcm_b_mem[address];
	return 0x00;
}

static void adpcm_b_mem_write(OPN_STATE* chip, UINT32 address, UINT8 data)
{
	if (address < chip->adpcm_b_mem_size)
		chip->adpcm_b_mem[address] = data;
	return;
}

// reset the channel state
static void adpcm_b_channel_reset(OPN_STATE* chip)
{
	ADPCM_B_CHANNEL* ch = &chip->adpcm_b;

	ch->status = ADPCM_B_STATUS_BRDY;
	ch->curnibble = 0;
	ch->curbyte = 0;
	ch->dummy_read = 0;
	ch->position = 0;
	ch->curaddress = 0;
	ch->accumulator = 0;
	ch->prev_accum = 0;
	ch->adpcm_step = ADPCM_B_STEP_MIN;
	return;
}

// reset the registers and the channel
static void adpcm_b_reset(OPN_STATE* chip)
{
	memset(chip->adpcm_b.regdata, 0x00, ADPCM_B_REGISTERS);

	// default limit to wide open
	ADPCM_REG(0x0c) = ADPCM_REG(0x0d) = 0xff;

	adpcm_b_channel_reset(chip);
	return;
}

// compute the current address shift amount based on register settings
static UINT32 adpcm_b_address_shift(const OPN_STATE* chip)
{
	// if a constant address shift, just provide that
	if (chip->adpcm_b_shift != 0)
		return chip->adpcm_b_shift;

	// if ROM or 8-bit DRAM, shift is 5 bits
	if (adpcm_rom_ram())
		return 5;
	if (adpcm_dram_8bit())
		return 5;

	// otherwise, shift is 2 bits
	return 2;
}

// limit checker; stops at the last byte of the chunk described by address_shift()
static UINT8 adpcm_b_at_limit(const OPN_STATE* chip)
{
	return (chip->adpcm_b.curaddress == (((adpcm_limit() + 1) << adpcm_b_address_shift(chip)) - 1));
}

// end checker; stops at the last byte of the chunk described by address_shift()
static UINT8 adpcm_b_at_end(const OPN_STATE* chip)
{
	return (chip->adpcm_b.curaddress == (((adpcm_end() + 1) << adpcm_b_address_shift(chip)) - 1));
}

// load the start address and initialize the state
static void adpcm_b_load_start(OPN_STATE* chip)
{
	ADPCM_B_CHANNEL* ch = &chip->adpcm_b;

	ch->status = (ch->status & ~ADPCM_B_STATUS_EOS) | ADPCM_B_STATUS_PLAYING;
	ch->curaddress = adpcm_external() ? (adpcm_start() << adpcm_b_address_shift(chip)) : 0;
	ch->curnibble = 0;
	ch->curbyte = 0;
	ch->position = 0;
	ch->accumulator = 0;
	ch->prev_accum = 0;
	ch->adpcm_step = ADPCM_B_STEP_MIN;
	return;
}

// master clocking function
static void adpcm_b_clock(OPN_STATE* chip)
{
	ADPCM_B_CHANNEL* ch = &chip->adpcm_b;
	UINT32 position;
	UINT8 data;
	INT32 delta;

	// only process if active and not recording (which we don't support)
	if (! adpcm_execute() || adpcm_record() || (ch->status & ADPCM_B_STATUS_PLAYING) == 0)
	{
		ch->status &= ~ADPCM_B_STATUS_PLAYING;
		return;
	}

	// otherwise, advance the step
	position = ch->position + adpcm_delta_n();
	ch->position = (UINT16)position;
	if (position < 0x10000)
		return;

	// if we're about to process nibble 0, fetch sample
	if (ch->curnibble == 0)
	{
		// playing from RAM/ROM
		if (adpcm_external())
			ch->curbyte = adpcm_b_mem_read(chip, ch->curaddress);
	}

	// extract the nibble from our current byte
	data = (UINT8)((UINT8)(ch->curbyte << (4 * ch->curnibble)) >> 4);
	ch->curnibble ^= 1;

	// we just processed the last nibble
	if (ch->curnibble == 0)
	{
		// if playing from RAM/ROM, check the end/limit address or advance
		if (adpcm_external())
		{
			// handle the sample end, either repeating or stopping
			if (adpcm_b_at_end(chip))
			{
				// if repeating, go back to the start
				if (adpcm_repeat())
				{
					adpcm_b_load_start(chip);
				}
				// otherwise, done; set the EOS bit
				else
				{
					ch->accumulator = 0;
					ch->prev_accum = 0;
					ch->status = (ch->status & ~ADPCM_B_STATUS_PLAYING) | ADPCM_B_STATUS_EOS;
					return;
				}
			}
			// wrap at the limit address
			else if (adpcm_b_at_limit(chip))
			{
				ch->curaddress = 0;
			}
			// otherwise, advance the current address
			else
			{
				ch->curaddress ++;
				ch->curaddress &= 0xffffff;
			}
		}
		// if CPU-driven, copy the next byte and request more
		else
		{
			ch->curbyte = adpcm_cpudata();
			ch->status |= ADPCM_B_STATUS_BRDY;
		}
	}

	// remember previous value for interpolation
	ch->prev_accum = ch->accumulator;

	// forecast to next forecast: 1/8, 3/8, 5/8, 7/8, 9/8, 11/8, 13/8, 15/8
	delta = (INT32)(2 * bitfield(data, 0, 3) + 1) * ch->adpcm_step / 8;
	if (bitfield(data, 3, 1))
		delta = -delta;

	// add and clamp to 16 bits
	ch->accumulator = clamp(ch->accumulator + delta, -32768, 32767);

	// scale the ADPCM step
	ch->adpcm_step = clamp((ch->adpcm_step * s_adpcm_b_step_scale[bitfield(data, 0, 3)]) / 64,
		ADPCM_B_STEP_MIN, ADPCM_B_STEP_MAX);
	return;
}

// add the computed output value, with panning applied
static void adpcm_b_output(OPN_STATE* chip, INT32* output, UINT32 rshift)
{
	const ADPCM_B_CHANNEL* ch = &chip->adpcm_b;
	INT32 result;

	if ((chip->muteMask >> MUTE_ADPCM_B) & 0x01)
		return;

	// do a linear interpolation between samples
	result = (ch->prev_accum * (INT32)((ch->position ^ 0xffff) + 1) + ch->accumulator * (INT32)ch->position) >> 16;

	// apply volume (level) in a linear fashion and reduce
	result = (result * (INT32)adpcm_level()) >> (8 + rshift);

	// apply to left/right
	if (adpcm_pan_left())
		output[0] += result;
	if (adpcm_pan_right())
		output[1] += result;
	return;
}

// handle special register reads
static UINT8 adpcm_b_read(OPN_STATE* chip, UINT32 regnum)
{
	ADPCM_B_CHANNEL* ch = &chip->adpcm_b;
	UINT8 result = 0;

	// register 8 reads over the bus under some conditions
	if (regnum == 0x08 && ! adpcm_execute() && ! adpcm_record() && adpcm_external())
	{
		// two dummy reads are consumed first
		if (ch->dummy_read != 0)
		{
			adpcm_b_load_start(chip);
			ch->dummy_read --;
		}
		// read the data
		else
		{
			// read from outside of the chip
			result = adpcm_b_mem_read(chip, ch->curaddress++);

			// did we hit the end? if so, signal EOS
			if (adpcm_b_at_end(chip))
				ch->status = ADPCM_B_STATUS_EOS | ADPCM_B_STATUS_BRDY;
			// signal ready
			else
				ch->status = ADPCM_B_STATUS_BRDY;

			// wrap at the limit address
			if (adpcm_b_at_limit(chip))
				ch->curaddress = 0;
		}
	}
	return result;
}

// handle writes to the ADPCM-B registers
static void adpcm_b_write(OPN_STATE* chip, UINT32 regnum, UINT8 value)
{
	ADPCM_B_CHANNEL* ch = &chip->adpcm_b;

	// store the raw value to the register array;
	// most writes are passive, consumed only when needed
	ADPCM_REG(regnum) = value;

	// register 0 can do a reset; also use writes here to reset the
	// dummy read counter
	if (regnum == 0x00)
	{
		if (adpcm_execute())
			adpcm_b_load_start(chip);
		else
			ch->status &= ~ADPCM_B_STATUS_EOS;
		if (adpcm_resetflag())
			adpcm_b_channel_reset(chip);
		if (adpcm_external())
			ch->dummy_read = 2;
	}
	// register 8 writes over the bus under some conditions
	else if (regnum == 0x08)
	{
		// if writing from the CPU during execute, clear the ready flag
		if (adpcm_execute() && ! adpcm_record() && ! adpcm_external())
		{
			ch->status &= ~ADPCM_B_STATUS_BRDY;
		}
		// if writing during "record", pass through as data
		else if (! adpcm_execute() && adpcm_record() && adpcm_external())
		{
			// clear out dummy reads and set start address
			if (ch->dummy_read != 0)
			{
				adpcm_b_load_start(chip);
				ch->dummy_read = 0;
			}

			// did we hit the end? if so, signal EOS
			if (adpcm_b_at_end(chip))
			{
				ch->status = ADPCM_B_STATUS_EOS | ADPCM_B_STATUS_BRDY;
			}
			// otherwise, write the data and signal ready
			else
			{
				adpcm_b_mem_write(chip, ch->curaddress++, value);
				ch->status = ADPCM_B_STATUS_BRDY;
			}
		}
	}
	return;
}


//----------------------------------------------------------------------------
// SSG link and prescaler
//----------------------------------------------------------------------------

static void ssg_write(OPN_STATE* chip, UINT8 offset, UINT8 data)
{
	if (chip->ssg.write != NULL)
		chip->ssg.write(chip->ssg.param, offset, data);
	return;
}

static UINT8 ssg_read(OPN_STATE* chip)
{
	if (chip->ssg.read != NULL)
		return chip->ssg.read(chip->ssg.param, 0);
	return 0x00;
}

// the SSG clock that the linked AY8910 needs; it divides by 8 internally
static UINT32 ssg_clock(const OPN_STATE* chip)
{
	UINT32 scale;

	if (chip->type == CHIP_YM2610)
		return chip->clock / 4;
	scale = chip->prescale * 2 / 3;	// integer, so prescale 2 gives 1
	if (chip->type == CHIP_YM2203)
		return chip->clock * 2 / scale;
	return chip->clock / scale;
}

// FM sample rate for the current prescale
static UINT32 fm_sample_rate(const OPN_STATE* chip)
{
	UINT32 operators = chip->channels * 4;
	return chip->clock / (operators * chip->prescale);
}

static void update_prescale(OPN_STATE* chip, UINT8 prescale)
{
	if (chip->prescale == prescale)
		return;
	chip->prescale = prescale;
	if (chip->ssg.set_clock != NULL)
		chip->ssg.set_clock(chip->ssg.param, ssg_clock(chip));
	if (chip->smpRateFunc != NULL)
		chip->smpRateFunc(chip->smpRateData, fm_sample_rate(chip));
	return;
}

// handle writes to the address port of the YM2203/YM2608
static void opn_write_address(OPN_STATE* chip, UINT8 data)
{
	// just set the address
	chip->address = data;

	// the SSG latches its own address
	if (data < 0x10)
		ssg_write(chip, 0, data);

	// special case: update the prescale
	if (chip->address >= 0x2d && chip->address <= 0x2f)
	{
		// 2D-2F: prescaler select
		if (chip->address == 0x2d)
			update_prescale(chip, 6);
		else if (chip->address == 0x2e && chip->prescale == 6)
			update_prescale(chip, 3);
		else if (chip->address == 0x2f)
			update_prescale(chip, 2);
	}
	return;
}


//----------------------------------------------------------------------------
// YM2203
//----------------------------------------------------------------------------

static void ym2203_write_data(OPN_STATE* chip, UINT8 data)
{
	if (chip->address < 0x10)
	{
		// 00-0F: write to SSG
		ssg_write(chip, 1, data);
	}
	else
	{
		// 10-FF: write to FM
		engine_write(chip, chip->address, data);
	}
	return;
}

static UINT8 ym2203_read_data(OPN_STATE* chip)
{
	// 00-0F: Read from SSG
	if (chip->address < 0x10)
		return ssg_read(chip);
	return 0x00;
}

// generate one sample of sound
static void ym2203_generate(OPN_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];

	engine_clock_timers(chip);

	// clock the system
	engine_clock(chip, chip->all_channels);

	// update the FM content; OPN is full 14-bit with no intermediate clipping
	output[0] = output[1] = 0;
	engine_output(chip, output, 0, 32767, chip->all_channels);

	// convert to 10.3 floating point value for the DAC and back;
	// the chip is mono, so both outputs carry the same signal
	*outL = *outR = roundtrip_fp(output[0]);
	return;
}


//----------------------------------------------------------------------------
// YM2608
//----------------------------------------------------------------------------

// read the extended status register, merging in the live ADPCM-B status
static UINT8 ym2608_read_status_hi(OPN_STATE* chip)
{
	UINT8 status;
	UINT32 adpcm_status;

	// fetch regular status
	status = chip->status & ~(STATUS_ADPCM_B_EOS | STATUS_ADPCM_B_BRDY | STATUS_ADPCM_B_PLAYING);

	// fetch ADPCM-B status, and merge in the bits
	adpcm_status = chip->adpcm_b.status;
	if ((adpcm_status & ADPCM_B_STATUS_EOS) != 0)
		status |= STATUS_ADPCM_B_EOS;
	if ((adpcm_status & ADPCM_B_STATUS_BRDY) != 0)
		status |= STATUS_ADPCM_B_BRDY;
	if ((adpcm_status & ADPCM_B_STATUS_PLAYING) != 0)
		status |= STATUS_ADPCM_B_PLAYING;

	// turn off any bits that have been requested to be masked
	status &= ~(chip->flag_control & 0x1f);

	// update the status so that IRQs are propagated
	engine_set_reset_status(chip, status, ~status);
	return status;
}

static UINT8 ym2608_read_data(OPN_STATE* chip)
{
	if (chip->address < 0x10)
		return ssg_read(chip);	// 00-0F: Read from SSG
	else if (chip->address == 0xff)
		return 1;	// FF: ID code
	return 0x00;
}

static UINT8 ym2608_read_data_hi(OPN_STATE* chip)
{
	// 00-0F: Read from ADPCM-B
	if ((chip->address & 0xff) < 0x10)
		return adpcm_b_read(chip, chip->address & 0x0f);
	return 0x00;
}

static void ym2608_write_data(OPN_STATE* chip, UINT8 data)
{
	// ignore if paired with upper address
	if (bitfield(chip->address, 8, 1))
		return;

	if (chip->address < 0x10)
	{
		// 00-0F: write to SSG
		ssg_write(chip, 1, data);
	}
	else if (chip->address < 0x20)
	{
		// 10-1F: write to ADPCM-A
		adpcm_a_write(chip, chip->address & 0x0f, data);
	}
	else if (chip->address == 0x29)
	{
		// 29: special IRQ mask register
		chip->irq_enable = data;
	}
	else
	{
		// 20-28, 2A-FF: write to FM
		engine_write(chip, chip->address, data);
	}
	return;
}

static void ym2608_write_data_hi(OPN_STATE* chip, UINT8 data)
{
	// ignore if paired with lower address
	if (! bitfield(chip->address, 8, 1))
		return;

	if (chip->address < 0x110)
	{
		// 100-10F: write to ADPCM-B
		adpcm_b_write(chip, chip->address & 0x0f, data);
	}
	else if (chip->address == 0x110)
	{
		// 110: IRQ flag control
		if (bitfield(data, 7, 1))
			engine_set_reset_status(chip, 0, 0xff);
		else
			chip->flag_control = data;
	}
	else
	{
		// 111-1FF: write to FM
		engine_write(chip, chip->address, data);
	}
	return;
}

// generate one sample of sound
static void ym2608_generate(OPN_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	UINT32 fmmask;
	UINT32 env_counter;

	engine_clock_timers(chip);

	// top bit of the IRQ enable flags controls 3-channel vs 6-channel mode
	fmmask = bitfield(chip->irq_enable, 7, 1) ? 0x3f : 0x07;

	// clock the system
	env_counter = engine_clock(chip, chip->all_channels);

	// clock the ADPCM-A engine on every envelope cycle
	// (channels 4 and 5 clock every 2 envelope clocks)
	if (bitfield(env_counter, 0, 2) == 0)
		adpcm_a_clock(chip, bitfield(env_counter, 2, 1) ? 0x0f : 0x3f);

	// clock the ADPCM-B engine every cycle
	adpcm_b_clock(chip);

	// update the FM content; OPNA is 13-bit with no intermediate clipping
	output[0] = output[1] = 0;
	engine_output(chip, output, 1, 32767, fmmask);

	// mix in the ADPCM and clamp; then double, to match the YM2203 level
	adpcm_a_output(chip, output);
	adpcm_b_output(chip, output, 1);
	*outL = clamp(output[0], -32768, 32767) * 2;
	*outR = clamp(output[1], -32768, 32767) * 2;
	return;
}


//----------------------------------------------------------------------------
// YM2610
//----------------------------------------------------------------------------

static UINT8 ym2610_read_data(OPN_STATE* chip)
{
	if (chip->address < 0x0e)
		return ssg_read(chip);	// 00-0D: Read from SSG
	else if (chip->address < 0x10)
		return 0xff;	// 0E-0F: I/O ports not supported
	else if (chip->address == 0xff)
		return 1;	// FF: ID code
	return 0x00;
}

static void ym2610_write_data(OPN_STATE* chip, UINT8 data)
{
	// ignore if paired with upper address
	if (bitfield(chip->address, 8, 1))
		return;

	if (chip->address < 0x0e)
	{
		// 00-0D: write to SSG
		ssg_write(chip, 1, data);
	}
	else if (chip->address < 0x10)
	{
		// 0E-0F: I/O ports not supported
	}
	else if (chip->address < 0x1c)
	{
		// 10-1B: write to ADPCM-B
		// YM2610 effectively forces external mode on, and disables recording
		if (chip->address == 0x10)
			data = (data | 0x20) & ~0x40;
		adpcm_b_write(chip, chip->address & 0x0f, data);
	}
	else if (chip->address == 0x1c)
	{
		// 1C: EOS flag reset
		chip->flag_mask = ~data & EOS_FLAGS_MASK;
		chip->eos_status &= ~(data & EOS_FLAGS_MASK);
	}
	else
	{
		// 1D-FF: write to FM
		engine_write(chip, chip->address, data);
	}
	return;
}

static void ym2610_write_data_hi(OPN_STATE* chip, UINT8 data)
{
	// ignore if paired with lower address
	if (! bitfield(chip->address, 8, 1))
		return;

	if (chip->address < 0x130)
	{
		// 100-12F: write to ADPCM-A
		adpcm_a_write(chip, chip->address & 0x3f, data);
	}
	else
	{
		// 130-1FF: write to FM
		engine_write(chip, chip->address, data);
	}
	return;
}

// generate one sample of sound
static void ym2610_generate(OPN_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	UINT32 env_counter;
	UINT8 live_eos;

	engine_clock_timers(chip);

	// clock the system
	env_counter = engine_clock(chip, chip->fm_mask);

	// clock the ADPCM-A engine on every envelope cycle
	if (bitfield(env_counter, 0, 2) == 0)
		chip->eos_status |= (UINT8)adpcm_a_clock(chip, 0x3f);

	// clock the ADPCM-B engine every cycle
	adpcm_b_clock(chip);

	// we track the last ADPCM-B EOS value in bit 6 (which is hidden from callers);
	// if it changed since the last sample, update the visible EOS state in bit 7
	live_eos = ((chip->adpcm_b.status & ADPCM_B_STATUS_EOS) != 0) ? 0x40 : 0x00;
	if (((live_eos ^ chip->eos_status) & 0x40) != 0)
		chip->eos_status = (chip->eos_status & ~0xc0) | live_eos | (live_eos << 1);

	// update the FM content; OPNB is 13-bit with no intermediate clipping
	output[0] = output[1] = 0;
	engine_output(chip, output, 1, 32767, chip->fm_mask);

	// mix in the ADPCM and clamp; then double, to match the YM2203 level
	adpcm_a_output(chip, output);
	adpcm_b_output(chip, output, 1);
	*outL = clamp(output[0], -32768, 32767) * 2;
	*outR = clamp(output[1], -32768, 32767) * 2;
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

// reset the chip, as ymfm's reset() does
static void opn_reset_chip(OPN_STATE* chip)
{
	engine_reset(chip);
	if (chip->ssg.reset != NULL)
		chip->ssg.reset(chip->ssg.param);
	if (chip->type == CHIP_YM2608)
	{
		adpcm_a_reset(chip);
		adpcm_b_reset(chip);

		// configure ADPCM percussion sounds; these are present in an embedded ROM
		adpcm_a_set_start_end(chip, 0, 0x0000, 0x01bf);	// bass drum
		adpcm_a_set_start_end(chip, 1, 0x01c0, 0x043f);	// snare drum
		adpcm_a_set_start_end(chip, 2, 0x0440, 0x1b7f);	// top cymbal
		adpcm_a_set_start_end(chip, 3, 0x1b80, 0x1cff);	// high hat
		adpcm_a_set_start_end(chip, 4, 0x1d00, 0x1f7f);	// tom tom
		adpcm_a_set_start_end(chip, 5, 0x1f80, 0x1fff);	// rim shot

		// initialize our special interrupt states, then read the upper status
		// register, which updates the IRQs
		chip->irq_enable = 0x1f;
		chip->flag_control = 0x1c;
		ym2608_read_status_hi(chip);
	}
	else if (chip->type == CHIP_YM2610)
	{
		adpcm_a_reset(chip);
		adpcm_b_reset(chip);

		// initialize our special interrupt states
		chip->eos_status = 0x00;
		chip->flag_mask = EOS_FLAGS_MASK;
	}
	return;
}

static void init_ssg_link(DEV_INFO* devInf, const DEV_GEN_CFG* cfg, UINT32 ssgClock, UINT8 ssgType)
{
	DEVLINK_INFO* devLink;
	AY8910_CFG* ayCfg;

	devInf->linkDevCount = 0;
	devInf->linkDevs = (DEVLINK_INFO*)calloc(1, sizeof(DEVLINK_INFO));
	ayCfg = (AY8910_CFG*)calloc(1, sizeof(AY8910_CFG));
	if (devInf->linkDevs == NULL || ayCfg == NULL)
	{
		free(devInf->linkDevs);
		free(ayCfg);
		devInf->linkDevs = NULL;
		return;
	}

	ayCfg->_genCfg = *cfg;
	ayCfg->_genCfg.clock = ssgClock;
	ayCfg->_genCfg.flags = 0x00;
	ayCfg->_genCfg.emuCore = 0;
	ayCfg->chipType = ssgType;
	ayCfg->chipFlags = YM2149_PIN26_HIGH;

	devLink = &devInf->linkDevs[0];
	devLink->devID = DEVID_AY8910;
	devLink->linkID = 0;
	devLink->cfg = (DEV_GEN_CFG*)ayCfg;
	devInf->linkDevCount = 1;
	return;
}

static UINT8 device_start_opn(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf, UINT8 type, DEV_DEF* devDef)
{
	OPN_STATE* chip;
	static const UINT8 ssgTypes[3] = {AYTYPE_YM2203, AYTYPE_YM2608, AYTYPE_YM2610};

	chip = (OPN_STATE*)calloc(1, sizeof(OPN_STATE));
	if (chip == NULL)
		return 0xFF;

	chip->type = type;
	chip->is_opna = (type != CHIP_YM2203);
	chip->channels = chip->is_opna ? 6 : 3;
	chip->clock = cfg->clock;
	chip->prescale = DEFAULT_PRESCALE;
	engine_init(chip);
	chip->fm_mask = chip->all_channels;
	if (type == CHIP_YM2608)
	{
		chip->adpcm_a_data = ym2608_rhythm_rom;
		chip->adpcm_a_size = YM2608_RHYTHM_ROM_SIZE;
	}
	else if (type == CHIP_YM2610)
	{
		// the YM2610 lacks FM channels 1 and 4
		chip->fm_mask = cfg->flags ? 0x3f : 0x36;
		chip->adpcm_a_shift = 8;
		chip->adpcm_b_shift = 8;
	}
	opn_reset_chip(chip);
	chip->muteMask = 0x00;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, fm_sample_rate(chip), devDef);
	init_ssg_link(retDevInf, cfg, ssg_clock(chip), ssgTypes[type]);
	return 0x00;
}

static UINT8 device_start_ym2203(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opn(cfg, retDevInf, CHIP_YM2203, &devDef2203);
}

static UINT8 device_start_ym2608(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opn(cfg, retDevInf, CHIP_YM2608, &devDef2608);
}

static UINT8 device_start_ym2610(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opn(cfg, retDevInf, CHIP_YM2610, cfg->flags ? &devDef2610B : &devDef2610);
}

static void device_stop_opn(void* info)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	free(chip->adpcm_a_mem);	// YM2610 ADPCM-A memory, or a replaced YM2608 rhythm ROM
	free(chip->adpcm_b_mem);
	free(chip);
	return;
}

static void device_reset_opn(void* info)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	// ymfm's reset leaves the address and the prescaler alone; a device reset
	// returns them to their power-on state too
	chip->address = 0;
	update_prescale(chip, DEFAULT_PRESCALE);
	opn_reset_chip(chip);
	return;
}

static void ym2203_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	OPN_STATE* chip = (OPN_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2203_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2608_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	OPN_STATE* chip = (OPN_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2608_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2610_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	OPN_STATE* chip = (OPN_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2610_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2203_w(void* info, UINT8 offset, UINT8 data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	if (! (offset & 1))
		opn_write_address(chip, data);	// address port
	else
		ym2203_write_data(chip, data);	// data port
	return;
}

static UINT8 ym2203_r(void* info, UINT8 offset)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	if (! (offset & 1))
		return chip->status;	// status port
	return ym2203_read_data(chip);	// data port (only SSG)
}

static void ym2608_w(void* info, UINT8 offset, UINT8 data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	switch(offset & 3)
	{
	case 0:	// address port
		opn_write_address(chip, data);
		break;
	case 1:	// data port
		ym2608_write_data(chip, data);
		break;
	case 2:	// upper address port
		chip->address = 0x100 | data;
		break;
	case 3:	// upper data port
		ym2608_write_data_hi(chip, data);
		break;
	}
	return;
}

static UINT8 ym2608_r(void* info, UINT8 offset)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	switch(offset & 3)
	{
	case 0:	// status port, YM2203 compatible
		return chip->status & (STATUS_TIMERA | STATUS_TIMERB);
	case 1:	// data port (only SSG)
		return ym2608_read_data(chip);
	case 2:	// status port, extended
		return ym2608_read_status_hi(chip);
	default:	// ADPCM-B data
		return ym2608_read_data_hi(chip);
	}
}

static void ym2610_w(void* info, UINT8 offset, UINT8 data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	switch(offset & 3)
	{
	case 0:	// address port
		chip->address = data;
		if (data < 0x10)
			ssg_write(chip, 0, data);
		break;
	case 1:	// data port
		ym2610_write_data(chip, data);
		break;
	case 2:	// upper address port
		chip->address = 0x100 | data;
		break;
	case 3:	// upper data port
		ym2610_write_data_hi(chip, data);
		break;
	}
	return;
}

static UINT8 ym2610_r(void* info, UINT8 offset)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	switch(offset & 3)
	{
	case 0:	// status port, YM2203 compatible
		return chip->status & (STATUS_TIMERA | STATUS_TIMERB);
	case 1:	// data port (only SSG)
		return ym2610_read_data(chip);
	case 2:	// status port, extended
		return chip->eos_status & chip->flag_mask;
	default:	// ADPCM-B data
		return 0x00;
	}
}

static void opn_alloc_mem(UINT8** memPtr, UINT32* memSize, UINT32 newSize)
{
	UINT8* mem;

	if (*memSize == newSize)
		return;
	mem = (UINT8*)realloc(*memPtr, newSize);
	if (mem == NULL && newSize > 0)
		return;
	*memPtr = mem;
	*memSize = newSize;
	if (newSize > 0)
		memset(mem, 0xFF, newSize);
	return;
}

static void opn_write_mem(UINT8* mem, UINT32 memSize, UINT32 offset, UINT32 length, const UINT8* data)
{
	if (offset > memSize)
		return;
	if (offset + length > memSize)
		length = memSize - offset;
	memcpy(mem + offset, data, length);
	return;
}

static void opn_alloc_pcmroma(void* info, UINT32 memsize)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	opn_alloc_mem(&chip->adpcm_a_mem, &chip->adpcm_a_mem_size, memsize);
	chip->adpcm_a_data = chip->adpcm_a_mem;
	chip->adpcm_a_size = chip->adpcm_a_mem_size;
	return;
}

static void opn_write_pcmroma(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	opn_write_mem(chip->adpcm_a_mem, chip->adpcm_a_mem_size, offset, length, data);
	return;
}

static void opn_alloc_pcmromb(void* info, UINT32 memsize)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	opn_alloc_mem(&chip->adpcm_b_mem, &chip->adpcm_b_mem_size, memsize);
	return;
}

static void opn_write_pcmromb(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	opn_write_mem(chip->adpcm_b_mem, chip->adpcm_b_mem_size, offset, length, data);
	return;
}

// replace (part of) the YM2608 rhythm ROM; the rest keeps the built-in sounds
static void ym2608_write_rhythmrom(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	if (chip->adpcm_a_mem == NULL)
	{
		chip->adpcm_a_mem = (UINT8*)malloc(YM2608_RHYTHM_ROM_SIZE);
		if (chip->adpcm_a_mem == NULL)
			return;
		chip->adpcm_a_mem_size = YM2608_RHYTHM_ROM_SIZE;
		memcpy(chip->adpcm_a_mem, ym2608_rhythm_rom, YM2608_RHYTHM_ROM_SIZE);
		chip->adpcm_a_data = chip->adpcm_a_mem;
		chip->adpcm_a_size = chip->adpcm_a_mem_size;
	}
	opn_write_mem(chip->adpcm_a_mem, chip->adpcm_a_mem_size, offset, length, data);
	return;
}

static void opn_set_mute_mask(void* info, UINT32 MuteMask)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	chip->muteMask = MuteMask;
	return;
}

static void opn_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	chip->smpRateFunc = CallbackFunc;
	chip->smpRateData = DataPtr;
	return;
}

static UINT8 opn_link_device(void* info, UINT8 linkID, const DEV_INFO* devInfLink)
{
	OPN_STATE* chip = (OPN_STATE*)info;
	SSG_FUNCS ssg;
	UINT8 retVal;

	if (linkID != 0)
		return EERR_UNK_DEVICE;

	memset(&chip->ssg, 0x00, sizeof(SSG_FUNCS));
	if (devInfLink == NULL)
		return 0x00;

	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&ssg.write);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, (void**)&ssg.read);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_CLOCK | RWF_WRITE, DEVRW_VALUE, 0, (void**)&ssg.set_clock);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	ssg.reset = devInfLink->devDef->Reset;
	ssg.param = devInfLink->dataPtr;
	chip->ssg = ssg;

	// the SSG may be linked after a prescaler change
	chip->ssg.set_clock(chip->ssg.param, ssg_clock(chip));
	return 0x00;
}
