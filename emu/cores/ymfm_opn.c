/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    Yamaha OPN family

    C port of ymfm's OPN family (ymfm_opn.h, ymfm_opn.cpp).
    https://github.com/aaronsgiles/ymfm

    Chips: YM2203 (OPN), YM2608 (OPNA), YM2610 / YM2610B (OPNB) and
    YM2612 (OPN2).

    Port notes:
    - As in ymfm, all chips share one register implementation
      (OPN_REGISTERS, ymfm's opn_registers_base). The YM2203 uses the OPN
      layout (3 channels, mono, no LFO); the others use the OPNA layout
      (6 channels in 2 banks, stereo, LFO). ymfm's IsOpnA template flag
      becomes the is_opna field, and each layout has its own FM_FAMILY
      table for the shared FM engine (ymfm_fm.c).
    - The ADPCM-A and ADPCM-B channels are the shared engines of
      ymfm_adpcm.c.
    - The SSG part is not emulated here. As in libvgm's other OPN cores,
      it is a linked AY8910 device: SSG register accesses are forwarded
      to it, and prescaler changes update its clock.
    - One update sample is one FM sample (clock / (12 * prescale) on the
      YM2203, clock / (24 * prescale) on the YM2608, clock / 144 on the
      YM2610 and the YM2612). A prescaler change changes the sample rate
      and calls the sample rate change callback.
    - The busy flag and the IRQ line are not emulated.
    - YM2608/YM2610: ymfm mixes FM and ADPCM at 13 bits, 6 dB below the
      YM2203. The output is doubled after ymfm's mix, so the level and the
      balance with the linked SSG match libvgm's other OPN cores.
    - YM2610: the ADPCM-A and ADPCM-B sample memories are filled by the
      player (DEVRW_MEMSIZE, DEVRW_BLOCK with 'A'/'B'). YM2608: the ADPCM-B
      memory is filled the same way ('B'). Reads outside a memory return 0.
    - YM2608: Yamaha's rhythm (ADPCM-A) ROM is not included. The rhythm
      channels play the Tetorica OPNA rhythm sounds instead (BSD-3, see
      ymfm_opn_rhythm.h): the same layout and encoding, but different
      sounds. The player can replace them with another 8 KB rhythm ROM
      (DEVRW_BLOCK with 0x524F, 'RO', as for the YMF278B ROM).
    - YM2610: the ADPCM-A channels are silent while no ADPCM-A memory is
      loaded (see ymfm_adpcm.c).
    - YM2612: the YM3438 (OPN2C) and YMF276 (OPN2L) output stages are not
      included: all variants use the YM2612 mix with the DAC
      discontinuity.
    - The key-on fix of the shared engine applies (see ymfm_fm.c).

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

#include "ymfm_fm.h"
#include "ymfm_fm_impl.h"
#include "ymfm_adpcm.h"
#include "ymfm_opn.h"
#include "ymfm_opn_rhythm.h"


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

static void ym2612_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym2612(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ym2612(void* info);
static void device_reset_ym2612(void* info);
static void ym2612_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2612_r(void* info, UINT8 offset);
static void ym2612_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

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

static DEVDEF_RWFUNC devFunc2612[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2612_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2612_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ym2612_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef2612 =
{
	"YM2612", "ymfm", FCC_YMFM,

	device_start_ym2612,
	device_stop_ym2612,
	device_reset_ym2612,
	ym2612_update,

	NULL,	// SetOptionBits
	ym2612_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc2612,	// rwFuncs
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

static const char* DeviceName_YM2612(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && devCfg->flags)
		return "YM3438";
	return "YM2612";
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

#define DEV_CHN_COUNT_YM2612	7
static UINT16 DeviceChannels_YM2612(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_YM2612;
}

static const char** DeviceChannelNames_YM2612(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_YM2612] =
	{
		"FM 1", "FM 2", "FM 3", "FM 4", "FM 5", "FM 6",
		"DAC",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs_OPN(const DEV_GEN_CFG* devCfg)
{
	static const DEVLINK_IDS dlIDs = {1, {DEVID_AY8910}};
	return &dlIDs;
}

static const DEVLINK_IDS* DeviceLinkIDs_YM2612(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YM2203 =
{
	DEVID_YM2203,
	DeviceName_YM2203,
	DeviceChannels_YM2203,
	DeviceChannelNames_YM2203,
	DeviceLinkIDs_OPN,
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
	DeviceLinkIDs_OPN,
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
	DeviceLinkIDs_OPN,
	{	// cores
		&devDef2610,
		NULL
	}
};

const DEV_DECL sndDev_YM2612 =
{
	DEVID_YM2612,
	DeviceName_YM2612,
	DeviceChannels_YM2612,
	DeviceChannelNames_YM2612,
	DeviceLinkIDs_YM2612,
	{	// cores
		&devDef2612,
		NULL
	}
};


//----------------------------------------------------------------------------
// chip state
//----------------------------------------------------------------------------

#define CHIP_YM2203		0
#define CHIP_YM2608		1
#define CHIP_YM2610		2

#define REGISTERS		0x200
#define REG_MODE		0x27
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

// the OPN registers (ymfm's opn_registers_base); the FM engine gets a
// pointer to this as its owner
typedef struct _opn_registers
{
	UINT8 is_opna;			// OPNA layout (YM2608, YM2610, YM2612)
	UINT32 lfo_counter;		// LFO counter
	UINT8 regdata[REGISTERS];	// register data
	UINT16 waveform[FM_WAVEFORM_LENGTH];	// sine waveform
} OPN_REGISTERS;

// functions of the linked SSG (AY8910) device
typedef struct _ssg_funcs
{
	DEVFUNC_WRITE_A8D8 write;
	DEVFUNC_READ_A8D8 read;
	DEVFUNC_WRITE_CLOCK set_clock;
	DEVFUNC_CTRL reset;
	void* param;
} SSG_FUNCS;

// YM2203, YM2608, YM2610
typedef struct _opn_state
{
	DEV_DATA _devData;

	UINT8 type;				// CHIP_YM2203, CHIP_YM2608 or CHIP_YM2610
	UINT32 clock;			// input clock
	UINT32 fm_mask;			// mask of FM channels that are clocked

	FM_ENGINE fm;
	OPN_REGISTERS regs;

	// chip
	UINT16 address;			// address register
	UINT8 irq_enable;		// YM2608: IRQ enable register
	UINT8 flag_control;		// YM2608: flag control register
	UINT8 eos_status;		// YM2610: end-of-sample signals
	UINT8 flag_mask;		// YM2610: flag mask control

	ADPCM_A_ENGINE adpcm_a;
	UINT8* adpcm_a_mem;		// YM2610: ADPCM-A sample memory; YM2608: a replaced rhythm ROM
	UINT32 adpcm_a_mem_size;
	ADPCM_B_ENGINE adpcm_b;
	UINT8* adpcm_b_mem;		// ADPCM-B sample memory
	UINT32 adpcm_b_mem_size;

	SSG_FUNCS ssg;
	DEVCB_SRATE_CHG smpRateFunc;
	void* smpRateData;

	UINT32 muteMask;
} OPN_STATE;

// YM2612
typedef struct _ym2612_state
{
	DEV_DATA _devData;

	FM_ENGINE fm;
	OPN_REGISTERS regs;

	UINT16 address;			// address register
	UINT16 dac_data;		// 9-bit DAC data
	UINT8 dac_enable;		// DAC enabled?

	UINT32 muteMask;
} YM2612_STATE;


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

// LFO clock dividers, converted from the frequencies in the application manual
static const UINT8 s_lfo_max_count[8] = { 109, 78, 72, 68, 63, 45, 9, 6 };

#define REG(offset)		(regs->regdata[offset])

// system-wide registers
#define lfo_enable()		(regs->is_opna ? bitfield(REG(0x22), 3, 1) : 0)
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

// per-channel registers
#define ch_block_freq(choffs)	((bitfield(REG(0xa4 + (choffs)), 0, 6) << 8) | bitfield(REG(0xa0 + (choffs)), 0, 8))
#define ch_feedback(choffs)		bitfield(REG(0xb0 + (choffs)), 3, 3)
#define ch_algorithm(choffs)	bitfield(REG(0xb0 + (choffs)), 0, 3)
#define ch_output_0(choffs)		(regs->is_opna ? bitfield(REG(0xb4 + (choffs)), 7, 1) : 1)
#define ch_output_1(choffs)		(regs->is_opna ? bitfield(REG(0xb4 + (choffs)), 6, 1) : 0)
#define ch_lfo_am_sens(choffs)	(regs->is_opna ? bitfield(REG(0xb4 + (choffs)), 4, 2) : 0)
#define ch_lfo_pm_sens(choffs)	(regs->is_opna ? bitfield(REG(0xb4 + (choffs)), 0, 3) : 0)

// per-operator registers
#define op_detune(opoffs)		bitfield(REG(0x30 + (opoffs)), 4, 3)
#define op_multiple(opoffs)		bitfield(REG(0x30 + (opoffs)), 0, 4)
#define op_total_level(opoffs)	bitfield(REG(0x40 + (opoffs)), 0, 7)
#define op_ksr(opoffs)			bitfield(REG(0x50 + (opoffs)), 6, 2)
#define op_attack_rate(opoffs)	bitfield(REG(0x50 + (opoffs)), 0, 5)
#define op_decay_rate(opoffs)	bitfield(REG(0x60 + (opoffs)), 0, 5)
#define op_lfo_am_enable(opoffs)	(regs->is_opna ? bitfield(REG(0x60 + (opoffs)), 7, 1) : 0)
#define op_sustain_rate(opoffs)	bitfield(REG(0x70 + (opoffs)), 0, 5)
#define op_sustain_level(opoffs)	bitfield(REG(0x80 + (opoffs)), 4, 4)
#define op_release_rate(opoffs)	bitfield(REG(0x80 + (opoffs)), 0, 4)
#define op_ssg_eg(opoffs)		bitfield(REG(0x90 + (opoffs)), 0, 4)

// the OPNA has 2 register banks of 3 channels (the OPN only the first);
// the channel offset is the register offset of the bank plus the channel
// number in the bank
static UINT32 opn_channel_offset(UINT32 chnum)
{
	return (chnum % 3) + 0x100 * (chnum / 3);
}

// the operators of each bank are spaced 4 registers apart, with a gap
// every 3 operators
static UINT32 opn_operator_offset(UINT32 opnum)
{
	return (opnum % 12) + ((opnum % 12) / 3) + 0x100 * (opnum / 12);
}

// the order of the operators in a channel is 0,2,1,3, because the order in
// the register map is: carrier 1, carrier 2, modulator 1, modulator 2
static void opn_operator_map(void* owner, UINT32* map)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;
	UINT32 channels = regs->is_opna ? 6 : 3;
	UINT32 chnum;
	UINT32 base;

	for (chnum = 0; chnum < channels; chnum ++)
	{
		base = (chnum / 3) * 12 + (chnum % 3);
		map[chnum] = fm_operator_list(base + 0, base + 6, base + 3, base + 9);
	}
	return;
}

static void opn_reset(void* owner)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;

	memset(regs->regdata, 0x00, REGISTERS);
	if (regs->is_opna)
	{
		// enable output on both channels by default
		REG(0xb4) = REG(0xb5) = REG(0xb6) = 0xc0;
		REG(0x1b4) = REG(0x1b5) = REG(0x1b6) = 0xc0;
	}
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 opn_write(void* owner, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;
	UINT32 latchindex;

	// writes in the 0xa0-af/0x1a0-af region are handled as latched pairs
	// borrow unused registers 0xb8-bf as temporary holding locations
	if ((index & 0xf0) == 0xa0)
	{
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
		if (regs->is_opna)
			*channel += bitfield(data, 2, 1) * 3;
		*opmask = bitfield(data, 4, 4);
		return 1;
	}
	return 0;
}

static void opn_read_mode(void* owner, FM_MODE* mode)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;

	mode->timer_a_value = timer_a_value();
	mode->timer_b_value = timer_b_value();
	mode->status_mask = 0;
	mode->irq_reset = 0;
	mode->csm = (UINT8)csm();
	mode->reset_timer_a = (UINT8)reset_timer_a();
	mode->reset_timer_b = (UINT8)reset_timer_b();
	mode->enable_timer_a = (UINT8)enable_timer_a();
	mode->enable_timer_b = (UINT8)enable_timer_b();
	mode->load_timer_a = (UINT8)load_timer_a();
	mode->load_timer_b = (UINT8)load_timer_b();
	return;
}

// clock the LFO, returning the LFO PM value; OPN has no noise generator
static INT32 opn_clock_noise_and_lfo(void* owner, UINT8* lfo_am, UINT8* noise_state)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;
	UINT32 subcount;
	INT32 pm;

	*noise_state = 0;

	// if LFO not enabled (not present on OPN), quick exit with 0s
	if (! lfo_enable())
	{
		regs->lfo_counter = 0;

		// special case: if LFO is disabled on OPNA, it basically just keeps the counter
		// at 0; since position 0 gives an AM value of 0x3f, it is important to reflect
		// that here; for example, MegaDrive Venom plays some notes with LFO globally
		// disabled but enabling LFO on the operators, and it expects this added attenutation
		*lfo_am = regs->is_opna ? 0x3f : 0x00;
		return 0;
	}

	subcount = (UINT8)(regs->lfo_counter ++);

	// when we cross the divider count, add enough to zero it and cause an
	// increment at bit 8; the 7-bit value lives from bits 8-14
	if (subcount >= s_lfo_max_count[lfo_rate()])
	{
		// note: to match the published values this should be 0x100 - subcount;
		// however, tests on the hardware and nuked bear out an off-by-one
		// error exists that causes the max LFO rate to be faster than published
		regs->lfo_counter += 0x101 - subcount;
	}

	// AM value is 7 bits, staring at bit 8; grab the low 6 directly
	*lfo_am = (UINT8)bitfield(regs->lfo_counter, 8, 6);

	// first half of the AM period (bit 6 == 0) is inverted
	if (bitfield(regs->lfo_counter, 8+6, 1) == 0)
		*lfo_am ^= 0x3f;

	// PM value is 5 bits, starting at bit 10; grab the low 3 directly
	pm = (INT32)bitfield(regs->lfo_counter, 10, 3);

	// PM is reflected based on bit 3
	if (bitfield(regs->lfo_counter, 10+3, 1))
		pm ^= 7;

	// PM is negated based on bit 4
	return bitfield(regs->lfo_counter, 10+4, 1) ? -pm : pm;
}

static void opn_cache_channel_data(void* owner, UINT32 choffs, FM_CHANNEL_CACHE* cache)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;

	cache->feedback = (UINT8)ch_feedback(choffs);
	cache->algorithm = (UINT8)ch_algorithm(choffs);
	cache->outputs = (UINT8)(ch_output_0(choffs) | (ch_output_1(choffs) << 1));

	// shift value for AM sensitivity is [7, 3, 1, 0],
	// mapping to values of [0, 1.4, 5.9, and 11.8dB]

	// raw LFO AM value on OPN is 0-3F, scale that up by a factor of 2
	// (giving 7 bits) before applying the final shift
	cache->am_lshift = 1;
	cache->am_rshift = (UINT8)((1 << (ch_lfo_am_sens(choffs) ^ 3)) - 1);
	cache->kind = FM_CH_NORMAL;
	return;
}

// compute the phase step, given a PM value
static UINT32 opn_compute_phase_step(void* owner, UINT32 choffs, UINT32 opoffs, const FM_OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;
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
		fnum += fm_opn_lfo_pm_phase_adjustment(bitfield(cache->block_freq, 4, 7), pm_sensitivity, lfo_raw_pm);

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
static void opn_cache_operator_data(void* owner, UINT32 choffs, UINT32 opoffs, FM_OPDATA_CACHE* cache)
{
	OPN_REGISTERS* regs = (OPN_REGISTERS*)owner;
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 ksrval;

	// set up the easy stuff
	cache->waveform = regs->waveform;
	cache->lfo_am_enable = (UINT8)op_lfo_am_enable(opoffs);
	cache->ssg_eg = (UINT8)op_ssg_eg(opoffs);

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
	cache->detune = fm_detune_adjustment(op_detune(opoffs), keycode);

	// multiple value, as an x.1 value (0 means 0.5)
	cache->multiple = op_multiple(opoffs) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on
	// block_freq, detune, and multiple, so compute it after we've done those
	if (lfo_enable() == 0 || ch_lfo_pm_sens(choffs) == 0)
		cache->phase_step = opn_compute_phase_step(owner, choffs, opoffs, cache, 0);
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
	cache->eg_rate[EG_ATTACK] = (UINT8)fm_effective_rate(op_attack_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_DECAY] = (UINT8)fm_effective_rate(op_decay_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_SUSTAIN] = (UINT8)fm_effective_rate(op_sustain_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_RELEASE] = (UINT8)fm_effective_rate(op_release_rate(opoffs) * 4 + 2, ksrval);
	return;
}

#undef REG

// YM2203: 3 channels, 1 output
static const FM_FAMILY opn_family =
{
	3,					// channels
	1,					// outputs
	3 * 4,				// operators
	DEFAULT_PRESCALE,	// default_prescale
	3,					// eg_clock_divider
	REG_MODE,			// reg_mode
	1 << 2,				// csm_trigger_mask
	STATUS_TIMERA,		// status_timera
	STATUS_TIMERB,		// status_timerb
	STATUS_BUSY,		// status_busy
	0,					// status_irq
	0,					// dynamic_ops
	0,					// modulator_delay
	1,					// eg_has_ssg

	opn_channel_offset,
	opn_operator_offset,
	opn_operator_map,
	opn_reset,
	opn_write,
	opn_read_mode,
	opn_clock_noise_and_lfo,
	opn_cache_channel_data,
	opn_cache_operator_data,
	opn_compute_phase_step,
};

// YM2608, YM2610, YM2612: 6 channels, 2 outputs
static const FM_FAMILY opna_family =
{
	6,					// channels
	2,					// outputs
	6 * 4,				// operators
	DEFAULT_PRESCALE,	// default_prescale
	3,					// eg_clock_divider
	REG_MODE,			// reg_mode
	1 << 2,				// csm_trigger_mask
	STATUS_TIMERA,		// status_timera
	STATUS_TIMERB,		// status_timerb
	STATUS_BUSY,		// status_busy
	0,					// status_irq
	0,					// dynamic_ops
	0,					// modulator_delay
	1,					// eg_has_ssg

	opn_channel_offset,
	opn_operator_offset,
	opn_operator_map,
	opn_reset,
	opn_write,
	opn_read_mode,
	opn_clock_noise_and_lfo,
	opn_cache_channel_data,
	opn_cache_operator_data,
	opn_compute_phase_step,
};

// set up the registers and the FM engine of one chip
static void opn_engine_init(FM_ENGINE* fm, OPN_REGISTERS* regs, UINT8 is_opna)
{
	UINT32 index;

	regs->is_opna = is_opna;

	// create the waveforms
	for (index = 0; index < FM_WAVEFORM_LENGTH; index ++)
		regs->waveform[index] = (UINT16)(fm_abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	fm_engine_init(fm, is_opna ? &opna_family : &opn_family, regs);
	return;
}

#define ch_output_0_of(regs, choffs)	bitfield((regs)->regdata[0xb4 + (choffs)], 7, 1)
#define ch_output_1_of(regs, choffs)	bitfield((regs)->regdata[0xb4 + (choffs)], 6, 1)


//----------------------------------------------------------------------------
// SSG link and prescaler (YM2203, YM2608, YM2610)
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
	scale = chip->fm.clock_prescale * 2 / 3;	// integer, so prescale 2 gives 1
	if (chip->type == CHIP_YM2203)
		return chip->clock * 2 / scale;
	return chip->clock / scale;
}

static void update_prescale(OPN_STATE* chip, UINT8 prescale)
{
	if (chip->fm.clock_prescale == prescale)
		return;
	chip->fm.clock_prescale = prescale;
	if (chip->ssg.set_clock != NULL)
		chip->ssg.set_clock(chip->ssg.param, ssg_clock(chip));
	if (chip->smpRateFunc != NULL)
		chip->smpRateFunc(chip->smpRateData, fm_engine_sample_rate(&chip->fm, chip->clock));
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
		else if (chip->address == 0x2e && chip->fm.clock_prescale == 6)
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
		fm_engine_write(&chip->fm, chip->address, data);
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
	INT32 output[1];

	// clock the system
	fm_engine_clock(&chip->fm, &opn_family, chip->fm.all_channels);

	// update the FM content; OPN is full 14-bit with no intermediate clipping
	output[0] = 0;
	fm_engine_output(&chip->fm, &opn_family, output, 0, 32767, chip->fm.all_channels);

	// convert to 10.3 floating point value for the DAC and back;
	// the chip is mono, so both outputs carry the same signal
	*outL = *outR = fm_roundtrip_fp(output[0]);
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
	status = fm_engine_status(&chip->fm) & ~(STATUS_ADPCM_B_EOS | STATUS_ADPCM_B_BRDY | STATUS_ADPCM_B_PLAYING);

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
	fm_engine_set_reset_status(&chip->fm, status, ~status);
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
		return adpcm_b_read(&chip->adpcm_b, chip->address & 0x0f);
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
		adpcm_a_write(&chip->adpcm_a, chip->address & 0x0f, data);
	}
	else if (chip->address == 0x29)
	{
		// 29: special IRQ mask register
		chip->irq_enable = data;
	}
	else
	{
		// 20-28, 2A-FF: write to FM
		fm_engine_write(&chip->fm, chip->address, data);
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
		adpcm_b_write(&chip->adpcm_b, chip->address & 0x0f, data);
	}
	else if (chip->address == 0x110)
	{
		// 110: IRQ flag control
		if (bitfield(data, 7, 1))
			fm_engine_set_reset_status(&chip->fm, 0, 0xff);
		else
			chip->flag_control = data;
	}
	else
	{
		// 111-1FF: write to FM
		fm_engine_write(&chip->fm, chip->address, data);
	}
	return;
}

// mix the ADPCM channels into output[] and convert it to the device output
static void opn_mix_adpcm(OPN_STATE* chip, INT32* output, DEV_SMPL* outL, DEV_SMPL* outR)
{
	adpcm_a_output(&chip->adpcm_a, output, ~(chip->muteMask >> MUTE_ADPCM_A) & 0x3f);
	if (! ((chip->muteMask >> MUTE_ADPCM_B) & 0x01))
		adpcm_b_output(&chip->adpcm_b, output, 2, 1);

	// clamp; then double, to match the YM2203 level
	*outL = clamp(output[0], -32768, 32767) * 2;
	*outR = clamp(output[1], -32768, 32767) * 2;
	return;
}

// generate one sample of sound
static void ym2608_generate(OPN_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	UINT32 fmmask;
	UINT32 env_counter;

	// top bit of the IRQ enable flags controls 3-channel vs 6-channel mode
	fmmask = bitfield(chip->irq_enable, 7, 1) ? 0x3f : 0x07;

	// clock the system
	env_counter = fm_engine_clock(&chip->fm, &opna_family, chip->fm.all_channels);

	// clock the ADPCM-A engine on every envelope cycle
	// (channels 4 and 5 clock every 2 envelope clocks)
	if (bitfield(env_counter, 0, 2) == 0)
		adpcm_a_clock(&chip->adpcm_a, bitfield(env_counter, 2, 1) ? 0x0f : 0x3f);

	// clock the ADPCM-B engine every cycle
	adpcm_b_clock(&chip->adpcm_b);

	// update the FM content; OPNA is 13-bit with no intermediate clipping
	output[0] = output[1] = 0;
	fm_engine_output(&chip->fm, &opna_family, output, 1, 32767, fmmask);

	opn_mix_adpcm(chip, output, outL, outR);
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
		adpcm_b_write(&chip->adpcm_b, chip->address & 0x0f, data);
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
		fm_engine_write(&chip->fm, chip->address, data);
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
		adpcm_a_write(&chip->adpcm_a, chip->address & 0x3f, data);
	}
	else
	{
		// 130-1FF: write to FM
		fm_engine_write(&chip->fm, chip->address, data);
	}
	return;
}

// generate one sample of sound
static void ym2610_generate(OPN_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	UINT32 env_counter;
	UINT8 live_eos;

	// clock the system
	env_counter = fm_engine_clock(&chip->fm, &opna_family, chip->fm_mask);

	// clock the ADPCM-A engine on every envelope cycle
	if (bitfield(env_counter, 0, 2) == 0)
		chip->eos_status |= (UINT8)adpcm_a_clock(&chip->adpcm_a, 0x3f);

	// clock the ADPCM-B engine every cycle
	adpcm_b_clock(&chip->adpcm_b);

	// we track the last ADPCM-B EOS value in bit 6 (which is hidden from callers);
	// if it changed since the last sample, update the visible EOS state in bit 7
	live_eos = ((chip->adpcm_b.status & ADPCM_B_STATUS_EOS) != 0) ? 0x40 : 0x00;
	if (((live_eos ^ chip->eos_status) & 0x40) != 0)
		chip->eos_status = (chip->eos_status & ~0xc0) | live_eos | (live_eos << 1);

	// update the FM content; OPNB is 13-bit with no intermediate clipping
	output[0] = output[1] = 0;
	fm_engine_output(&chip->fm, &opna_family, output, 1, 32767, chip->fm_mask);

	opn_mix_adpcm(chip, output, outL, outR);
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface (YM2203, YM2608, YM2610)
//----------------------------------------------------------------------------

// reset the chip, as ymfm's reset() does
static void opn_reset_chip(OPN_STATE* chip)
{
	fm_engine_reset(&chip->fm);
	if (chip->ssg.reset != NULL)
		chip->ssg.reset(chip->ssg.param);
	if (chip->type == CHIP_YM2608)
	{
		adpcm_a_reset(&chip->adpcm_a);
		adpcm_b_reset(&chip->adpcm_b);

		// configure ADPCM percussion sounds; these are present in an embedded ROM
		adpcm_a_set_start_end(&chip->adpcm_a, 0, 0x0000, 0x01bf);	// bass drum
		adpcm_a_set_start_end(&chip->adpcm_a, 1, 0x01c0, 0x043f);	// snare drum
		adpcm_a_set_start_end(&chip->adpcm_a, 2, 0x0440, 0x1b7f);	// top cymbal
		adpcm_a_set_start_end(&chip->adpcm_a, 3, 0x1b80, 0x1cff);	// high hat
		adpcm_a_set_start_end(&chip->adpcm_a, 4, 0x1d00, 0x1f7f);	// tom tom
		adpcm_a_set_start_end(&chip->adpcm_a, 5, 0x1f80, 0x1fff);	// rim shot

		// initialize our special interrupt states, then read the upper status
		// register, which updates the IRQs
		chip->irq_enable = 0x1f;
		chip->flag_control = 0x1c;
		ym2608_read_status_hi(chip);
	}
	else if (chip->type == CHIP_YM2610)
	{
		adpcm_a_reset(&chip->adpcm_a);
		adpcm_b_reset(&chip->adpcm_b);

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
	chip->clock = cfg->clock;
	opn_engine_init(&chip->fm, &chip->regs, type != CHIP_YM2203);
	chip->fm_mask = chip->fm.all_channels;
	if (type == CHIP_YM2608)
	{
		adpcm_a_init(&chip->adpcm_a, 0);
		adpcm_b_init(&chip->adpcm_b, 0);
		chip->adpcm_a.mem = ym2608_rhythm_rom;
		chip->adpcm_a.mem_size = YM2608_RHYTHM_ROM_SIZE;
	}
	else if (type == CHIP_YM2610)
	{
		// the YM2610 lacks FM channels 1 and 4
		chip->fm_mask = cfg->flags ? 0x3f : 0x36;
		adpcm_a_init(&chip->adpcm_a, 8);
		adpcm_b_init(&chip->adpcm_b, 8);
	}
	opn_reset_chip(chip);
	chip->muteMask = 0x00;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, fm_engine_sample_rate(&chip->fm, chip->clock), devDef);
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
		return fm_engine_status(&chip->fm);	// status port
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
		return fm_engine_status(&chip->fm) & (STATUS_TIMERA | STATUS_TIMERB);
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
		return fm_engine_status(&chip->fm) & (STATUS_TIMERA | STATUS_TIMERB);
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
	chip->adpcm_a.mem = chip->adpcm_a_mem;
	chip->adpcm_a.mem_size = chip->adpcm_a_mem_size;
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
	chip->adpcm_b.mem = chip->adpcm_b_mem;
	chip->adpcm_b.mem_size = chip->adpcm_b_mem_size;
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
		chip->adpcm_a.mem = chip->adpcm_a_mem;
		chip->adpcm_a.mem_size = chip->adpcm_a_mem_size;
	}
	opn_write_mem(chip->adpcm_a_mem, chip->adpcm_a_mem_size, offset, length, data);
	return;
}

static void opn_set_mute_mask(void* info, UINT32 MuteMask)
{
	OPN_STATE* chip = (OPN_STATE*)info;

	chip->muteMask = MuteMask;
	chip->fm.mute_mask = MuteMask & chip->fm.all_channels;
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


//----------------------------------------------------------------------------
// YM2612
//----------------------------------------------------------------------------

// simulate the DAC discontinuity
#define dac_discontinuity(value)	(((value) < 0) ? ((value) - 3) : ((value) + 4))

static void ym2612_write_data(YM2612_STATE* chip, UINT8 data)
{
	// ignore if paired with upper address
	if (bitfield(chip->address, 8, 1))
		return;

	if (chip->address == 0x2a)
	{
		// 2A: DAC data (most significant 8 bits)
		chip->dac_data = (chip->dac_data & ~0x1fe) | ((data ^ 0x80) << 1);
	}
	else if (chip->address == 0x2b)
	{
		// 2B: DAC enable (bit 7)
		chip->dac_enable = (UINT8)bitfield(data, 7, 1);
	}
	else if (chip->address == 0x2c)
	{
		// 2C: test/low DAC bit
		chip->dac_data = (chip->dac_data & ~1) | bitfield(data, 3, 1);
	}
	else
	{
		// 00-29, 2D-FF: write to FM
		fm_engine_write(&chip->fm, chip->address, data);
	}
	return;
}

static void ym2612_write_data_hi(YM2612_STATE* chip, UINT8 data)
{
	// ignore if paired with lower address
	if (! bitfield(chip->address, 8, 1))
		return;

	// 100-1FF: write to FM
	fm_engine_write(&chip->fm, chip->address, data);
	return;
}

// generate one sample of sound
static void ym2612_generate(YM2612_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];
	INT32 temp[2];
	INT32 dacval;
	UINT8 last_fm_channel;
	UINT8 chan;

	// clock the system
	fm_engine_clock(&chip->fm, &opna_family, chip->fm.all_channels);

	// sum individual channels to apply DAC discontinuity on each
	output[0] = output[1] = 0;

	// first do FM-only channels; OPN2 is 9-bit with intermediate clipping
	last_fm_channel = chip->dac_enable ? 5 : 6;
	for (chan = 0; chan < last_fm_channel; chan ++)
	{
		temp[0] = temp[1] = 0;
		fm_engine_output(&chip->fm, &opna_family, temp, 5, 256, 1 << chan);
		if ((chip->muteMask >> chan) & 0x01)
			continue;
		output[0] += dac_discontinuity(temp[0]);
		output[1] += dac_discontinuity(temp[1]);
	}

	// add in DAC
	if (chip->dac_enable && ! ((chip->muteMask >> 6) & 0x01))
	{
		// DAC enabled: start with DAC value then add the first 5 channels only
		dacval = dac_discontinuity((INT16)(chip->dac_data << 7) >> 7);
		output[0] += ch_output_0_of(&chip->regs, 0x102) ? dacval : dac_discontinuity(0);
		output[1] += ch_output_1_of(&chip->regs, 0x102) ? dacval : dac_discontinuity(0);
	}

	// output is technically multiplexed rather than mixed, but that requires
	// a better sound mixer than we usually have, so just average over the six
	// channels; also apply a 64/65 factor to account for the discontinuity
	// adjustment above
	*outL = (output[0] * 128) * 64 / (6 * 65);
	*outR = (output[1] * 128) * 64 / (6 * 65);
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface (YM2612)
//----------------------------------------------------------------------------

static UINT8 device_start_ym2612(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YM2612_STATE* chip;

	chip = (YM2612_STATE*)calloc(1, sizeof(YM2612_STATE));
	if (chip == NULL)
		return 0xFF;

	opn_engine_init(&chip->fm, &chip->regs, 1);
	fm_engine_reset(&chip->fm);
	chip->muteMask = 0x00;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / 144, &devDef2612);
	return 0x00;
}

static void device_stop_ym2612(void* info)
{
	free(info);
	return;
}

static void device_reset_ym2612(void* info)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	// ymfm's reset leaves the YM2612 address and DAC alone; a device reset clears them too
	chip->address = 0;
	chip->dac_data = 0;
	chip->dac_enable = 0;
	fm_engine_reset(&chip->fm);
	return;
}

static void ym2612_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2612_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2612_w(void* info, UINT8 offset, UINT8 data)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	switch(offset & 3)
	{
	case 0:	// address port
		chip->address = data;
		break;
	case 1:	// data port
		ym2612_write_data(chip, data);
		break;
	case 2:	// upper address port
		chip->address = 0x100 | data;
		break;
	case 3:	// upper data port
		ym2612_write_data_hi(chip, data);
		break;
	}
	return;
}

static UINT8 ym2612_r(void* info, UINT8 offset)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	// status port, YM2203 compatible; the other offsets are unused
	if ((offset & 3) != 0)
		return 0x00;
	return fm_engine_status(&chip->fm);
}

static void ym2612_set_mute_mask(void* info, UINT32 MuteMask)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	chip->muteMask = MuteMask;
	return;
}
