/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    Yamaha OPL family

    C port of ymfm's OPL family (ymfm_opl.h, ymfm_opl.cpp).
    https://github.com/aaronsgiles/ymfm

    Chips: YM3526 (OPL), Y8950 (MSX-AUDIO), YM3812 (OPL2), YMF262 (OPL3)
    and YMF278B (OPL4).

    Port notes:
    - As in ymfm, all chips share one register implementation
      (OPL_REGISTERS, ymfm's opl_registers_base). ymfm's Revision template
      parameter becomes the revision field (1 = OPL: YM3526/Y8950, 2 = OPL2:
      YM3812, 3 = OPL3: YMF262), and the OPL/OPL2 and the OPL3 each have
      their own FM_FAMILY table for the shared FM engine (ymfm_fm.c).
    - The OPLL (YM2413) and the OPL4 revision of the registers are not
      ported. The FM part of the YMF278B (OPL4) is a linked YMF262 device,
      as in version 1 (YMF262_FLAG_OPL4).
    - The Y8950 ADPCM-B channel is the shared engine of ymfm_adpcm.c.
    - OPL has no AM sensitivity: the AM offset is the LFO AM value, cached
      as the shift pair (0, 0). The rhythm channels 6-8 use the engine's
      FM_CH_RHYTHM_xxx channel kinds, set when the channel is cached.
    - One update sample is one FM sample: clock / 72 on the OPL/OPL2,
      clock / 288 on the OPL3.
    - OPL/OPL2/Y8950: the chips are mono; both outputs carry the same
      signal.
    - OPL3: the 4 outputs are mixed to stereo like libvgm's MAME core:
      left = A + C, right = B + D. Each output is clamped to 16 bits first,
      as ymfm does. With YMF262_FLAG_OPL4 (cfg.flags bit 0, the FM part of
      a YMF278B), left = A and right = B: the OPL4 mixes only those into
      its stereo pin.
    - OPL3: the volume function scales the stereo output by n / 0x10000
      (the OPL4 FM mix level), with a 11-bit resolution.
    - Y8950: the ADPCM-B sample memory is filled by the player
      (DEVRW_MEMSIZE, DEVRW_BLOCK); reads outside it return 0. The keyboard
      and I/O ports are not connected: reads return 0 and writes are
      dropped.
    - YMF278B: this device is the wavetable part (ymfm_pcm.c) and the
      register interface. The FM part is a linked YMF262 device that runs at
      clock * 8 / 19 with YMF262_FLAG_OPL4, which is the OPL4 FM engine
      rate (19 * 36 cycles per sample). Register writes of the FM banks are
      forwarded to it, its status is merged into the status register and
      the FM mix level of the mix control (registers F8/F9) is set as its
      volume.
    - YMF278B: ymfm clocks the FM engine once per 44.1 kHz sample plus an
      extra tick every few samples and drops the output of the extra ticks.
      Here the FM runs at its own rate and the player resamples it. This
      also computes the feedback of every tick, which ymfm skips on the
      extra ticks.
    - YMF278B: the wavetable part outputs one sample per clock / 768. Only
      the DO2 mix of ymfm (output channel 0 of the PCM channels) is
      audible; the DO0 and DO1 pins are not mixed. The FM and PCM levels
      are rounded separately: (fm * level >> 11) + (pcm * level >> 11),
      ymfm rounds the sum.
    - YMF278B: the ROM is not part of this core: the host loads it with the
      0x524F memory function, the RAM with 0x5241 (memory map: see
      ymfm_pcm.c). The LD flag and the chip ID of the first status read
      are emulated.
    - The busy flag and the IRQ line are not emulated.
    - The key-on fix of the shared engine applies (see ymfm_fm.c); the
      YMF278B wavetable part has its own (see ymfm_pcm.c).

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

#include "ymfm_fm.h"
#include "ymfm_fm_impl.h"
#include "ymfm_adpcm.h"
#include "ymfm_pcm.h"
#include "ymfm_opl.h"


static void opl_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void ymf262_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym3526(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static UINT8 device_start_y8950(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static UINT8 device_start_ym3812(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static UINT8 device_start_ymf262(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_opl(void* info);
static void device_reset_opl(void* info);
static void ym3812_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym3812_r(void* info, UINT8 offset);
static void y8950_w(void* info, UINT8 offset, UINT8 data);
static UINT8 y8950_r(void* info, UINT8 offset);
static void y8950_alloc_pcmrom(void* info, UINT32 memsize);
static void y8950_write_pcmrom(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void ymf262_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ymf262_r(void* info, UINT8 offset);
static void ymf262_set_vol_lr(void* info, INT32 volL, INT32 volR);
static void opl_set_mute_mask(void* info, UINT32 MuteMask);

static void ymf278b_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ymf278b(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ymf278b(void* info);
static void device_reset_ymf278b(void* info);
static UINT8 device_ymf278b_link_fm(void* info, UINT8 linkID, const DEV_INFO* devInfLink);
static void ymf278b_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ymf278b_r(void* info, UINT8 offset);
static void ymf278b_alloc_rom(void* info, UINT32 memsize);
static void ymf278b_alloc_ram(void* info, UINT32 memsize);
static void ymf278b_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void ymf278b_write_ram(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void ymf278b_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc3812[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym3812_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym3812_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opl_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef3812 =
{
	"YM3812", "ymfm", FCC_YMFM,

	device_start_ym3812,
	device_stop_opl,
	device_reset_opl,
	opl_update,

	NULL,	// SetOptionBits
	opl_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc3812,	// rwFuncs
};
static DEV_DEF devDef3526 =
{
	"YM3526", "ymfm", FCC_YMFM,

	device_start_ym3526,
	device_stop_opl,
	device_reset_opl,
	opl_update,

	NULL,	// SetOptionBits
	opl_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc3812,	// rwFuncs
};

static DEVDEF_RWFUNC devFunc8950[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, y8950_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, y8950_r},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, y8950_write_pcmrom},
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0, y8950_alloc_pcmrom},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opl_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef8950 =
{
	"Y8950", "ymfm", FCC_YMFM,

	device_start_y8950,
	device_stop_opl,
	device_reset_opl,
	opl_update,

	NULL,	// SetOptionBits
	opl_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc8950,	// rwFuncs
};

static DEVDEF_RWFUNC devFunc262[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ymf262_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ymf262_r},
	{RWF_VOLUME_LR | RWF_WRITE, DEVRW_VALUE, 0, ymf262_set_vol_lr},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, opl_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef262 =
{
	"YMF262", "ymfm", FCC_YMFM,

	device_start_ymf262,
	device_stop_opl,
	device_reset_opl,
	ymf262_update,

	NULL,	// SetOptionBits
	opl_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc262,	// rwFuncs
};

static DEVDEF_RWFUNC devFunc278b[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ymf278b_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ymf278b_r},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0x524F, ymf278b_write_rom},	// 0x524F = 'RO' for ROM
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0x524F, ymf278b_alloc_rom},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0x5241, ymf278b_write_ram},	// 0x5241 = 'RA' for RAM
	{RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0x5241, ymf278b_alloc_ram},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ymf278b_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef278b =
{
	"YMF278B", "ymfm", FCC_YMFM,

	device_start_ymf278b,
	device_stop_ymf278b,
	device_reset_ymf278b,
	ymf278b_update,

	NULL,	// SetOptionBits
	ymf278b_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	device_ymf278b_link_fm,	// LinkDevice

	devFunc278b,	// rwFuncs
};

static const char* DeviceName_YM3526(const DEV_GEN_CFG* devCfg)
{
	return "YM3526";
}

static const char* DeviceName_Y8950(const DEV_GEN_CFG* devCfg)
{
	return "Y8950";
}

static const char* DeviceName_YM3812(const DEV_GEN_CFG* devCfg)
{
	return "YM3812";
}

static const char* DeviceName_YMF262(const DEV_GEN_CFG* devCfg)
{
	return "YMF262";
}

#define DEV_CHN_COUNT_OPL	14
static UINT16 DeviceChannels_OPL(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_OPL;
}

static const char** DeviceChannelNames_OPL(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_OPL] =
	{
		"1", "2", "3", "4", "5", "6", "7", "8", "9",
		"Bass Drum", "Snare Drum", "Tom Tom", "Cymbal", "Hi-Hat",
	};
	return names;
}

#define DEV_CHN_COUNT_Y8950	15
static UINT16 DeviceChannels_Y8950(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_Y8950;
}

static const char** DeviceChannelNames_Y8950(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_Y8950] =
	{
		"1", "2", "3", "4", "5", "6", "7", "8", "9",
		"Bass Drum", "Snare Drum", "Tom Tom", "Cymbal", "Hi-Hat",
		"ADPCM",
	};
	return names;
}

#define DEV_CHN_COUNT_YMF262	23
static UINT16 DeviceChannels_YMF262(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_YMF262;
}

static const char** DeviceChannelNames_YMF262(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_YMF262] =
	{
		"1", "2", "3", "4", "5", "6", "7", "8", "9",
		"10", "11", "12", "13", "14", "15", "16", "17", "18",
		"Bass Drum", "Snare Drum", "Tom Tom", "Cymbal", "Hi-Hat",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs_OPL(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YM3526 =
{
	DEVID_YM3526,
	DeviceName_YM3526,
	DeviceChannels_OPL,
	DeviceChannelNames_OPL,
	DeviceLinkIDs_OPL,
	{	// cores
		&devDef3526,
		NULL
	}
};

const DEV_DECL sndDev_Y8950 =
{
	DEVID_Y8950,
	DeviceName_Y8950,
	DeviceChannels_Y8950,
	DeviceChannelNames_Y8950,
	DeviceLinkIDs_OPL,
	{	// cores
		&devDef8950,
		NULL
	}
};

const DEV_DECL sndDev_YM3812 =
{
	DEVID_YM3812,
	DeviceName_YM3812,
	DeviceChannels_OPL,
	DeviceChannelNames_OPL,
	DeviceLinkIDs_OPL,
	{	// cores
		&devDef3812,
		NULL
	}
};

const DEV_DECL sndDev_YMF262 =
{
	DEVID_YMF262,
	DeviceName_YMF262,
	DeviceChannels_YMF262,
	DeviceChannelNames_YMF262,
	DeviceLinkIDs_OPL,
	{	// cores
		&devDef262,
		NULL
	}
};


static const char* DeviceName_YMF278B(const DEV_GEN_CFG* devCfg)
{
	return "YMF278B";
}

#define DEV_CHN_COUNT_YMF278B	24
static UINT16 DeviceChannels_YMF278B(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT_YMF278B;
}

static const char** DeviceChannelNames_YMF278B(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_YMF278B] =
	{
		"PCM 1", "PCM 2", "PCM 3", "PCM 4", "PCM 5", "PCM 6", "PCM 7", "PCM 8",
		"PCM 9", "PCM 10", "PCM 11", "PCM 12", "PCM 13", "PCM 14", "PCM 15", "PCM 16",
		"PCM 17", "PCM 18", "PCM 19", "PCM 20", "PCM 21", "PCM 22", "PCM 23", "PCM 24",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs_YMF278B(const DEV_GEN_CFG* devCfg)
{
	static const DEVLINK_IDS dlIDs = {1, {DEVID_YMF262}};
	return &dlIDs;
}

const DEV_DECL sndDev_YMF278B =
{
	DEVID_YMF278B,
	DeviceName_YMF278B,
	DeviceChannels_YMF278B,
	DeviceChannelNames_YMF278B,
	DeviceLinkIDs_YMF278B,
	{	// cores
		&devDef278b,
		NULL
	}
};

//----------------------------------------------------------------------------
// chip state
//----------------------------------------------------------------------------

#define CHIP_YM3526		0
#define CHIP_Y8950		1
#define CHIP_YM3812		2
#define CHIP_YMF262		3

#define REGISTERS		0x200
#define WAVEFORMS		8
#define REG_MODE		0x04

#define STATUS_TIMERA	0x40
#define STATUS_TIMERB	0x20
#define STATUS_IRQ		0x80

// Y8950 status bits
#define STATUS_ADPCM_B_PLAYING	0x01
#define STATUS_ADPCM_B_BRDY		0x08
#define STATUS_ADPCM_B_EOS		0x10

// Y8950 mute mask bit beyond the 9 FM channels and the 5 rhythm instruments
#define MUTE_ADPCM		14

// YMF262 output volume: 0x800 = 100%
#define VOL_UNITY		0x800

// YMF278B
#define STATUS_LD		0x02

// the FM part of the YMF278B clocks every 19 * 36 cycles instead of 8 * 36
#define FM_CLOCK_NUM	8
#define FM_CLOCK_DEN	19

//
// OPL/OPL2/OPL3/OPL4 register map:
//
//      System-wide registers:
//           01 xxxxxxxx Test register
//              --x----- Enable OPL compatibility mode [OPL2 only] (0 = enable)
//           02 xxxxxxxx Timer A value (4 * OPN)
//           03 xxxxxxxx Timer B value
//           04 x------- RST
//              -x------ Mask timer A
//              --x----- Mask timer B
//              ------x- Load timer B
//              -------x Load timer A
//           08 x------- CSM mode [OPL/OPL2 only]
//              -x------ Note select
//           BD x------- AM depth
//              -x------ PM depth
//              --x----- Rhythm enable
//              ---x---- Bass drum key on
//              ----x--- Snare drum key on
//              -----x-- Tom key on
//              ------x- Top cymbal key on
//              -------x High hat key on
//          101 --xxxxxx Test register 2 [OPL3 only]
//          104 --x----- Channel 6 4-operator mode [OPL3 only]
//              ---x---- Channel 5 4-operator mode [OPL3 only]
//              ----x--- Channel 4 4-operator mode [OPL3 only]
//              -----x-- Channel 3 4-operator mode [OPL3 only]
//              ------x- Channel 2 4-operator mode [OPL3 only]
//              -------x Channel 1 4-operator mode [OPL3 only]
//          105 -------x New [OPL3 only]
//              ------x- New2 [OPL4 only]
//
//     Per-channel registers (channel in address bits 0-3)
//     Note that all these apply to address+100 as well on OPL3+
//        A0-A8 xxxxxxxx F-number (low 8 bits)
//        B0-B8 --x----- Key on
//              ---xxx-- Block (octvate, 0-7)
//              ------xx F-number (high two bits)
//        C0-C8 x------- CHD output (to DO0 pin) [OPL3+ only]
//              -x------ CHC output (to DO0 pin) [OPL3+ only]
//              --x----- CHB output (mixed right, to DO2 pin) [OPL3+ only]
//              ---x---- CHA output (mixed left, to DO2 pin) [OPL3+ only]
//              ----xxx- Feedback level for operator 1 (0-7)
//              -------x Operator connection algorithm
//
//     Per-operator registers (operator in bits 0-5)
//     Note that all these apply to address+100 as well on OPL3+
//        20-35 x------- AM enable
//              -x------ PM enable (VIB)
//              --x----- EG type
//              ---x---- Key scale rate
//              ----xxxx Multiple value (0-15)
//        40-55 xx------ Key scale level (0-3)
//              --xxxxxx Total level (0-63)
//        60-75 xxxx---- Attack rate (0-15)
//              ----xxxx Decay rate (0-15)
//        80-95 xxxx---- Sustain level (0-15)
//              ----xxxx Release rate (0-15)
//        E0-F5 ------xx Wave select (0-3) [OPL2 only]
//              -----xxx Wave select (0-7) [OPL3+ only]
//

// the OPL registers (ymfm's opl_registers_base); the FM engine gets a
// pointer to this as its owner
typedef struct _opl_registers
{
	UINT8 revision;			// 1 = OPL, 2 = OPL2, 3 = OPL3
	UINT16 lfo_am_counter;	// LFO AM counter
	UINT16 lfo_pm_counter;	// LFO PM counter
	UINT32 noise_lfsr;		// noise LFSR state
	UINT8 regdata[REGISTERS];	// register data
	UINT16 waveform[WAVEFORMS][FM_WAVEFORM_LENGTH];	// waveforms
} OPL_REGISTERS;

// YM3526, Y8950, YM3812, YMF262
typedef struct _opl_state
{
	DEV_DATA _devData;

	UINT8 type;				// CHIP_xxx

	FM_ENGINE fm;
	OPL_REGISTERS regs;

	// chip
	UINT16 address;			// address register

	// Y8950
	UINT8 io_ddr;			// data direction register for I/O
	ADPCM_B_ENGINE adpcm_b;
	UINT8* adpcm_b_mem;		// ADPCM-B sample memory
	UINT32 adpcm_b_mem_size;

	// YMF262
	UINT8 opl4;				// non-zero if this is the FM part of a YMF278B
	INT32 vol[2];			// output scale for left/right, VOL_UNITY = 100%
} OPL_STATE;

// YMF278B: the wavetable part and the register interface; the FM part is a
// linked YMF262 device
// functions of the linked FM (YMF262) device
typedef void (*DEVFUNC_WRITE_VOLUME_LR)(void* info, INT32 volL, INT32 volR);
typedef struct _fm_funcs
{
	DEVFUNC_WRITE_A8D8 write;
	DEVFUNC_READ_A8D8 read;
	DEVFUNC_WRITE_VOLUME_LR set_vol;
	DEVFUNC_CTRL reset;
	void* param;
} FM_FUNCS;

typedef struct _ymf278b_state
{
	DEV_DATA _devData;

	PCM_ENGINE pcm;

	// chip
	UINT16 address;			// address register
	UINT32 load_remaining;	// how many more samples until LD flag clears
	UINT8 next_status_id;	// the next status read returns the chip ID
	UINT8 fm_reg105;		// the FM register 0x105 (NEW and NEW2 flags)
	FM_FUNCS fm;

	UINT32 rom_size;
	UINT32 ram_size;
	UINT8* rom;
	UINT8* ram;
} YMF278B_STATE;


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

// OPL key scale level attenuation, for the top 4 bits of FNUM
static const UINT8 s_fnum_to_atten[16] = { 0,24,32,37,40,43,45,47,48,50,51,52,53,54,55,56 };

// OPL PM LFO scale: the PM value is a 1.3 fraction and sign applied to the
// upper bits of FNUM
static const INT8 s_lfo_pm_scale[8] = { 8, 4, 0, -4, -8, -4, 0, 4 };

// convert an OPL concatenated block (3 bits) and the top 4 bits of fnum into
// an attenuation offset; values are for 6dB/octave, in 0.75dB units
// (matching total level LSB)
static UINT32 opl_key_scale_atten(UINT32 block, UINT32 fnum_4msb)
{
	INT32 result = s_fnum_to_atten[fnum_4msb] - 8 * (INT32)(block ^ 7);
	return (result > 0) ? (UINT32)result : 0;
}

#define REG(offset)		(regs->regdata[offset])
#define IS_OPL3			(regs->revision >= 3)

// system-wide registers
#define waveform_enable()	((regs->revision == 2) ? bitfield(REG(0x01), 5, 1) : (IS_OPL3 ? 1 : 0))
#define timer_a_value()		(bitfield(REG(0x02), 0, 8) * 4)	// 8->10 bits
#define timer_b_value()		bitfield(REG(0x03), 0, 8)
#define status_mask()		(bitfield(REG(0x04), 0, 8) & 0x78)
#define irq_reset()			bitfield(REG(0x04), 7, 1)
#define reset_timer_b()		(bitfield(REG(0x04), 7, 1) | bitfield(REG(0x04), 5, 1))
#define reset_timer_a()		(bitfield(REG(0x04), 7, 1) | bitfield(REG(0x04), 6, 1))
#define load_timer_b()		bitfield(REG(0x04), 1, 1)
#define load_timer_a()		bitfield(REG(0x04), 0, 1)
#define csm()				(IS_OPL3 ? 0 : bitfield(REG(0x08), 7, 1))
#define note_select()		bitfield(REG(0x08), 6, 1)
#define lfo_am_depth()		bitfield(REG(0xbd), 7, 1)
#define lfo_pm_depth()		bitfield(REG(0xbd), 6, 1)
#define rhythm_enable()		bitfield(REG(0xbd), 5, 1)
#define newflag()			(IS_OPL3 ? bitfield(REG(0x105), 0, 1) : 0)
#define fourop_enable()		(IS_OPL3 ? bitfield(REG(0x104), 0, 6) : 0)

// per-channel registers
#define ch_block_freq(choffs)	((bitfield(REG(0xb0 + (choffs)), 0, 5) << 8) | bitfield(REG(0xa0 + (choffs)), 0, 8))
#define ch_feedback(choffs)		bitfield(REG(0xc0 + (choffs)), 1, 3)
#define ch_algorithm(choffs)	(bitfield(REG(0xc0 + (choffs)), 0, 1) | (IS_OPL3 ? (8 | (bitfield(REG(0xc3 + (choffs)), 0, 1) << 1)) : 0))
#define ch_output_0(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 4, 1) : 1)
#define ch_output_1(choffs)		(newflag() ? bitfield(REG(0xc0 + (choffs)), 5, 1) : (IS_OPL3 ? 1 : 0))
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
#define op_waveform(opoffs)		(waveform_enable() ? bitfield(REG(0xe0 + (opoffs)), 0, newflag() ? 3 : 2) : 0)

// OPL3 has 2 register banks of 9 channels (OPL/OPL2 only the first); the
// channel offset is the register offset of the bank plus the channel
// number in the bank
static UINT32 opl_channel_offset(UINT32 chnum)
{
	return (chnum % 9) + 0x100 * (chnum / 9);
}

// the operators of each bank have a gap of 2 registers every 6 operators
static UINT32 opl_operator_offset(UINT32 opnum)
{
	return (opnum % 18) + 2 * ((opnum % 18) / 6) + 0x100 * (opnum / 18);
}

#define OPS2(o1, o2)	fm_operator_list(o1, o2, 0xff, 0xff)
#define OPS_NONE		fm_operator_list(0xff, 0xff, 0xff, 0xff)

static void opl_operator_map(void* owner, UINT32* map)
{
	// OPL/OPL2 has a fixed map, all 2 operators
	static const UINT32 s_fixed_map[9] =
	{
		OPS2(  0,  3 ),  // Channel 0 operators
		OPS2(  1,  4 ),  // Channel 1 operators
		OPS2(  2,  5 ),  // Channel 2 operators
		OPS2(  6,  9 ),  // Channel 3 operators
		OPS2(  7, 10 ),  // Channel 4 operators
		OPS2(  8, 11 ),  // Channel 5 operators
		OPS2( 12, 15 ),  // Channel 6 operators
		OPS2( 13, 16 ),  // Channel 7 operators
		OPS2( 14, 17 ),  // Channel 8 operators
	};
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;
	UINT32 fourop;

	if (! IS_OPL3)
	{
		memcpy(map, s_fixed_map, sizeof(s_fixed_map));
		return;
	}

	// OPL3/OPL4 can be configured for 2 or 4 operators
	fourop = fourop_enable();

	map[ 0] = bitfield(fourop, 0, 1) ? fm_operator_list(  0,  3,  6,  9 ) : OPS2(  0,  3 );
	map[ 1] = bitfield(fourop, 1, 1) ? fm_operator_list(  1,  4,  7, 10 ) : OPS2(  1,  4 );
	map[ 2] = bitfield(fourop, 2, 1) ? fm_operator_list(  2,  5,  8, 11 ) : OPS2(  2,  5 );
	map[ 3] = bitfield(fourop, 0, 1) ? OPS_NONE : OPS2(  6,  9 );
	map[ 4] = bitfield(fourop, 1, 1) ? OPS_NONE : OPS2(  7, 10 );
	map[ 5] = bitfield(fourop, 2, 1) ? OPS_NONE : OPS2(  8, 11 );
	map[ 6] = OPS2( 12, 15 );
	map[ 7] = OPS2( 13, 16 );
	map[ 8] = OPS2( 14, 17 );

	map[ 9] = bitfield(fourop, 3, 1) ? fm_operator_list( 18, 21, 24, 27 ) : OPS2( 18, 21 );
	map[10] = bitfield(fourop, 4, 1) ? fm_operator_list( 19, 22, 25, 28 ) : OPS2( 19, 22 );
	map[11] = bitfield(fourop, 5, 1) ? fm_operator_list( 20, 23, 26, 29 ) : OPS2( 20, 23 );
	map[12] = bitfield(fourop, 3, 1) ? OPS_NONE : OPS2( 24, 27 );
	map[13] = bitfield(fourop, 4, 1) ? OPS_NONE : OPS2( 25, 28 );
	map[14] = bitfield(fourop, 5, 1) ? OPS_NONE : OPS2( 26, 29 );
	map[15] = OPS2( 30, 33 );
	map[16] = OPS2( 31, 34 );
	map[17] = OPS2( 32, 35 );
	return;
}

#undef OPS2
#undef OPS_NONE

static void opl_reset(void* owner)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;

	memset(regs->regdata, 0x00, REGISTERS);
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 opl_write(void* owner, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;

	// writes to the mode register with high bit set ignore the low bits
	if (index == REG_MODE && bitfield(data, 7, 1) != 0)
		REG(index) |= 0x80;
	else
		REG(index) = data;

	// handle writes to the rhythm keyons
	if (index == 0xbd)
	{
		*channel = FM_RHYTHM_CHANNEL;
		*opmask = bitfield(data, 5, 1) ? bitfield(data, 0, 5) : 0;
		return 1;
	}

	// handle writes to the channel keyons
	if ((index & 0xf0) == 0xb0)
	{
		*channel = index & 0x0f;
		if (*channel < 9)
		{
			// the second bank (OPL3 only) has channels 9-17
			*channel += 9 * bitfield(index, 8, 1);
			*opmask = bitfield(data, 5, 1) ? 15 : 0;
			return 1;
		}
	}
	return 0;
}

static void opl_read_mode(void* owner, FM_MODE* mode)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;

	mode->timer_a_value = timer_a_value();
	mode->timer_b_value = timer_b_value();
	mode->status_mask = (UINT8)status_mask();
	mode->irq_reset = (UINT8)irq_reset();
	mode->csm = (UINT8)csm();
	mode->reset_timer_a = (UINT8)reset_timer_a();
	mode->reset_timer_b = (UINT8)reset_timer_b();
	// both timers are always enabled on OPL
	mode->enable_timer_a = 1;
	mode->enable_timer_b = 1;
	mode->load_timer_a = (UINT8)load_timer_a();
	mode->load_timer_b = (UINT8)load_timer_b();
	return;
}

// clock the noise and LFO, returning the LFO PM value
static INT32 opl_clock_noise_and_lfo(void* owner, UINT8* lfo_am, UINT8* noise_state)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;
	UINT32 am_counter;
	UINT32 pm_counter;
	int shift;

	// OPL has a 23-bit noise generator for the rhythm section, running at
	// a constant rate, used only for percussion input
	regs->noise_lfsr <<= 1;
	regs->noise_lfsr |= bitfield(regs->noise_lfsr, 23, 1) ^ bitfield(regs->noise_lfsr, 9, 1) ^
		bitfield(regs->noise_lfsr, 8, 1) ^ bitfield(regs->noise_lfsr, 1, 1);
	*noise_state = (UINT8)bitfield(regs->noise_lfsr, 23, 1);

	// OPL has two fixed-frequency LFOs, one for AM, one for PM

	// the AM LFO has 210*64 steps; at a nominal 50kHz output,
	// this equates to a period of 50000/(210*64) = 3.72Hz
	am_counter = regs->lfo_am_counter++;
	if (am_counter >= 210*64 - 1)
		regs->lfo_am_counter = 0;

	// low 8 bits are fractional; depth 0 is divided by 2, while depth 1 is times 2
	shift = 9 - 2 * lfo_am_depth();

	// AM value is the upper bits of the value, inverted across the midpoint
	// to produce a triangle
	*lfo_am = (UINT8)(((am_counter < 105*64) ? am_counter : (210*64+63 - am_counter)) >> shift);

	// the PM LFO has 8192 steps, or a nominal period of 6.1Hz
	pm_counter = regs->lfo_pm_counter++;

	// PM LFO is broken into 8 chunks, each lasting 1024 steps; the PM value
	// depends on the upper bits of FNUM, so this value is a fraction and
	// sign to apply to that value, as a 1.3 value
	return s_lfo_pm_scale[bitfield(pm_counter, 10, 3)] >> (lfo_pm_depth() ^ 1);
}

static void opl_cache_channel_data(void* owner, UINT32 choffs, FM_CHANNEL_CACHE* cache)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;

	cache->feedback = (UINT8)ch_feedback(choffs);
	cache->algorithm = (UINT8)ch_algorithm(choffs);
	cache->outputs = (UINT8)(ch_output_0(choffs) | (ch_output_1(choffs) << 1) |
		(ch_output_2(choffs) << 2) | (ch_output_3(choffs) << 3));

	// OPL has no AM sensitivity: the AM offset is the LFO AM value
	cache->am_lshift = 0;
	cache->am_rshift = 0;

	// in rhythm mode, channels 6-8 (first bank) output the percussion
	// instruments
	cache->kind = FM_CH_NORMAL;
	if (rhythm_enable())
	{
		if (choffs == 6)
			cache->kind = FM_CH_RHYTHM_BD;
		else if (choffs == 7)
			cache->kind = FM_CH_RHYTHM_HH_SD;
		else if (choffs == 8)
			cache->kind = FM_CH_RHYTHM_TT_CY;
	}
	return;
}

// compute the phase step, given a PM value
static UINT32 opl_compute_phase_step(void* owner, UINT32 choffs, UINT32 opoffs, const FM_OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;
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
static void opl_cache_operator_data(void* owner, UINT32 choffs, UINT32 opoffs, FM_OPDATA_CACHE* cache)
{
	OPL_REGISTERS* regs = (OPL_REGISTERS*)owner;
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 multiple;
	UINT32 ksl;
	UINT32 ksrval;

	// set up the easy stuff
	cache->waveform = regs->waveform[op_waveform(opoffs)];
	cache->lfo_am_enable = (UINT8)op_lfo_am_enable(opoffs);
	cache->ssg_eg = 0;

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

	// no detune adjustment on OPL
	cache->detune = 0;

	// multiple value, as an x.1 value (0 means 0.5)
	// replace the low bit with a table lookup to give 0,1,2,3,4,5,6,7,8,9,10,10,12,12,15,15
	multiple = op_multiple(opoffs);
	cache->multiple = ((multiple & 0xe) | bitfield(0xc2aa, multiple, 1)) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on block_freq
	// and multiple, so compute it after we've done those
	if (op_lfo_pm_enable(opoffs) == 0)
		cache->phase_step = opl_compute_phase_step(owner, choffs, opoffs, cache, 0);
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
	cache->eg_rate[EG_ATTACK] = (UINT8)fm_effective_rate(op_attack_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_DECAY] = (UINT8)fm_effective_rate(op_decay_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_SUSTAIN] = op_eg_sustain(opoffs) ? 0 : (UINT8)fm_effective_rate(op_release_rate(opoffs) * 4, ksrval);
	cache->eg_rate[EG_RELEASE] = (UINT8)fm_effective_rate(op_release_rate(opoffs) * 4, ksrval);
	return;
}

// YM3526, Y8950, YM3812: 9 channels, 1 output
static const FM_FAMILY opl_family =
{
	9,					// channels
	1,					// outputs
	9 * 2,				// operators
	4,					// default_prescale
	1,					// eg_clock_divider
	REG_MODE,			// reg_mode
	(1 << 9) - 1,		// csm_trigger_mask
	STATUS_TIMERA,		// status_timera
	STATUS_TIMERB,		// status_timerb
	0,					// status_busy
	STATUS_IRQ,			// status_irq
	0,					// dynamic_ops
	1,					// modulator_delay
	0,					// eg_has_ssg

	opl_channel_offset,
	opl_operator_offset,
	opl_operator_map,
	opl_reset,
	opl_write,
	opl_read_mode,
	opl_clock_noise_and_lfo,
	opl_cache_channel_data,
	opl_cache_operator_data,
	opl_compute_phase_step,
};

// YMF262: 18 channels in 2 banks, 4 outputs, 4-operator pairing
static const FM_FAMILY opl3_family =
{
	18,					// channels
	4,					// outputs
	18 * 2,				// operators
	8,					// default_prescale
	1,					// eg_clock_divider
	REG_MODE,			// reg_mode
	(1 << 18) - 1,		// csm_trigger_mask
	STATUS_TIMERA,		// status_timera
	STATUS_TIMERB,		// status_timerb
	0,					// status_busy
	STATUS_IRQ,			// status_irq
	1,					// dynamic_ops
	0,					// modulator_delay
	0,					// eg_has_ssg

	opl_channel_offset,
	opl_operator_offset,
	opl_operator_map,
	opl_reset,
	opl_write,
	opl_read_mode,
	opl_clock_noise_and_lfo,
	opl_cache_channel_data,
	opl_cache_operator_data,
	opl_compute_phase_step,
};

// set up the registers and the FM engine of one chip
static void opl_engine_init(FM_ENGINE* fm, OPL_REGISTERS* regs, UINT8 revision)
{
	UINT32 index;
	UINT16 zeroval;

	regs->revision = revision;
	regs->lfo_am_counter = 0;
	regs->lfo_pm_counter = 0;
	regs->noise_lfsr = 1;

	// create the waveforms; waveforms 1-3 are for OPL2+, 4-7 for OPL3+
	for (index = 0; index < FM_WAVEFORM_LENGTH; index ++)
		regs->waveform[0][index] = (UINT16)(fm_abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	zeroval = regs->waveform[0][0];
	for (index = 0; index < FM_WAVEFORM_LENGTH; index ++)
	{
		regs->waveform[1][index] = bitfield(index, 9, 1) ? zeroval : regs->waveform[0][index];
		regs->waveform[2][index] = regs->waveform[0][index] & 0x7fff;
		regs->waveform[3][index] = bitfield(index, 8, 1) ? zeroval : (regs->waveform[0][index] & 0x7fff);
		regs->waveform[4][index] = bitfield(index, 9, 1) ? zeroval : regs->waveform[0][index * 2];
		regs->waveform[5][index] = bitfield(index, 9, 1) ? zeroval : regs->waveform[0][(index * 2) & 0x1ff];
		regs->waveform[6][index] = (UINT16)(bitfield(index, 9, 1) << 15);
		regs->waveform[7][index] = (UINT16)((bitfield(index, 9, 1) ? (index ^ 0x13ff) : index) << 3);
	}

	fm_engine_init(fm, IS_OPL3 ? &opl3_family : &opl_family, regs);
	return;
}

#undef REG
#undef IS_OPL3

// the YMF262 "new" flag, for the address port
#define ymf262_newflag(chip)	bitfield((chip)->regs.regdata[0x105], 0, 1)


//----------------------------------------------------------------------------
// YM3526 / Y8950 / YM3812
//----------------------------------------------------------------------------

// read the Y8950 status register, merging in the live ADPCM-B status
static UINT8 y8950_read_status(OPL_STATE* chip)
{
	UINT8 status;
	UINT32 adpcm_status;

	// start with current FM status, masking out bits we might set
	status = fm_engine_status(&chip->fm) & ~(STATUS_ADPCM_B_EOS | STATUS_ADPCM_B_BRDY | STATUS_ADPCM_B_PLAYING);

	// insert the live ADPCM status bits
	adpcm_status = chip->adpcm_b.status;
	if ((adpcm_status & ADPCM_B_STATUS_EOS) != 0)
		status |= STATUS_ADPCM_B_EOS;
	if ((adpcm_status & ADPCM_B_STATUS_BRDY) != 0)
		status |= STATUS_ADPCM_B_BRDY;
	if ((adpcm_status & ADPCM_B_STATUS_PLAYING) != 0)
		status |= STATUS_ADPCM_B_PLAYING;

	// run it through the FM engine to handle interrupts for us
	return fm_engine_set_reset_status(&chip->fm, status, ~status);
}

static void y8950_write_data(OPL_STATE* chip, UINT8 data)
{
	switch (chip->address)
	{
	case 0x04:	// IRQ control
		fm_engine_write(&chip->fm, chip->address, data);
		y8950_read_status(chip);
		break;
	case 0x06:	// keyboard out (not connected)
		break;
	case 0x08:	// split FM/ADPCM-B
		adpcm_b_write(&chip->adpcm_b, chip->address - 0x07, (data & 0x0f) | 0x80);
		fm_engine_write(&chip->fm, chip->address, data & 0xc0);
		break;
	case 0x07:	// ADPCM-B registers
	case 0x09:
	case 0x0a:
	case 0x0b:
	case 0x0c:
	case 0x0d:
	case 0x0e:
	case 0x0f:
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x15:
	case 0x16:
	case 0x17:
		adpcm_b_write(&chip->adpcm_b, chip->address - 0x07, data);
		break;
	case 0x18:	// I/O direction
		chip->io_ddr = data & 0x0f;
		break;
	case 0x19:	// I/O data (not connected)
		break;
	default:	// everything else to FM
		fm_engine_write(&chip->fm, chip->address, data);
		break;
	}
	return;
}

static UINT8 y8950_read_data(OPL_STATE* chip)
{
	switch (chip->address)
	{
	case 0x05:	// keyboard in (not connected)
	case 0x19:	// I/O data (not connected)
		return 0x00;
	case 0x09:	// ADPCM data
	case 0x1a:
		return adpcm_b_read(&chip->adpcm_b, chip->address - 0x07);
	default:
		return 0xff;
	}
}

// generate one sample of sound
static void opl_generate(OPL_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output;

	// clock the system
	fm_engine_clock(&chip->fm, &opl_family, chip->fm.all_channels);
	if (chip->type == CHIP_Y8950)
		adpcm_b_clock(&chip->adpcm_b);

	// update the FM content; mixing details need verification
	output = 0;
	fm_engine_output(&chip->fm, &opl_family, &output, 1, 32767, chip->fm.all_channels);

	// mix in the ADPCM
	if (chip->type == CHIP_Y8950 && ! bitfield(chip->fm.mute_mask, MUTE_ADPCM, 1))
		adpcm_b_output(&chip->adpcm_b, &output, 1, 3);

	// the chips use an external DAC (YM3014) with mantissa/exponent format
	// convert to 10.3 floating point value and back to simulate truncation
	*outL = *outR = fm_roundtrip_fp(output);
	return;
}


//----------------------------------------------------------------------------
// YMF262
//----------------------------------------------------------------------------

// generate one sample of sound
static void ymf262_generate(OPL_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[4];
	UINT8 outnum;

	// clock the system
	fm_engine_clock(&chip->fm, &opl3_family, chip->fm.all_channels);

	// update the FM content; mixing details need verification
	output[0] = output[1] = output[2] = output[3] = 0;
	fm_engine_output(&chip->fm, &opl3_family, output, 0, 32767, chip->fm.all_channels);

	// YMF262 output is 16-bit offset serial via YAC512 DAC
	for (outnum = 0; outnum < 4; outnum ++)
		output[outnum] = clamp(output[outnum], -32768, 32767);

	if (chip->opl4)
	{
		*outL = output[0];
		*outR = output[1];
	}
	else
	{
		*outL = output[0] + output[2];
		*outR = output[1] + output[3];
	}
	if (chip->vol[0] != VOL_UNITY)
		*outL = (*outL * chip->vol[0]) >> 11;
	if (chip->vol[1] != VOL_UNITY)
		*outR = (*outR * chip->vol[1]) >> 11;
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

static UINT8 device_start_opl(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf, UINT8 type, DEV_DEF* devDef)
{
	static const UINT8 s_revision[4] = { 1, 1, 2, 3 };	// CHIP_YM3526, CHIP_Y8950, CHIP_YM3812, CHIP_YMF262
	OPL_STATE* chip;

	chip = (OPL_STATE*)calloc(1, sizeof(OPL_STATE));
	if (chip == NULL)
		return 0xFF;

	chip->type = type;
	opl_engine_init(&chip->fm, &chip->regs, s_revision[type]);
	fm_engine_reset(&chip->fm);
	if (type == CHIP_Y8950)
	{
		adpcm_b_init(&chip->adpcm_b, 0);
		adpcm_b_reset(&chip->adpcm_b);
	}
	if (type == CHIP_YMF262)
		chip->opl4 = (cfg->flags & YMF262_FLAG_OPL4) ? 1 : 0;
	chip->vol[0] = chip->vol[1] = VOL_UNITY;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / (chip->fm.family->default_prescale * chip->fm.family->operators), devDef);
	return 0x00;
}

static UINT8 device_start_ym3526(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opl(cfg, retDevInf, CHIP_YM3526, &devDef3526);
}

static UINT8 device_start_y8950(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opl(cfg, retDevInf, CHIP_Y8950, &devDef8950);
}

static UINT8 device_start_ym3812(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opl(cfg, retDevInf, CHIP_YM3812, &devDef3812);
}

static UINT8 device_start_ymf262(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	return device_start_opl(cfg, retDevInf, CHIP_YMF262, &devDef262);
}

static void device_stop_opl(void* info)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	free(chip->adpcm_b_mem);
	free(chip);
	return;
}

static void device_reset_opl(void* info)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	// ymfm's reset leaves the address and I/O registers alone; a device reset clears them too
	chip->address = 0;
	chip->io_ddr = 0;
	fm_engine_reset(&chip->fm);
	if (chip->type == CHIP_Y8950)
		adpcm_b_reset(&chip->adpcm_b);
	return;
}

static void opl_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	OPL_STATE* chip = (OPL_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		opl_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ymf262_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	OPL_STATE* chip = (OPL_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ymf262_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym3812_w(void* info, UINT8 offset, UINT8 data)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	if (! (offset & 1))
		chip->address = data;	// address port
	else
		fm_engine_write(&chip->fm, chip->address, data);	// data port
	return;
}

static UINT8 ym3812_r(void* info, UINT8 offset)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	// status port; reads with A0=1 are not guaranteed by the datasheet
	if (offset & 1)
		return 0xFF;
	return fm_engine_status(&chip->fm) | 0x06;
}

static void y8950_w(void* info, UINT8 offset, UINT8 data)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	if (! (offset & 1))
		chip->address = data;	// address port
	else
		y8950_write_data(chip, data);	// data port
	return;
}

static UINT8 y8950_r(void* info, UINT8 offset)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	if (! (offset & 1))
		return y8950_read_status(chip);	// status port
	return y8950_read_data(chip);	// data port
}

static void y8950_alloc_pcmrom(void* info, UINT32 memsize)
{
	OPL_STATE* chip = (OPL_STATE*)info;
	UINT8* mem;

	if (chip->adpcm_b_mem_size == memsize)
		return;
	mem = (UINT8*)realloc(chip->adpcm_b_mem, memsize);
	if (mem == NULL && memsize > 0)
		return;
	chip->adpcm_b_mem = mem;
	chip->adpcm_b_mem_size = memsize;
	if (memsize > 0)
		memset(chip->adpcm_b_mem, 0xFF, memsize);
	chip->adpcm_b.mem = chip->adpcm_b_mem;
	chip->adpcm_b.mem_size = chip->adpcm_b_mem_size;
	return;
}

static void y8950_write_pcmrom(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	if (offset > chip->adpcm_b_mem_size)
		return;
	if (offset + length > chip->adpcm_b_mem_size)
		length = chip->adpcm_b_mem_size - offset;
	memcpy(chip->adpcm_b_mem + offset, data, length);
	return;
}

static void ymf262_w(void* info, UINT8 offset, UINT8 data)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	switch (offset & 3)
	{
	case 0:	// address port, bank 0
		chip->address = data;
		break;
	case 2:	// address port, bank 1
		chip->address = data | 0x100;

		// tests reveal that in compatibility mode, upper bit is masked
		// except for register 0x105
		if (ymf262_newflag(chip) == 0 && chip->address != 0x105)
			chip->address &= 0xff;
		break;
	case 1:	// data ports
	case 3:
		fm_engine_write(&chip->fm, chip->address, data);
		break;
	}
	return;
}

static UINT8 ymf262_r(void* info, UINT8 offset)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	// status port; the other offsets are not readable
	if ((offset & 3) != 0)
		return 0xFF;
	return fm_engine_status(&chip->fm);
}

// volume of the stereo output, 0x10000 = 100%
static void ymf262_set_vol_lr(void* info, INT32 volL, INT32 volR)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	chip->vol[0] = volL >> 5;
	chip->vol[1] = volR >> 5;
	return;
}

static void opl_set_mute_mask(void* info, UINT32 MuteMask)
{
	OPL_STATE* chip = (OPL_STATE*)info;

	chip->fm.mute_mask = MuteMask;
	return;
}


//----------------------------------------------------------------------------
// YMF278B
//----------------------------------------------------------------------------

// mix control levels (registers F8/F9): 3 bit value, 0x800 = 100%
static const INT16 s_mix_scale[8] = { 0x7fa, 0x5a4, 0x3fd, 0x2d2, 0x1fe, 0x169, 0xff, 0 };

#define PCM_REG(offset)	(chip->pcm.regdata[offset])
#define mix_fm_r()		bitfield(PCM_REG(0xf8), 3, 3)
#define mix_fm_l()		bitfield(PCM_REG(0xf8), 0, 3)
#define mix_pcm_r()		bitfield(PCM_REG(0xf9), 3, 3)
#define mix_pcm_l()		bitfield(PCM_REG(0xf9), 0, 3)

// set the FM mix level of the linked FM device
static void refresh_fm_volume(const YMF278B_STATE* chip)
{
	if (chip->fm.set_vol != NULL)
		chip->fm.set_vol(chip->fm.param, s_mix_scale[mix_fm_l()] << 5, s_mix_scale[mix_fm_r()] << 5);
	return;
}

static UINT8 read_status(YMF278B_STATE* chip)
{
	UINT8 result;

	// first status read after initialization returns a chip ID, which
	// varies based on the "new" flags, indicating the mode
	if (chip->next_status_id)
	{
		if (chip->fm_reg105 & 0x02)
			result = 0x02;
		else if (chip->fm_reg105 & 0x01)
			result = 0x00;
		else
			result = 0x06;
		chip->next_status_id = 0;
	}
	else
	{
		result = (chip->fm.read != NULL) ? chip->fm.read(chip->fm.param, 0) : 0x00;
		if (chip->load_remaining != 0)
			result |= STATUS_LD;

		// if new2 flag is not set, we're in OPL2 or OPL3 mode
		if (! (chip->fm_reg105 & 0x02))
			result &= ~STATUS_LD;
	}
	return result;
}

static UINT8 read_data_pcm(YMF278B_STATE* chip)
{
	UINT8 result;

	// read from PCM
	if (bitfield(chip->address, 9, 1) != 0)
	{
		result = pcm_engine_read(&chip->pcm, chip->address & 0xff);
		if ((chip->address & 0xff) == 0x02)
			result |= 0x20;
		return result;
	}
	return 0;
}

static void write_data_fm(YMF278B_STATE* chip, UINT8 data)
{
	UINT8 old;
	UINT8 bank;

	// write to FM, which has its own address register
	if (bitfield(chip->address, 9, 1) == 0)
	{
		old = chip->fm_reg105 & 0x02;
		bank = (UINT8)bitfield(chip->address, 8, 1);
		if (chip->address == 0x105)
			chip->fm_reg105 = data;
		if (chip->fm.write != NULL)
		{
			chip->fm.write(chip->fm.param, bank * 2 + 0, (UINT8)(chip->address & 0xff));
			chip->fm.write(chip->fm.param, bank * 2 + 1, data);
		}

		// changing NEW2 from 0->1 causes the next status read to
		// return the chip ID
		if (old == 0 && (chip->fm_reg105 & 0x02))
			chip->next_status_id = 1;
	}
	return;
}

static void write_data_pcm(YMF278B_STATE* chip, UINT8 data)
{
	UINT8 addr;

	// ignore data writes if new2 is not yet set
	if (! (chip->fm_reg105 & 0x02))
		return;

	// write to PCM
	if (bitfield(chip->address, 9, 1) != 0)
	{
		addr = (UINT8)(chip->address & 0xff);
		pcm_engine_write(&chip->pcm, addr, data);

		// writes to the waveform number cause loads to happen for "about 300usec"
		// which is ~13 samples at the nominal output frequency of 44.1kHz
		if (addr >= 0x08 && addr <= 0x1f)
			chip->load_remaining = 13;
		else if (addr == 0xf8)
			refresh_fm_volume(chip);
	}
	return;
}

static void ymf278b_w(void* info, UINT8 offset, UINT8 data)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	switch (offset & 7)
	{
	case 0:	// address port
		chip->address = data;
		break;
	case 1:	// data port
	case 3:
		write_data_fm(chip, data);
		break;
	case 2:	// address port, bank 1
		chip->address = data | 0x100;

		// YMF262, in compatibility mode, treats the upper bit as masked
		// except for register 0x105; assuming YMF278B works the same way?
		if (! (chip->fm_reg105 & 0x01) && chip->address != 0x105)
			chip->address &= 0xff;
		break;
	case 4:	// PCM address port
		chip->address = data | 0x200;
		break;
	case 5:	// PCM data port
		write_data_pcm(chip, data);
		break;
	}
	return;
}

static UINT8 ymf278b_r(void* info, UINT8 offset)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	switch (offset & 7)
	{
	case 0:	// status port
		return read_status(chip);
	case 5:	// PCM data port
		return read_data_pcm(chip);
	}
	return 0xFF;
}

// generate one sample of sound
static void ymf278b_generate(YMF278B_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 pcmout[4];
	UINT8 outnum;

	// clock the system
	pcm_engine_clock(&chip->pcm);

	// update the PCM content
	for (outnum = 0; outnum < 4; outnum ++)
		pcmout[outnum] = 0;
	pcm_engine_output(&chip->pcm, pcmout);

	// DO2 output: wavetable channels 0+1, the FM part is mixed by the linked device
	*outL = clamp((pcmout[0] * s_mix_scale[mix_pcm_l()]) >> 11, -32768, 32767);
	*outR = clamp((pcmout[1] * s_mix_scale[mix_pcm_r()]) >> 11, -32768, 32767);

	// decrement the load waiting count
	if (chip->load_remaining > 0)
		chip->load_remaining --;
	return;
}

static void ymf278b_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ymf278b_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface (YMF278B)
//----------------------------------------------------------------------------

static void init_fm_link(DEV_INFO* devInf, const DEV_GEN_CFG* cfg)
{
	DEVLINK_INFO* devLink;
	DEV_GEN_CFG* fmCfg;

	devInf->linkDevCount = 0;
	devInf->linkDevs = (DEVLINK_INFO*)calloc(1, sizeof(DEVLINK_INFO));
	fmCfg = (DEV_GEN_CFG*)calloc(1, sizeof(DEV_GEN_CFG));
	if (devInf->linkDevs == NULL || fmCfg == NULL)
	{
		free(devInf->linkDevs);
		free(fmCfg);
		devInf->linkDevs = NULL;
		return;
	}

	*fmCfg = *cfg;
	fmCfg->clock = cfg->clock / FM_CLOCK_DEN * FM_CLOCK_NUM + cfg->clock % FM_CLOCK_DEN * FM_CLOCK_NUM / FM_CLOCK_DEN;
	fmCfg->flags = YMF262_FLAG_OPL4;
	fmCfg->emuCore = 0;

	devLink = &devInf->linkDevs[0];
	devLink->devID = DEVID_YMF262;
	devLink->linkID = 0;
	devLink->cfg = fmCfg;
	devInf->linkDevCount = 1;
	return;
}

static UINT8 device_start_ymf278b(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YMF278B_STATE* chip;

	chip = (YMF278B_STATE*)calloc(1, sizeof(YMF278B_STATE));
	if (chip == NULL)
		return 0xFF;

	pcm_engine_init(&chip->pcm);
	pcm_engine_reset(&chip->pcm);
	chip->next_status_id = 1;

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / 768, &devDef278b);
	init_fm_link(retDevInf, cfg);
	return 0x00;
}

static void device_stop_ymf278b(void* info)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	free(chip->ram);
	free(chip->rom);
	free(chip);
	return;
}

static void device_reset_ymf278b(void* info)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	if (chip->fm.reset != NULL)
		chip->fm.reset(chip->fm.param);
	chip->fm_reg105 = 0x00;
	chip->address = 0;
	chip->load_remaining = 0;
	pcm_engine_reset(&chip->pcm);
	refresh_fm_volume(chip);

	// next status read will return ID
	chip->next_status_id = 1;
	return;
}

static UINT8 device_ymf278b_link_fm(void* info, UINT8 linkID, const DEV_INFO* devInfLink)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;
	FM_FUNCS fm;
	UINT8 retVal;

	if (linkID != 0)
		return EERR_UNK_DEVICE;

	memset(&chip->fm, 0x00, sizeof(FM_FUNCS));
	if (devInfLink == NULL)
		return 0x00;

	memset(&fm, 0x00, sizeof(FM_FUNCS));
	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&fm.write);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, (void**)&fm.read);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	retVal = SndEmu_GetDeviceFunc(devInfLink->devDef, RWF_VOLUME_LR | RWF_WRITE, DEVRW_VALUE, 0, (void**)&fm.set_vol);
	if (retVal == EERR_NOT_FOUND)
		return retVal;
	fm.reset = devInfLink->devDef->Reset;
	fm.param = devInfLink->dataPtr;
	chip->fm = fm;

	refresh_fm_volume(chip);
	return 0x00;
}

static void ymf278b_alloc_rom(void* info, UINT32 memsize)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;
	UINT8* mem;

	if (chip->rom_size == memsize)
		return;

	mem = (UINT8*)realloc(chip->rom, memsize);
	if (mem == NULL && memsize > 0)
		return;
	chip->rom = mem;
	chip->rom_size = memsize;
	if (memsize > 0)
		memset(chip->rom, 0xFF, memsize);
	chip->pcm.rom = chip->rom;
	chip->pcm.rom_size = chip->rom_size;
	return;
}

static void ymf278b_alloc_ram(void* info, UINT32 memsize)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;
	UINT8* mem;

	if (chip->ram_size == memsize)
		return;

	mem = (UINT8*)realloc(chip->ram, memsize);
	if (mem == NULL && memsize > 0)
		return;
	chip->ram = mem;
	chip->ram_size = memsize;
	if (memsize > 0)
		memset(chip->ram, 0x00, memsize);
	chip->pcm.ram = chip->ram;
	chip->pcm.ram_size = chip->ram_size;
	return;
}

static void ymf278b_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	if (offset > chip->rom_size)
		return;
	if (offset + length > chip->rom_size)
		length = chip->rom_size - offset;

	memcpy(chip->rom + offset, data, length);
	return;
}

static void ymf278b_write_ram(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	if (offset > chip->ram_size)
		return;
	if (offset + length > chip->ram_size)
		length = chip->ram_size - offset;

	memcpy(chip->ram + offset, data, length);
	return;
}

static void ymf278b_set_mute_mask(void* info, UINT32 MuteMask)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	chip->pcm.mute_mask = MuteMask;
	return;
}
