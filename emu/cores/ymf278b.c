// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Yamaha YMF278B (OPL4)

    C port of the PCM part of ymfm (ymfm_pcm.cpp and ymf278b in ymfm_opl.cpp).
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
    - This core is the wavetable (PCM) part and the register interface. The FM
      part is a linked YMF262 device (ymf262.c) that runs at clock * 8 / 19
      with YMF262_FLAG_OPL4, which is the OPL4 FM engine rate (19 * 36 cycles
      per sample). Register writes of the FM banks are forwarded to it, its
      status is merged into the status register and the FM mix level of the
      mix control (registers F8/F9) is set as its volume.
    - ymfm clocks the FM engine once per 44.1 kHz sample plus an extra tick
      every few samples and drops the output of the extra ticks. Here the FM
      runs at its own rate and the player resamples it. This also computes
      the feedback of every tick, which ymfm skips on the extra ticks.
    - The wavetable part outputs one sample per clock / 768. Only the
      DO2 mix of ymfm (output channel 0 of the PCM channels) is audible, the
      DO0 and DO1 pins are not mixed. The FM and PCM levels are rounded
      separately: (fm * level >> 11) + (pcm * level >> 11), ymfm rounds the sum.
    - Memory: ymfm leaves it to the host. This core has the ROM (the YRW801
      is 2 MB) at address 0 and the SRAM of the MoonSound at 0x200000 (after
      the ROM if that is bigger). The address space wraps at 4 MB. The memory
      type bit of register 2 (SRAM mapped to 0x380000-0x3FFFFF) is not
      emulated. The ROM is not part of this core: the host loads it with the
      0x524F memory function, the RAM with 0x5241.
    - There is no busy flag. The LD flag and the chip ID of the first status
      read are emulated.
    - Octave -8 of a channel is not defined by ymfm (negative shift); here it
      shifts the f-number right by one more bit than octave -7.
    - Change from ymfm: ymfm samples the key state once per sample, so a key
      off followed by a key on before the next sample was lost and the
      note did not restart. It also kept a key on that was followed by a key
      off. Here the live key state is kept, and a key off is remembered
      until the next sample, so the channel releases and then attacks again.

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

#include "ymf278b.h"
#include "ymf262.h"


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

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "YMF278B";
}

#define DEV_CHN_COUNT	24
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT] =
	{
		"PCM 1", "PCM 2", "PCM 3", "PCM 4", "PCM 5", "PCM 6", "PCM 7", "PCM 8",
		"PCM 9", "PCM 10", "PCM 11", "PCM 12", "PCM 13", "PCM 14", "PCM 15", "PCM 16",
		"PCM 17", "PCM 18", "PCM 19", "PCM 20", "PCM 21", "PCM 22", "PCM 23", "PCM 24",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	static const DEVLINK_IDS dlIDs = {1, {DEVID_YMF262}};
	return &dlIDs;
}

const DEV_DECL sndDev_YMF278B =
{
	DEVID_YMF278B,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef278b,
		NULL
	}
};


#define CHANNELS		24
#define ALL_CHANNELS	((1 << CHANNELS) - 1)
#define REGISTERS		0x100

#define STATUS_LD		0x02

// the FM part runs 19 * 36 cycles per sample instead of the OPL3's 8 * 36
#define FM_CLOCK_NUM	8
#define FM_CLOCK_DEN	19

// the address space of the chip wraps at 4 MB, the SRAM starts at 2 MB
#define ADDRESS_MASK	0x3FFFFF
#define RAM_BASE		0x200000

// envelope states
#define EG_ATTACK	1
#define EG_DECAY	2
#define EG_SUSTAIN	3
#define EG_RELEASE	4
#define EG_REVERB	5
#define EG_STATES	6

// "quiet" value, used to optimize when we can skip doing working
#define EG_QUIET	0x200

// key states
#define KEY_ON			0x01
#define KEY_PENDING_ON	0x02
#define KEY_PENDING		0x04

// data computed once at the start of clocking that remains static during
// subsequent sound generation
typedef struct _pcm_cache
{
	UINT32 step;			// sample position step, as a .16 value
	UINT32 total_level;		// target total level, as a .10 value
	UINT32 pan_left;		// left panning attenuation
	UINT32 pan_right;		// right panning attenuation
	UINT32 eg_sustain;		// sustain level, shifted up to envelope values
	UINT8 eg_rate[EG_STATES];	// envelope rate, including KSR
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

	// PCM registers and engine
	UINT8 regdata[REGISTERS];	// register data
	UINT32 env_counter;		// envelope counter
	UINT32 modified_channels;	// bitmask of modified channels
	UINT32 active_channels;	// bitmask of active channels
	UINT32 prepare_count;	// counter to do periodic prepare sweeps
	PCM_CHANNEL channel[CHANNELS];

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

	UINT32 muteMask;
} YMF278B_STATE;


//----------------------------------------------------------------------------
// tables
//----------------------------------------------------------------------------

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

// LFO stepping values, how much to add to a running x.18 value for the LFO
static const UINT8 s_lfo_steps[8] = { 1, 12, 19, 25, 31, 35, 37, 42 };

// AM LFO depth values; each has at most 2 bits to make the "multiply" easy
static const UINT8 s_am_depth[8] = { 0, 0x14, 0x20, 0x28, 0x30, 0x40, 0x50, 0x80 };

// PM LFO depth values, converted from cents into f-numbers
static const UINT8 s_pm_depth[8] = { 0, 2, 3, 4, 6, 12, 24, 48 };

// mix control levels (registers F8/F9): 3 bit value, 0x800 = 100%
static const INT16 s_mix_scale[8] = { 0x7fa, 0x5a4, 0x3fd, 0x2d2, 0x1fe, 0x169, 0xff, 0 };


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

// sign-extend a 4-bit value
INLINE INT32 sign_extend_4(UINT32 value)
{
	return (value & 0x08) ? (INT32)value - 16 : (INT32)value;
}


//----------------------------------------------------------------------------
// memory
//----------------------------------------------------------------------------

// returns the RAM offset of an address, or (UINT32)-1 if it is not mapped
static UINT32 ram_offset(const YMF278B_STATE* chip, UINT32 address)
{
	UINT32 base;
	UINT32 offset;

	base = (chip->rom_size > RAM_BASE) ? chip->rom_size : RAM_BASE;
	if (address < base)
		return (UINT32)-1;
	offset = address - base;

	// 640 KB are a 512 KB chip and a 128 KB chip; the 128 KB chip is visible
	// four times in the upper part
	if (chip->ram_size == 640 * 1024 && (offset & 0x080000))
		offset &= ~0x060000;
	return (offset < chip->ram_size) ? offset : (UINT32)-1;
}

static UINT8 mem_read(const YMF278B_STATE* chip, UINT32 address)
{
	UINT32 offset;

	address &= ADDRESS_MASK;
	if (address < chip->rom_size)
		return chip->rom[address];
	offset = ram_offset(chip, address);
	if (offset != (UINT32)-1)
		return chip->ram[offset];
	return 0xFF;
}

static void mem_write(YMF278B_STATE* chip, UINT32 address, UINT8 data)
{
	UINT32 offset;

	address &= ADDRESS_MASK;
	if (address < chip->rom_size)
		return;	// can't write to ROM
	offset = ram_offset(chip, address);
	if (offset != (UINT32)-1)
		chip->ram[offset] = data;
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

#define REG(offset)		(chip->regdata[offset])

// system-wide registers
#define memory_access_mode()	bitfield(REG(0x02), 0, 1)
#define wave_table_header()		bitfield(REG(0x02), 2, 3)
#define memory_address()		((bitfield(REG(0x03), 0, 6) << 16) | (REG(0x04) << 8) | REG(0x05))
#define mix_fm_r()				bitfield(REG(0xf8), 3, 3)
#define mix_fm_l()				bitfield(REG(0xf8), 0, 3)
#define mix_pcm_r()				bitfield(REG(0xf9), 3, 3)
#define mix_pcm_l()				bitfield(REG(0xf9), 0, 3)

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
static void regs_reset(YMF278B_STATE* chip)
{
	memset(chip->regdata, 0x00, REGISTERS);
	REG(0xf8) = 0x1b;
	return;
}

// return the memory address and increment it
static UINT32 regs_memory_address_autoinc(YMF278B_STATE* chip)
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
static void regs_cache_channel_data(const YMF278B_STATE* chip, UINT32 choffs, PCM_CACHE* cache)
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
	cache->eg_rate[EG_ATTACK] = (UINT8)effective_rate(ch_attack_rate(choffs), correction);
	cache->eg_rate[EG_DECAY] = (UINT8)effective_rate(ch_decay_rate(choffs), correction);
	cache->eg_rate[EG_SUSTAIN] = (UINT8)effective_rate(ch_sustain_rate(choffs), correction);
	cache->eg_rate[EG_RELEASE] = (UINT8)effective_rate(ch_release_rate(choffs), correction);
	cache->eg_rate[EG_REVERB] = 5;

	// if damping is on, override some things; essentially decay at a hardcoded
	// rate of 48 until -12db (0x80), then at maximum rate for the rest
	if (ch_damp(choffs) != 0)
	{
		cache->eg_rate[EG_DECAY] = 48;
		cache->eg_rate[EG_SUSTAIN] = 63;
		cache->eg_rate[EG_RELEASE] = 63;
		cache->eg_sustain = 0x80;
	}
	return;
}


//----------------------------------------------------------------------------
// PCM channel
//----------------------------------------------------------------------------

static void channel_reset(PCM_CHANNEL* ch)
{
	ch->baseaddr = 0;
	ch->endpos = 0;
	ch->looppos = 0;
	ch->curpos = 0;
	ch->nextpos = 0;
	ch->lfo_counter = 0;
	ch->eg_state = EG_RELEASE;
	ch->env_attenuation = 0x3ff;
	ch->total_level = 0x7f << 10;
	ch->format = 0;
	ch->key_state = 0;
	ch->keyoff_pending = 0;
	return;
}

// start the attack phase
static void channel_start_attack(YMF278B_STATE* chip, PCM_CHANNEL* ch, UINT32 choffs)
{
	// don't change anything if already in attack state
	if (ch->eg_state == EG_ATTACK)
		return;
	ch->eg_state = EG_ATTACK;

	// reset the LFO if requested
	if (ch_lfo_reset(choffs))
		ch->lfo_counter = 0;

	// if the attack rate == 63 then immediately go to max attenuation
	if (ch->cache.eg_rate[EG_ATTACK] == 63)
		ch->env_attenuation = 0;

	// reset the positions
	ch->curpos = ch->nextpos = 0;
	return;
}

// start the release phase
static void channel_start_release(PCM_CHANNEL* ch)
{
	// don't change anything if already in release or reverb state
	if (ch->eg_state >= EG_RELEASE)
		return;
	ch->eg_state = EG_RELEASE;
	return;
}

// prepare for clocking; returns non-zero if the channel is active
static UINT8 channel_prepare(YMF278B_STATE* chip, PCM_CHANNEL* ch, UINT32 choffs)
{
	UINT8 oldon;
	UINT8 newon;

	// cache the data
	regs_cache_channel_data(chip, choffs, &ch->cache);

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
				channel_start_attack(chip, ch, choffs);
			else
				channel_start_release(ch);
		}
	}

	// set the total level directly if not interpolating
	if (ch_level_direct(choffs))
		ch->total_level = ch->cache.total_level;

	// we're active until we're quiet after the release
	return (ch->eg_state < EG_RELEASE || ch->env_attenuation < EG_QUIET);
}

// clock the envelope generator
static void channel_clock_envelope(const YMF278B_STATE* chip, PCM_CHANNEL* ch, UINT32 choffs, UINT32 env_counter)
{
	UINT32 rate;
	UINT32 rate_shift;
	UINT32 relevant_bits;
	UINT32 increment;

	// handle attack->decay transitions
	if (ch->eg_state == EG_ATTACK && ch->env_attenuation == 0)
		ch->eg_state = EG_DECAY;

	// handle decay->sustain transitions
	if (ch->eg_state == EG_DECAY && ch->env_attenuation >= ch->cache.eg_sustain)
		ch->eg_state = EG_SUSTAIN;

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
	if (ch->eg_state == EG_ATTACK)
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
		if (ch->env_attenuation >= 0xc0 && ch->eg_state < EG_REVERB && ch_pseudo_reverb(choffs))
			ch->eg_state = EG_REVERB;
	}
	return;
}

// master clocking function
static void channel_clock(const YMF278B_STATE* chip, PCM_CHANNEL* ch, UINT32 choffs, UINT32 env_counter)
{
	UINT32 step;
	UINT32 lfo_shifted;
	INT32 lfo_value;

	// clock the LFO, which is an x.18 value incremented based on the
	// LFO speed value
	ch->lfo_counter += ch->cache.lfo_step;

	// clock the envelope
	channel_clock_envelope(chip, ch, choffs, env_counter);

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
static INT16 channel_fetch_sample(const YMF278B_STATE* chip, const PCM_CHANNEL* ch)
{
	UINT32 addr;
	UINT32 pos;

	addr = ch->baseaddr;
	pos = ch->curpos >> 16;

	// 8-bit PCM: shift up by 8
	if (ch->format == 0)
		return (INT16)(mem_read(chip, addr + pos) << 8);

	// 16-bit PCM: assemble from 2 halves
	if (ch->format == 2)
	{
		addr += pos * 2;
		return (INT16)((mem_read(chip, addr) << 8) | mem_read(chip, addr + 1));
	}

	// 12-bit PCM: assemble out of half of 3 bytes
	addr += (pos / 2) * 3;
	if ((pos & 1) == 0)
		return (INT16)((mem_read(chip, addr + 0) << 8) | ((mem_read(chip, addr + 1) << 4) & 0xf0));
	else
		return (INT16)((mem_read(chip, addr + 2) << 8) | ((mem_read(chip, addr + 1) << 0) & 0xf0));
}

// add the computed output value, with panning applied, to the 4 outputs
static void channel_output(const YMF278B_STATE* chip, const PCM_CHANNEL* ch, UINT32 choffs, INT32* output)
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
	if (envelope > EG_QUIET)
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
	sample = channel_fetch_sample(chip, ch);
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
// PCM engine
//----------------------------------------------------------------------------

static void engine_init(YMF278B_STATE* chip)
{
	UINT8 curChn;

	chip->env_counter = 0;
	chip->modified_channels = ALL_CHANNELS;
	chip->active_channels = ALL_CHANNELS;
	chip->prepare_count = 0;
	for (curChn = 0; curChn < CHANNELS; curChn ++)
		channel_reset(&chip->channel[curChn]);
	return;
}

// reset the engine state
static void engine_reset(YMF278B_STATE* chip)
{
	UINT8 curChn;

	// reset register state
	regs_reset(chip);

	// reset each channel
	for (curChn = 0; curChn < CHANNELS; curChn ++)
		channel_reset(&chip->channel[curChn]);
	return;
}

// load a wavetable by fetching its data from external memory
static void engine_write(YMF278B_STATE* chip, UINT32 regnum, UINT8 data);
static void channel_load_wavetable(YMF278B_STATE* chip, PCM_CHANNEL* ch, UINT32 choffs)
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
	byte = mem_read(chip, wavheader + 0);
	ch->format = (UINT8)bitfield(byte, 6, 2);
	ch->baseaddr = bitfield(byte, 0, 6) << 16;
	ch->baseaddr |= mem_read(chip, wavheader + 1) << 8;
	ch->baseaddr |= mem_read(chip, wavheader + 2) << 0;

	// fetch the 16-bit loop position
	ch->looppos = mem_read(chip, wavheader + 3) << 8;
	ch->looppos |= mem_read(chip, wavheader + 4);
	ch->looppos <<= 16;

	// fetch the 16-bit end position, which is stored as a negative value
	// for some reason that is unclear
	ch->endpos = mem_read(chip, wavheader + 5) << 8;
	ch->endpos |= mem_read(chip, wavheader + 6);
	ch->endpos = (0 - ch->endpos) << 16;

	// remaining data values set registers
	engine_write(chip, 0x80 + choffs, mem_read(chip, wavheader + 7));
	engine_write(chip, 0x98 + choffs, mem_read(chip, wavheader + 8));
	engine_write(chip, 0xb0 + choffs, mem_read(chip, wavheader + 9));
	engine_write(chip, 0xc8 + choffs, mem_read(chip, wavheader + 10));
	engine_write(chip, 0xe0 + choffs, mem_read(chip, wavheader + 11));

	// reset the envelope so we don't continue playing mid-sample from previous key ons
	ch->env_attenuation = 0x3ff;
	return;
}

// handle writes to the PCM registers
static void engine_write(YMF278B_STATE* chip, UINT32 regnum, UINT8 data)
{
	// handle writes to the data register
	if (regnum == 0x06 && memory_access_mode() != 0)
	{
		mem_write(chip, regs_memory_address_autoinc(chip), data);
		return;
	}

	// for now just mark all channels as modified
	chip->modified_channels = ALL_CHANNELS;

	// most writes are passive, consumed only when needed
	REG(regnum) = data;

	// however, process keyons immediately
	if (regnum >= 0x68 && regnum <= 0x7f)
		channel_keyonoff(&chip->channel[regnum - 0x68], bitfield(data, 7, 1));

	// and also wavetable writes
	else if (regnum >= 0x08 && regnum <= 0x1f)
		channel_load_wavetable(chip, &chip->channel[regnum - 0x08], regnum - 0x08);
	return;
}

// handle reads from the PCM registers
static UINT8 engine_read(YMF278B_STATE* chip, UINT32 regnum)
{
	// handle reads from the data register
	if (regnum == 0x06 && memory_access_mode() != 0)
		return mem_read(chip, regs_memory_address_autoinc(chip));

	return REG(regnum);
}

// master clocking function
static void engine_clock(YMF278B_STATE* chip)
{
	UINT8 chnum;

	// if something was modified, prepare
	// also prepare every 4k samples to catch ending notes
	if (chip->modified_channels != 0 || chip->prepare_count++ >= 4096)
	{
		// call each channel to prepare
		chip->active_channels = 0;
		for (chnum = 0; chnum < CHANNELS; chnum ++)
		{
			if (channel_prepare(chip, &chip->channel[chnum], chnum))
				chip->active_channels |= 1 << chnum;
		}

		// reset the modified channels and prepare count
		chip->modified_channels = chip->prepare_count = 0;
	}

	// increment the envelope counter; the envelope generator
	// only clocks every other sample in order to make the PCM
	// envelopes line up with the FM envelopes (after taking into
	// account the different FM sampling rate)
	chip->env_counter ++;

	// now update the state of all the channels
	for (chnum = 0; chnum < CHANNELS; chnum ++)
		channel_clock(chip, &chip->channel[chnum], chnum, chip->env_counter >> 1);
	return;
}

// compute the sum of the channel outputs into the 4 outputs; muted channels
// are skipped here, as their state is kept by the clocking
static void engine_output(const YMF278B_STATE* chip, INT32* output)
{
	UINT8 chnum;

	for (chnum = 0; chnum < CHANNELS; chnum ++)
	{
		if ((chip->muteMask >> chnum) & 0x01)
			continue;
		channel_output(chip, &chip->channel[chnum], chnum, output);
	}
	return;
}


//----------------------------------------------------------------------------
// YMF278B
//----------------------------------------------------------------------------

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
		result = engine_read(chip, chip->address & 0xff);
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
		engine_write(chip, addr, data);

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
	engine_clock(chip);

	// update the PCM content
	for (outnum = 0; outnum < 4; outnum ++)
		pcmout[outnum] = 0;
	engine_output(chip, pcmout);

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
// libvgm device interface
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

	engine_init(chip);
	engine_reset(chip);
	chip->next_status_id = 1;
	chip->muteMask = 0x00;

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
	engine_reset(chip);
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

	if (chip->rom_size == memsize)
		return;

	chip->rom = (UINT8*)realloc(chip->rom, memsize);
	chip->rom_size = memsize;
	memset(chip->rom, 0xFF, memsize);
	return;
}

static void ymf278b_alloc_ram(void* info, UINT32 memsize)
{
	YMF278B_STATE* chip = (YMF278B_STATE*)info;

	if (chip->ram_size == memsize)
		return;

	chip->ram = (UINT8*)realloc(chip->ram, memsize);
	chip->ram_size = memsize;
	memset(chip->ram, 0x00, memsize);
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

	chip->muteMask = MuteMask;
	return;
}
