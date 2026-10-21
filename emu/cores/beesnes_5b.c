/*
	MIT License

	Copyright (c) 2025 Shawn (L. Spiro) Wilcoxen
	Copyright (c) 2026 Purpbatboi (C port)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

	Sunsoft 5B expansion audio

	C port of the 5B audio of L-Spiro's BeesNES (Src/Mappers/LSNAudio5b.h, CAudio5b) and of the gain its
	mapper gives it in the mix (Src/Mappers/LSNMapper069.h, GetExtAudio).
	https://github.com/L-Spiro/BeesNES

	Port notes:
	- The C++ class is a mapper part: it maps its registers onto the CPU bus ($C000-$DFFF register select,
	  $E000-$FFFF data) and is ticked by the mapper every CPU cycle. Here it is a libvgm device. The owner
	  (the NSF player) applies the register writes with the CPU address and renders the device up to the
	  write's sample first.
	- The tone counters step every 16 CPU cycles (a channel toggles when its counter reaches its 12-bit
	  period) and the noise register every 32 (a 17-bit shift register with taps on bits 0 and 3, a step
	  when its counter reaches the 5-bit period). Channel A-C: the tone and noise disable bits of register
	  7 select tone, noise, tone and noise, or (both disabled) the constant volume; the 4-bit volume goes
	  through a 16 entry level table (the C++'s PolynomialVol, a fit of measured output levels); the three
	  levels are added and the sum is inverted and scaled by 0.4177518188953399658203125 (the C++'s
	  LSN_SUNSOFT_5B_REL_VOL), the level of the 5B next to the APU's 1.0 in the mapper's mix. The sum is
	  negative, as in the C++.
	- The level table is a literal here. The C++ computes it when the object is made, with a degree 14
	  polynomial in double (Horner) rounded to float; the literals are those 16 floats (the test checks
	  them bit for bit), so the output no longer depends on how a compiler fuses multiply-adds.
	- Register writes store their value masked the way the C++ does (tone high 4 bits, noise period 5 bits,
	  register 7 6 bits, volume registers 5 bits, envelope shape 4 bits); a data write only counts when the
	  upper 4 bits of the selected register number are 0. The registers 0 / 2 / 4 (tone low bytes), 11 /
	  12 (envelope period) and 14 / 15 (I/O ports) store the full byte.
	- Not implemented in the C++ either, and so not here: the envelope generator. The envelope enable bit
	  (bit 4 of registers 8-10), the envelope period and shape (registers 11-13) are stored and have no effect,
	  every channel plays at its 4-bit volume. (Gimmick!, the 5B game, does not need it; the NSF format notes
	  that most players leave it out.) The noise and the tone/noise mixing are there.
	- Deliberate changes from the C++:
	  - The output stage is the one of the BeesNES APU (nes_expout.h): the level goes to a band-limited
	    buffer at the CPU clock and out through the console filter, at the output rate. The C++'s own
	    stage (the APU's mixer, low-pass and output poles) is not part of the port. The level is the
	    inverted sum times NES_EXPOUT_LEVEL_ONE, the scale of the APU.
	  - PostProcessSample is not ported. In BeesNES it is a soft limiter on the final mix (the APU and the 5B
	    together, after filtering and down-sampling, with a running average of its gain), which a device
	    that produces only its own signal cannot be. Without it a loud 5B is not compressed as in BeesNES.
	  - Reset starts from the power-on state: all registers, tone and noise counters and the divider 0, the
	  	tone outputs 0, the noise register 1 (its initial value in the C++). The C++ never resets it (the
	  	mapper's reset is ResetSoft(), which does nothing, and the constructor leaves the counters and
	  	registers unset).
	  - A mute mask (bit 0-2) sets the level of a tone channel to 0 in the sum. The channel still runs.
	  - A write is ignored below $C000.
	  - The device runs at any output rate, up to the CPU clock / 4 (a higher rate is resampled by libvgm),
	    like the BeesNES APU device.
	- The level is worked out when something it depends on changes: after the tone steps (every 16 cycles),
	  after a register write, a mute change and a reset. Between those the C++'s Sample() gives the same value
	  every cycle (it only reads the registers, the tone outputs and the noise bit), so the output is the same.
	- Not ported: ApplyMap (the decoding is in the write function), ResetSoft (empty), ResetFull's masking of
	  the tone registers (they are masked when written), the volume table m_fVolTable with its natural
	  cubic spline (PrepareNaturalCubicSpline, EvaluateSpline: filled by the C++'s constructor and never
	  read) and Ideal4Bit (never called), PostProcessSample (above).
*/

#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../../common_def.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "nes_expout.h"
#include "beesnes_5b.h"


static UINT8 device_start_5b(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_5b(void* info);
static void n5b_reset(void* info);
static void n5b_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void n5b_w(void* info, UINT16 addr, UINT8 data);
static void n5b_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, 0, n5b_w},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, n5b_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"Sunsoft 5B", "BeesNES", FCC_BNES,

	device_start_5b,
	device_stop_5b,
	n5b_reset,
	n5b_update,

	NULL,	// SetOptionBits
	n5b_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "Sunsoft 5B";
}

#define DEV_CHN_COUNT	3
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT] =
	{
		"Tone A", "Tone B", "Tone C",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_Sunsoft5B =
{
	DEVID_SUNSOFT5B,
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

#define N5B_DEFAULT_CLOCK	1789773	// NTSC
/* The level of the 5B next to the APU's 1.0 (the mapper's LSN_SUNSOFT_5B_REL_VOL) */
#define N5B_REL_VOL			0.4177518188953399658203125f

/* The registers (the AY-3-8910 layout): $00/$01 $02/$03 $04/$05 tone period A/B/C (low byte, high 4 bits),
   $06 noise period, $07 noise (bit 3-5) and tone (bit 0-2) disable of C/B/A, $08-$0A envelope enable and volume,
   $0B/$0C envelope period, $0D envelope shape, $0E/$0F I/O ports. */
#define N5B_REG_NOISE_PERIOD	0x06
#define N5B_REG_DISABLE			0x07
#define N5B_REG_VOLUME			0x08

/* The level of a volume: the C++'s PolynomialVol(0..15), as floats */
static const float vol_table[16] =
{
	0.0f, 0.00563480565f, 0.00887339935f, 0.0137352003f,
	0.0195317268f, 0.0301277973f, 0.0440568551f, 0.0604445972f,
	0.0861640498f, 0.122611821f, 0.183268964f, 0.25392583f,
	0.363758147f, 0.502022922f, 0.707106769f, 0.924697459f
};

/* What is written to each register is masked to the bits it has (the C++'s switch in WriteRegisterE000_FFFF) */
static const UINT8 reg_mask[16] =
{
	0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0x1F, 0x3F,
	0x1F, 0x1F, 0x1F, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF
};

/** A tone channel. */
typedef struct n5b_tone
{
	UINT16 counter;	/**< The channel counter. */
	UINT8 on_off;	/**< Whether the pulse is 0 or non-0. */
} N5B_TONE;

typedef struct n5b_chip
{
	DEV_DATA _devData;

	NES_EXPOUT out;

	N5B_TONE tone[3];		/**< The tone channels. */
	UINT32 noise;			/**< The noise shift register. */
	UINT16 noise_counter;	/**< The noise counter. */
	UINT8 regs[16];			/**< Each register. */
	UINT8 divider;			/**< The divider. */
	UINT8 reg;				/**< The register to which to write. */
	INT32 level;			/**< The output level (NES_EXPOUT_LEVEL_ONE scale). */

	UINT8 mute_mask;
} N5B_CHIP;


/**
 * The output of tone channel `ch`: the level of its volume, or 0 while the channel is off.
 */
INLINE float channel_output(const N5B_CHIP* chip, UINT8 ch)
{
	const UINT8 tone_off = (UINT8)(1 << ch);		// register 7: tone disable
	const UINT8 noise_off = (UINT8)(8 << ch);		// register 7: noise disable
	const UINT8 disable = chip->regs[N5B_REG_DISABLE] & (tone_off | noise_off);
	const UINT8 vol = chip->regs[N5B_REG_VOLUME + ch] & 0x0F;
	const UINT8 noise = (UINT8)(chip->noise & 1);

	if (chip->mute_mask & (1 << ch))
		return 0.0f;
	if (disable == (tone_off | noise_off))
		return vol_table[vol];	// both disabled: constant volume
	if (disable == 0)
		return vol_table[vol * (chip->tone[ch].on_off & noise)];	// tone and noise
	if (! (disable & tone_off))
		return vol_table[vol * chip->tone[ch].on_off];	// tone only
	return vol_table[vol * noise];	// noise only
}

/**
 * The extended-audio sample: the three channels added, inverted and scaled.
 */
INLINE void n5b_update_level(N5B_CHIP* chip)
{
	float sample = 0.0f;

	sample += channel_output(chip, 0);
	sample += channel_output(chip, 1);
	sample += channel_output(chip, 2);
	sample = sample * -N5B_REL_VOL;
	chip->level = (INT32)(sample * NES_EXPOUT_LEVEL_ONE + 0.5f);
}

/**
 * Selects ($C000-$DFFF) and writes ($E000-$FFFF) an audio register.
 */
static void n5b_w(void* info, UINT16 addr, UINT8 data)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;

	if (addr >= 0xE000)
	{
		if ((chip->reg & 0xF0) == 0)
		{
			UINT8 reg = chip->reg & 0x0F;

			chip->regs[reg] = data & reg_mask[reg];
			n5b_update_level(chip);
		}
	}
	else if (addr >= 0xC000)
	{
		chip->reg = data;
	}
}

/**
 * Advances a tone by 1 step.
 */
INLINE void tone_tick(N5B_CHIP* chip, UINT8 ch)
{
	N5B_TONE* tone = &chip->tone[ch];
	UINT16 period = (UINT16)(chip->regs[ch * 2] | ((UINT16)chip->regs[ch * 2 + 1] << 8));

	if (++ tone->counter >= period)
	{
		tone->counter = 0;
		tone->on_off = ! tone->on_off;
	}
}

/**
 * Advances the noise by 1 step.
 */
INLINE void noise_tick(N5B_CHIP* chip)
{
	if (++ chip->noise_counter >= chip->regs[N5B_REG_NOISE_PERIOD])
	{
		chip->noise_counter = 0;
		chip->noise ^= (((chip->noise & 1) ^ ((chip->noise >> 3) & 1)) << 17);
		chip->noise >>= 1;
	}
}

/**
 * One CPU cycle. Returns 1 when the tones and the noise were stepped (the level may have changed).
 */
INLINE UINT8 n5b_tick(N5B_CHIP* chip)
{
	if ((++ chip->divider & 0xF) == 0)
	{
		// Easy to unroll loop.
		tone_tick(chip, 0);
		tone_tick(chip, 1);
		tone_tick(chip, 2);
		if ((chip->divider & 0x1F) == 0)
			noise_tick(chip);
		n5b_update_level(chip);
		return 1;
	}
	return 0;
}

/* Clocks the chip `cycles` times, adding each change of the level as a band-limited step. The level is also
   added at the first cycle: it can have been changed by a register write since the last call. */
static void n5b_run(void* info, UINT32 cycles)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;
	UINT32 cycle;

	for (cycle = 0; cycle < cycles; cycle ++)
	{
		if (n5b_tick(chip) || cycle == 0)
			nes_expout_add(&chip->out, cycle, chip->level);
	}
}

static void n5b_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;

	nes_expout_render(&chip->out, chip, n5b_run, samples, outputs);
}


//----------------------------------------------------------------------------
// libvgm interface
//----------------------------------------------------------------------------

static UINT8 device_start_5b(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	N5B_CHIP* chip;
	UINT32 clock;
	UINT32 rate;

	clock = cfg->clock ? cfg->clock : N5B_DEFAULT_CLOCK;
	// The device synthesizes band-limited output at any rate, so it renders at the output rate
	// directly. Above clock/4 it would gain nothing, so libvgm resamples from there instead.
	rate = cfg->smplRate;
	if (rate == 0 || rate > clock / 4)
		rate = clock / 4;

	chip = (N5B_CHIP*)calloc(1, sizeof(N5B_CHIP));
	if (chip == NULL)
		return 0xFF;
	if (nes_expout_init(&chip->out, clock, rate))
	{
		free(chip);
		return 0xFF;
	}

	n5b_reset(chip);
	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef);
	return 0x00;
}

static void device_stop_5b(void* info)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;

	nes_expout_free(&chip->out);
	free(chip);
}

static void n5b_reset(void* info)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;

	memset(chip->tone, 0x00, sizeof(chip->tone));
	chip->noise = 1;
	chip->noise_counter = 0;
	memset(chip->regs, 0x00, sizeof(chip->regs));
	chip->divider = 0;
	chip->reg = 0;
	n5b_update_level(chip);

	nes_expout_reset(&chip->out, chip->level);
}

static void n5b_set_mute_mask(void* info, UINT32 MuteMask)
{
	N5B_CHIP* chip = (N5B_CHIP*)info;

	chip->mute_mask = (UINT8)(MuteMask & 0x07);
	n5b_update_level(chip);
}
