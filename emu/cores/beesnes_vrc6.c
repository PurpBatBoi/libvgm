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

	Konami VRC6 expansion audio

	C port of the VRC6 audio of L-Spiro's BeesNES (Src/Mappers/LSNAudioVrc6.h, CAudioVrc6) and of the
	gain its mapper gives it in the mix (Src/Mappers/LSNMapper024.h, GetExtAudio).
	https://github.com/L-Spiro/BeesNES

	Port notes:
	- The C++ class is a mapper part: it maps its registers onto the CPU bus and is ticked by the
	  mapper. Here it is a libvgm device. The owner (the NSF player) applies the register writes
	  with the CPU address and renders the device up to the write's sample first.
	- The registers decode `address & 0xF003`, which is the C++'s mapper 24 swizzle
	  (Swizzle_VRC6a_A0_A1) applied to every address of $8000-$FFFF. NSF files use this layout.
	  Not ported: the mapper 26 swizzle (A0 and A1 swapped), the bus mapping (ApplyMap), ResetSoft
	  (empty in the C++), PostProcessSample (identity) and the float Sample() accessor (see the
	  output below).
	- The output of the C++ per CPU cycle is the integer mix pulse 1 + pulse 2 + saw (0-61), as the
	  float mix / 63, times 0.6074657440185546875, which the mapper gives the extended audio next to
	  the APU's 1.0. The 62 possible values are in a table, computed with the same float operations
	  (the result is the same number the C++ gets for each mix).
	- Deliberate changes from the C++:
	  - The output stage is the one of the BeesNES APU (nes_expout.h): the level goes to a
	    band-limited buffer at the CPU clock and out through the console filter, at the output rate.
	    The C++'s own output stage (the APU's mixer, low-pass and output poles) is not part of the
	    port. The level is the table value times NES_EXPOUT_LEVEL_ONE, the scale of the APU.
	  - A mute mask (bit 0, 1, 2) silences a channel in the mix. The channel still runs.
	  - Reset starts from the state of the C++ constructor (all registers 0, saw step 13); the C++'s
	    ResetFull() does nothing for its default argument.
	  - The device runs at any output rate, up to the CPU clock / 4 (a higher rate is resampled by
	    libvgm), like the BeesNES APU device.
	- Equivalence of the restructured parts:
	  - SetPulseHigh resets the duty step and the divider when the new E bit is 0, and when it is 1
	    while the old one was 0: that is when not both are 1, which is how it is written here.
	    SetSawHigh differs in its two cases (the accumulator only, or the accumulator and the step),
	    so it keeps both branches.
	  - CombinePeriod(low, high) is at most 0x0FFF, so the C++'s `& 0x0FFF` after it does nothing.
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
#include "beesnes_vrc6.h"


static UINT8 device_start_vrc6(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_vrc6(void* info);
static void vrc6_reset(void* info);
static void vrc6_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void vrc6_w(void* info, UINT16 addr, UINT8 data);
static void vrc6_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, 0, vrc6_w},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, vrc6_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"VRC6", "BeesNES", FCC_BNES,

	device_start_vrc6,
	device_stop_vrc6,
	vrc6_reset,
	vrc6_update,

	NULL,	// SetOptionBits
	vrc6_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "VRC6";
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
		"Pulse 1", "Pulse 2", "Saw",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_VRC6 =
{
	DEVID_VRC6,
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

#define VRC6_DEFAULT_CLOCK	1789773	// NTSC
/* The gain the mapper gives the VRC6 mix (mix / 63) next to the APU's 1.0 */
#define VRC6_MIX_GAIN		0.6074657440185546875f
/* the 3 outputs add up to 15 + 15 + 31 = 61 at most */
#define VRC6_MIX_LEVELS		64

/** A VRC6 pulse channel. */
typedef struct vrc6_pulse
{
	UINT8 control;		/**< $9000/$A000: MDDD VVVV. */
	UINT8 freq_low;		/**< Low 8 bits of the 12-bit period. */
	UINT8 freq_high;	/**< High 4 bits of period + E bit (bit 7). */
	UINT16 divider;		/**< Current timer divider value. */
	UINT8 duty_step;	/**< Duty sequence position [0..15]. */
} VRC6_PULSE;

/** The VRC6 saw channel. */
typedef struct vrc6_saw
{
	UINT8 rate;			/**< $B000: ..AA AAAA accumulator rate. */
	UINT8 freq_low;		/**< Low 8 bits of the 12-bit period. */
	UINT8 freq_high;	/**< High 4 bits of period + E bit (bit 7). */
	UINT16 divider;		/**< Current timer divider value. */
	UINT8 accumulator;	/**< 8-bit internal accumulator. */
	UINT8 step;			/**< 14-step sequence position, init so first clock resets. */
} VRC6_SAW;

typedef struct vrc6_chip
{
	DEV_DATA _devData;

	NES_EXPOUT out;
	INT32 level[VRC6_MIX_LEVELS];	/* output level (NES_EXPOUT_LEVEL_ONE scale) of each mix */

	VRC6_PULSE pulse[2];	/** VRC6 pulse channels (Pulse 1 and Pulse 2). */
	VRC6_SAW saw;			/** VRC6 saw channel. */
	UINT8 freq_ctrl;		/** Global VRC6 frequency control ($9003: ..ABH). */

	UINT8 mute_mask;
} VRC6_CHIP;


/**
 * Sets the high frequency/enable byte for a VRC6 pulse channel.
 */
INLINE void set_pulse_high(VRC6_PULSE* ch, UINT8 val)
{
	UINT8 was_enabled = (ch->freq_high & 0x80) != 0;
	UINT8 is_enabled = (val & 0x80) != 0;

	ch->freq_high = val;
	if (! (was_enabled && is_enabled))
	{
		// Disable: reset and halt duty so phase can be restarted when re-enabled.
		// Rising edge of enable: restart from beginning.
		ch->duty_step = 0;
		ch->divider = 0;
	}
}

/**
 * Sets the high frequency/enable byte for the VRC6 saw channel.
 */
INLINE void set_saw_high(VRC6_SAW* saw, UINT8 val)
{
	UINT8 was_enabled = (saw->freq_high & 0x80) != 0;
	UINT8 is_enabled = (val & 0x80) != 0;

	saw->freq_high = val;
	if (! is_enabled)
	{
		// Disable: accumulator forced to zero while disabled.
		saw->accumulator = 0;
	}
	else if (! was_enabled && is_enabled)
	{
		// Rising edge of enable: mostly reset phase.
		saw->accumulator = 0;
		saw->step = 13;	// Next clock will be step 0, which resets accumulator.
	}
}

static void vrc6_w(void* info, UINT16 addr, UINT8 data)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;

	switch(addr & 0xF003)
	{
	case 0x9000:	// Pulse 1 control: MDDD VVVV.
		chip->pulse[0].control = data;
		break;
	case 0xA000:	// Pulse 2 control: MDDD VVVV.
		chip->pulse[1].control = data;
		break;
	case 0xB000:	// Saw accumulator rate ..AA AAAA.
		chip->saw.rate = data & 0x3F;
		break;
	case 0x9001:	// Pulse 1 period low.
		chip->pulse[0].freq_low = data;
		break;
	case 0xA001:	// Pulse 2 period low.
		chip->pulse[1].freq_low = data;
		break;
	case 0xB001:	// Saw period low.
		chip->saw.freq_low = data;
		break;
	case 0x9002:	// Pulse 1 freq high + enable.
		set_pulse_high(&chip->pulse[0], data);
		break;
	case 0xA002:	// Pulse 2 freq high + enable.
		set_pulse_high(&chip->pulse[1], data);
		break;
	case 0xB002:	// Saw freq high + enable.
		set_saw_high(&chip->saw, data);
		break;
	case 0x9003:	// Global frequency scaling and halt: keep only the ..ABH bits.
		chip->freq_ctrl = data & 0x07;
		break;
	}
}

/**
 * Applies the global frequency scaling (AB bits of $9003) to the given 12-bit period.
 */
INLINE UINT16 scaled_period(const VRC6_CHIP* chip, UINT16 period)
{
	UINT8 ctrl = chip->freq_ctrl;

	// 256x overrides 16x.
	if (ctrl & 0x04)
		return (UINT16)(period >> 8);
	if (ctrl & 0x02)
		return (UINT16)(period >> 4);
	return period;
}

/**
 * Ticks a single VRC6 pulse channel by 1 CPU cycle (timer + duty).
 */
INLINE void tick_pulse_channel(const VRC6_CHIP* chip, VRC6_PULSE* ch, UINT8 halt)
{
	UINT16 period;
	UINT16 effective;

	if (halt)
		return;
	// If channel disabled, timer/duty are halted.
	if ((ch->freq_high & 0x80) == 0)
		return;

	period = (UINT16)(ch->freq_low | ((ch->freq_high & 0x0F) << 8));
	effective = scaled_period(chip, period);

	if (ch->divider == 0)
	{
		ch->divider = effective;
		// Duty generator: 16-step sequence.
		ch->duty_step = (UINT8)((ch->duty_step + 1) & 0x0F);
	}
	else
	{
		-- ch->divider;
	}
}

/**
 * Computes the 4-bit output of a VRC6 pulse channel, in the range [0..15].
 */
INLINE UINT8 pulse_output(const VRC6_PULSE* ch)
{
	UINT8 vol;
	UINT8 duty;

	// E bit clears output immediately when low.
	if ((ch->freq_high & 0x80) == 0)
		return 0;

	vol = ch->control & 0x0F;
	if (vol == 0)
		return 0;

	// Mode bit: when set, ignore duty and output constant volume.
	if (ch->control & 0x80)
		return vol;

	duty = (ch->control >> 4) & 0x07;
	// Duty generator: step is in [0..15]. Output volume when step <= D, else 0.
	if (ch->duty_step <= duty)
		return vol;
	return 0;
}

/**
 * Ticks the VRC6 saw channel by 1 CPU cycle (timer + accumulator).
 */
INLINE void tick_saw(const VRC6_CHIP* chip, VRC6_SAW* saw, UINT8 halt)
{
	UINT16 period;
	UINT16 effective;

	if (halt)
		return;

	period = (UINT16)(saw->freq_low | ((saw->freq_high & 0x0F) << 8));
	effective = scaled_period(chip, period);

	if (saw->divider == 0)
	{
		saw->divider = effective;

		// Divider reached zero: clock the 14-step accumulator sequence.
		saw->step = (UINT8)((saw->step + 1) % 14);

		// Accumulator reacts every 2 clocks; even steps modify the accumulator.
		if ((saw->step & 0x01) == 0)
		{
			if (saw->step == 0)
			{
				// 14th step: reset accumulator.
				saw->accumulator = 0;
			}
			else
			{
				// Even step (2,4,6,8,10,12): add A to accumulator.
				saw->accumulator = (UINT8)(saw->accumulator + (saw->rate & 0x3F));
			}
		}
	}
	else
	{
		-- saw->divider;
	}

	// If disabled, accumulator is forced to zero until E is set again.
	if ((saw->freq_high & 0x80) == 0)
		saw->accumulator = 0;
}

/**
 * Computes the 5-bit output of the VRC6 saw channel, in the range [0..31].
 */
INLINE UINT8 saw_output(const VRC6_SAW* saw)
{
	if ((saw->freq_high & 0x80) == 0)
		return 0;
	// Output high 5 bits of accumulator.
	return (UINT8)(saw->accumulator >> 3);
}

/**
 * One CPU cycle: advance the oscillators and return the mix of the outputs, the sum of the two
 * 4-bit pulses and the 5-bit saw (the 6-bit DAC), in the range [0..61].
 */
INLINE UINT32 vrc6_tick(VRC6_CHIP* chip)
{
	const UINT8 halt = (chip->freq_ctrl & 0x01) != 0;
	UINT8 p0;
	UINT8 p1;
	UINT8 saw;

	// Advance oscillators.
	tick_pulse_channel(chip, &chip->pulse[0], halt);
	tick_pulse_channel(chip, &chip->pulse[1], halt);
	tick_saw(chip, &chip->saw, halt);

	// Sample outputs.
	p0 = pulse_output(&chip->pulse[0]);
	p1 = pulse_output(&chip->pulse[1]);
	saw = saw_output(&chip->saw);
	if (chip->mute_mask & 0x01)
		p0 = 0;
	if (chip->mute_mask & 0x02)
		p1 = 0;
	if (chip->mute_mask & 0x04)
		saw = 0;

	return (UINT32)p0 + (UINT32)p1 + (UINT32)saw;
}

/* Clocks the chip `cycles` times, adding each change of the level as a band-limited step. */
static void vrc6_run(void* info, UINT32 cycles)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;
	UINT32 cycle;

	for (cycle = 0; cycle < cycles; cycle ++)
		nes_expout_add(&chip->out, cycle, chip->level[vrc6_tick(chip)]);
}

static void vrc6_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;

	nes_expout_render(&chip->out, chip, vrc6_run, samples, outputs);
}


//----------------------------------------------------------------------------
// libvgm interface
//----------------------------------------------------------------------------

static UINT8 device_start_vrc6(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	VRC6_CHIP* chip;
	UINT32 clock;
	UINT32 rate;
	UINT32 mix;

	clock = cfg->clock ? cfg->clock : VRC6_DEFAULT_CLOCK;
	// The device synthesizes band-limited output at any rate, so it renders at the output rate
	// directly. Above clock/4 it would gain nothing, so libvgm resamples from there instead.
	rate = cfg->smplRate;
	if (rate == 0 || rate > clock / 4)
		rate = clock / 4;

	chip = (VRC6_CHIP*)calloc(1, sizeof(VRC6_CHIP));
	if (chip == NULL)
		return 0xFF;
	if (nes_expout_init(&chip->out, clock, rate))
	{
		free(chip);
		return 0xFF;
	}
	for (mix = 0; mix < VRC6_MIX_LEVELS; mix ++)
	{
		float sample = (float)mix * (1.0f / 63.0f);
		float level = sample * VRC6_MIX_GAIN;

		chip->level[mix] = (INT32)(level * NES_EXPOUT_LEVEL_ONE + 0.5f);
	}

	vrc6_reset(chip);
	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef);
	return 0x00;
}

static void device_stop_vrc6(void* info)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;

	nes_expout_free(&chip->out);
	free(chip);
}

static void vrc6_reset(void* info)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;

	memset(chip->pulse, 0x00, sizeof(chip->pulse));
	memset(&chip->saw, 0x00, sizeof(chip->saw));
	chip->saw.step = 13;
	chip->freq_ctrl = 0;

	nes_expout_reset(&chip->out, chip->level[0]);
}

static void vrc6_set_mute_mask(void* info, UINT32 MuteMask)
{
	VRC6_CHIP* chip = (VRC6_CHIP*)info;

	chip->mute_mask = (UINT8)(MuteMask & 0x07);
}
