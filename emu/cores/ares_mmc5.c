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

    Nintendo MMC5 expansion audio

    C port of the audio of the MMC5 board of ares (ares/fc/cartridge/board/hvc-exrom.cpp: the Envelope,
    Pulse and PCM structs, the audio part of main() and the audio registers of writePRG() / readPRG()).
    https://github.com/ares-emulator/ares

    Port notes:
    - The C++ is a cartridge board: banking, ExRAM, the multiplier, the video logic and the IRQ lines
      share its register file and its main() with the audio. Here the audio is a libvgm device. The owner
      (the NSF player) applies the register writes with the CPU address and renders the device up to the
      write's sample first.
    - The pulses are the pulses of the APU without the sweep: the C++ clocks them every CPU cycle (the
      period counter counts (period + 1) * 2 cycles, the APU's timer period of (period + 1) APU cycles).
      The length counters and the envelopes are clocked by a divider that main() counts down by 2 per
      cycle from 14915, which is ~240 Hz at the NTSC clock (twice the rate of the APU's length counter).
      It is counted in CPU cycles, so at the PAL clock it runs at ~223 Hz, as the C++ would.
    - The output of the C++ per CPU cycle is -(pulseDAC[pulse 1 + pulse 2] + (PCM << 7)), on the
      32768 = 1.0 scale of its APU's DAC tables. The sign is the MMC5's inverted output and is kept.
    - Not ported: banking, ExRAM, nametables, fill mode, the vertical split, the multiplier ($5205 / $5206),
      the scanline IRQ, the $5209 timer, the cl3 / sl3 pins and the CHR / PRG access of the board (the
      player's memory map has them), the PCM read mode and the PCM IRQ (read mode copies the data of CPU
      reads of $8000-$BFFF into the DAC, which would need a hook on every CPU read; the mode bit is kept,
      and in read mode a write of $5011 is ignored as in the C++), the register reads other than $5015,
      the serializer, and the 16-bit clamp of the C++'s audio stream (see the output below).
    - Deliberate changes from the C++:
      - The output stage is the one of the BeesNES APU (nes_expout.h): the level goes to a band-limited
        buffer at the CPU clock and out through the console filter, at the output rate. The level is
        calibrated against the APU: a pulse is the same as an APU pulse with the same volume, so the pulse
        sum goes through the table of the BeesNES APU (95.52 / (8128 / n + 100), n = pulse 1 + pulse 2,
        the C++ has the constant 95.88) times NES_EXPOUT_LEVEL_ONE, the scale in which the APU's full
        mix is 1.0. The PCM keeps the scale of the C++, 255 << 7 against 32768 = 1.0, so one PCM step is
        NES_EXPOUT_LEVEL_ONE / 256 (nesdev: the PCM is about as loud as the APU with equivalent input,
        and twice as loud as a DMC with the same number of steps). The 16-bit clamp of the C++'s audio
        stream is dropped: libvgm's mix has room, and the clamp only acts when the PCM is above 190 while
        the pulses play.
      - A mute mask (bit 0, 1, 2) silences a channel in the mix. The channel still runs.
      - Reset starts from the power-on state of the board (the C++ power(): divider 1) with both pulses
        enabled ($5015 = 3, as the BeesNES APU device does for $4015), PCM mode 0, DAC 0.
      - The period counters start at 1, as the power() of ares' own APU pulse sets them. The C++ board
        never sets them, so they are 0 and the first note of a channel waits 65536 cycles (36.6 ms)
        before the timer reloads, with the duty step held at 0.
      - $5015 is implemented as the APU's $4015: bit 0 / bit 1 enable the pulse. A disabled pulse has
        its length counter cleared and a write of $5003 / $5007 does not load it, as in the APU, and as
        the real chip (nesdev: "analogous to $4015"). The C++ ignores $5015 writes, so a driver that
        cuts a pulse through $5015 would not be silenced. A game that never writes $5015 plays the
        same as with the C++, since both pulses are enabled at reset.
      - The device runs at any output rate, up to the CPU clock / 4 (a higher rate is resampled by
        libvgm), like the BeesNES APU device.
    - Equivalence of the restructured parts:
      - The length counter (n16 in the C++) only ever holds a value of the length table (at most 0xFE)
        minus decrements, so a UINT8 does the same.
      - n2 / n3 / n4 / n8 / n11 / n16 fields: the ones that wrap (duty step, decay volume, decay counter,
        period counter) are masked or stored in a type of the same width, so they wrap as the C++ does.
***************************************************************************/

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
#include "ares_mmc5.h"


static UINT8 device_start_mmc5(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_mmc5(void* info);
static void mmc5_reset(void* info);
static void mmc5_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void mmc5_w(void* info, UINT16 addr, UINT8 data);
static UINT8 mmc5_r(void* info, UINT16 addr);
static void mmc5_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, 0, mmc5_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A16D8, 0, mmc5_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, mmc5_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"MMC5", "ares", FCC_ARES,

	device_start_mmc5,
	device_stop_mmc5,
	mmc5_reset,
	mmc5_update,

	NULL,	// SetOptionBits
	mmc5_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "MMC5";
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
		"Pulse 1", "Pulse 2", "PCM",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_MMC5 =
{
	DEVID_MMC5,
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

#define MMC5_DEFAULT_CLOCK	1789773	// NTSC
/* The divider for the length counters and the envelopes: counted down by 2 per CPU cycle,
   ~(21.477MHz / 6 / 240hz) */
#define MMC5_FRAME_DIVIDER	14915
/* the two pulses add up to 15 + 15 = 30 at most */
#define MMC5_PULSE_LEVELS	31
/* One step of the 8-bit PCM in the level scale: the C++'s (dac << 7) on its 32768 = 1.0 scale. */
#define MMC5_PCM_LEVEL		(NES_EXPOUT_LEVEL_ONE / 256)

static const UINT8 length_table[32] =
{
	0x0A, 0xFE, 0x14, 0x02, 0x28, 0x04, 0x50, 0x06,
	0xA0, 0x08, 0x3C, 0x0A, 0x0E, 0x0C, 0x1A, 0x0E,
	0x0C, 0x10, 0x18, 0x12, 0x30, 0x14, 0x60, 0x16,
	0xC0, 0x18, 0x48, 0x1A, 0x10, 0x1C, 0x20, 0x1E,
};

static const UINT8 duty_table[4][8] =
{
	{0, 0, 0, 0, 0, 0, 0, 1},	//12.5%
	{0, 0, 0, 0, 0, 0, 1, 1},	//25.0%
	{0, 0, 0, 0, 1, 1, 1, 1},	//50.0%
	{1, 1, 1, 1, 1, 1, 0, 0},	//25.0% (negated)
};

/** The envelope of a pulse. */
typedef struct mmc5_envelope
{
	UINT8 speed;				/**< Envelope speed / constant volume, 4 bits. */
	UINT8 use_speed_as_volume;	/**< Constant volume flag. */
	UINT8 loop_mode;			/**< Envelope loop / length counter halt flag. */
	UINT8 reload_decay;			/**< Set by a write of the period high byte: restart the decay. */
	UINT8 decay_counter;
	UINT8 decay_volume;			/**< 4 bits. */
} MMC5_ENVELOPE;

/** An MMC5 pulse channel: operates identically to the APU pulse channels, only without sweep support. */
typedef struct mmc5_pulse
{
	MMC5_ENVELOPE envelope;
	UINT8 enable;			/**< $5015 bit of the channel. */
	UINT8 length_counter;
	UINT16 period_counter;
	UINT8 duty;				/**< 2 bits. */
	UINT8 duty_counter;		/**< 3 bits. */
	UINT16 period;			/**< 11 bits. */
} MMC5_PULSE;

typedef struct mmc5_chip
{
	DEV_DATA _devData;

	NES_EXPOUT out;
	INT32 pulse_level[MMC5_PULSE_LEVELS];	/* level of a pulse sum (NES_EXPOUT_LEVEL_ONE scale) */

	MMC5_PULSE pulse[2];	/** $5000-$5003, $5004-$5007 */
	UINT8 pcm_mode;			/** $5010 bit 0: 0 = write mode, 1 = read mode. */
	UINT8 pcm_dac;			/** $5011 */
	INT32 frame_divider;

	UINT8 mute_mask;
} MMC5_CHIP;


INLINE UINT8 envelope_volume(const MMC5_ENVELOPE* env)
{
	return env->use_speed_as_volume ? env->speed : env->decay_volume;
}

INLINE void envelope_clock(MMC5_ENVELOPE* env)
{
	if (env->reload_decay)
	{
		env->reload_decay = 0;
		env->decay_volume = 0x0F;
		env->decay_counter = (UINT8)(env->speed + 1);
		return;
	}
	if (-- env->decay_counter == 0)
	{
		env->decay_counter = (UINT8)(env->speed + 1);
		if (env->decay_volume || env->loop_mode)
			env->decay_volume = (UINT8)((env->decay_volume - 1) & 0x0F);
	}
}

INLINE void pulse_clock_length(MMC5_PULSE* ch)
{
	if (ch->envelope.loop_mode == 0)
	{
		if (ch->length_counter)
			-- ch->length_counter;
	}
}

/**
 * One CPU cycle of a pulse: returns its output, [0..15].
 */
INLINE UINT8 pulse_clock(MMC5_PULSE* ch)
{
	UINT8 result;

	if (ch->length_counter == 0)
		return 0;
	result = duty_table[ch->duty][ch->duty_counter] ? envelope_volume(&ch->envelope) : 0;
	if (-- ch->period_counter == 0)
	{
		ch->period_counter = (UINT16)((ch->period + 1) * 2);
		ch->duty_counter = (UINT8)((ch->duty_counter - 1) & 0x07);
	}
	return result;
}

/**
 * Writes register 0-3 of a pulse ($5000-$5003, $5004-$5007; register 1 is the sweep, which the MMC5 does not have).
 */
static void pulse_write(MMC5_PULSE* ch, UINT8 reg, UINT8 data)
{
	switch(reg)
	{
	case 0:	// DDLC VVVV: duty, loop / length counter halt, constant volume, volume / envelope speed
		ch->envelope.speed = data & 0x0F;
		ch->envelope.use_speed_as_volume = (data >> 4) & 0x01;
		ch->envelope.loop_mode = (data >> 5) & 0x01;
		ch->duty = data >> 6;
		break;
	case 2:	// period low
		ch->period = (UINT16)((ch->period & 0x0700) | data);
		break;
	case 3:	// LLLL LPPP: length counter index, period high
		ch->period = (UINT16)((ch->period & 0x00FF) | ((data & 0x07) << 8));
		ch->duty_counter = 0;
		ch->envelope.reload_decay = 1;
		if (ch->enable)
			ch->length_counter = length_table[data >> 3];
		break;
	}
}

static void mmc5_w(void* info, UINT16 addr, UINT8 data)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;

	switch(addr)
	{
	case 0x5000:	// Pulse 1
	case 0x5001:
	case 0x5002:
	case 0x5003:
		pulse_write(&chip->pulse[0], (UINT8)(addr & 0x03), data);
		break;
	case 0x5004:	// Pulse 2
	case 0x5005:
	case 0x5006:
	case 0x5007:
		pulse_write(&chip->pulse[1], (UINT8)(addr & 0x03), data);
		break;
	case 0x5010:	// PCM mode (the IRQ enable, bit 7, is not ported)
		chip->pcm_mode = data & 0x01;
		break;
	case 0x5011:	// raw PCM (the C++ raises the PCM IRQ for a write of 0, which does not change the DAC)
		if (chip->pcm_mode == 0 && data != 0x00)
			chip->pcm_dac = data;
		break;
	case 0x5015:	// pulse enables, as the APU's $4015
		chip->pulse[0].enable = data & 0x01;
		chip->pulse[1].enable = (data >> 1) & 0x01;
		if (! chip->pulse[0].enable)
			chip->pulse[0].length_counter = 0;
		if (! chip->pulse[1].enable)
			chip->pulse[1].length_counter = 0;
		break;
	}
}

static UINT8 mmc5_r(void* info, UINT16 addr)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;

	if (addr == 0x5015)	// length counter status
		return (UINT8)((chip->pulse[0].length_counter != 0) | ((chip->pulse[1].length_counter != 0) << 1));
	return 0x00;
}

/**
 * One CPU cycle: advance the divider and the channels and return the level of the output, which is
 * -(pulse level + PCM level) (the MMC5's output is inverted).
 */
INLINE INT32 mmc5_tick(MMC5_CHIP* chip)
{
	UINT8 p0;
	UINT8 p1;
	UINT8 dac;

	chip->frame_divider -= 2;
	if (chip->frame_divider <= 0)
	{
		// length counter clocked at twice the rate of the APU pulse channels
		pulse_clock_length(&chip->pulse[0]);
		envelope_clock(&chip->pulse[0].envelope);
		pulse_clock_length(&chip->pulse[1]);
		envelope_clock(&chip->pulse[1].envelope);
		chip->frame_divider += MMC5_FRAME_DIVIDER;
	}

	p0 = pulse_clock(&chip->pulse[0]);
	p1 = pulse_clock(&chip->pulse[1]);
	dac = chip->pcm_dac;
	if (chip->mute_mask & 0x01)
		p0 = 0;
	if (chip->mute_mask & 0x02)
		p1 = 0;
	if (chip->mute_mask & 0x04)
		dac = 0;

	return -(chip->pulse_level[p0 + p1] + (INT32)dac * MMC5_PCM_LEVEL);
}

/* Clocks the chip `cycles` times, adding each change of the level as a band-limited step. */
static void mmc5_run(void* info, UINT32 cycles)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;
	UINT32 cycle;

	for (cycle = 0; cycle < cycles; cycle ++)
		nes_expout_add(&chip->out, cycle, mmc5_tick(chip));
}

static void mmc5_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;

	nes_expout_render(&chip->out, chip, mmc5_run, samples, outputs);
}


//----------------------------------------------------------------------------
// libvgm interface
//----------------------------------------------------------------------------

static UINT8 device_start_mmc5(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	MMC5_CHIP* chip;
	UINT32 clock;
	UINT32 rate;
	UINT32 sum;

	clock = cfg->clock ? cfg->clock : MMC5_DEFAULT_CLOCK;
	// The device synthesizes band-limited output at any rate, so it renders at the output rate
	// directly. Above clock/4 it would gain nothing, so libvgm resamples from there instead.
	rate = cfg->smplRate;
	if (rate == 0 || rate > clock / 4)
		rate = clock / 4;

	chip = (MMC5_CHIP*)calloc(1, sizeof(MMC5_CHIP));
	if (chip == NULL)
		return 0xFF;
	if (nes_expout_init(&chip->out, clock, rate))
	{
		free(chip);
		return 0xFF;
	}
	// the pulses of the APU (entry 0 is 0)
	for (sum = 1; sum < MMC5_PULSE_LEVELS; sum ++)
	{
		float level = 95.52f / (8128.0f / (float)sum + 100.0f);

		chip->pulse_level[sum] = (INT32)(level * NES_EXPOUT_LEVEL_ONE + 0.5f);
	}

	mmc5_reset(chip);
	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef);
	return 0x00;
}

static void device_stop_mmc5(void* info)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;

	nes_expout_free(&chip->out);
	free(chip);
}

static void mmc5_reset(void* info)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;
	UINT8 ch;

	memset(chip->pulse, 0x00, sizeof(chip->pulse));
	for (ch = 0; ch < 2; ch ++)
	{
		chip->pulse[ch].enable = 1;
		chip->pulse[ch].period_counter = 1;
	}
	chip->pcm_mode = 0;
	chip->pcm_dac = 0;
	chip->frame_divider = 1;

	nes_expout_reset(&chip->out, chip->pulse_level[0]);
}

static void mmc5_set_mute_mask(void* info, UINT32 MuteMask)
{
	MMC5_CHIP* chip = (MMC5_CHIP*)info;

	chip->mute_mask = (UINT8)(MuteMask & 0x07);
}
