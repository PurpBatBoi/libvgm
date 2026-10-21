/*
	MIT License

	Copyright (c) 2026 Shawn (L. Spiro) Wilcoxen
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

	Namco 163 expansion audio

	C port of the N163 audio of L-Spiro's BeesNES (Src/Mappers/LSNAudioNamco163.h, CAudioNamco163) and of
	the way its mapper (Src/Mappers/LSNMapper019.h) uses it: the register decoding, the sound disable bit of
	$E000 and the level of the audio in the mix.
	https://github.com/L-Spiro/BeesNES

	Port notes:
	- The C++ class is a mapper part: the mapper decodes the CPU addresses, owns the RAM, sets the volume and
	  ticks the class every CPU cycle. Here it is a libvgm device with its own RAM. The owner (the NSF
	  player) applies the register writes with the CPU address and renders the device up to the write's
	  sample first. The decoding is the mapper's: $4800-$4FFF data port (read and write), $E000-$E7FF sound
	  disable (bit 6 of the PRG bank 0 register), $F800-$FFFF address port.
	- Every 15th CPU cycle one channel is processed, as in the C++ (the channels go from the highest number
	  down to channel 1, as many as $7F bits 4-6 say): its signed 4-bit sample times its volume, / 120, is
	  the channel's output until its next turn. The channel's phase goes back into the RAM.
	- The level of the sample in the mix is its float times 0.65392053127288818359375, the volume the mapper
	  sets for submapper 4 (a Namco 163 board 16-17 dB above the loudest APU square wave,
	  https://www.nesdev.org/wiki/N163_audio). The C++ gives the NSF no board; the other submappers are
	  0.7758271694183349609375 (5, 18-19.5 dB) and 0.4145052134990692138671875 (3, 11-13 dB).
	- Deliberate changes from the C++:
	  - The output stage is the one of the BeesNES APU (nes_expout.h): the level goes to a band-limited
	    buffer at the CPU clock and out through the console filter, at the output rate. The C++'s own
	    stage (the APU's mixer, low-pass and output poles) is not part of the port. The level is the
	    sample times NES_EXPOUT_LEVEL_ONE, the scale of the APU.
	  - The output is the mean of the outputs of the active channels, not the time-multiplexed output of
	    the C++ (the processed channel alone until the next one, 15 cycles later). The mean is the time
	    average of the multiplexed output, so the level is the same; the multiplexing adds a carrier at
	    the CPU clock / (15 * channels) (14.9 kHz with 8 channels) whose sidebands at the carrier minus
	    the notes fall into the audible band and sound as off-pitch notes. NSFPlay (by default) and
	    libgme mix the channels too.
	  - The chip is enabled after a reset. The C++ is disabled until the mapper writes $E000, which a
	    cartridge does at start; an NSF does not have to (the NSF format does not list $E000 for the N163).
	    A disabled chip keeps its last sample, as in the C++.
	  - Reset zeroes the RAM and all state, which is the C++'s ResetFull(true) (the mapper's Reset).
	  - A mute mask (bit per channel) sets the output of a channel to 0 in the mean. The channel
	    still runs, its phase advances.
	  - The device runs at any output rate, up to the CPU clock / 4 (a higher rate is resampled by libvgm),
	    like the BeesNES APU device.
	  - A read from outside $4800-$4FFF returns 0 (the C++ mapper does not map it to the chip).
	- Not ported: SetRam / GetRam / RamSize (the RAM belongs to the chip), SetVolume and SetEnabled (the
	  volume is a constant, the enable bit is $E000), the null-RAM checks, PostProcessSample (identity), the
	  float Sample() accessor (see the output above).
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
#include "beesnes_n163.h"


static UINT8 device_start_n163(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_n163(void* info);
static void n163_reset(void* info);
static void n163_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static void n163_w(void* info, UINT16 addr, UINT8 data);
static UINT8 n163_r(void* info, UINT16 addr);
static void n163_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, 0, n163_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A16D8, 0, n163_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, n163_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"N163", "BeesNES", FCC_BNES,

	device_start_n163,
	device_stop_n163,
	n163_reset,
	n163_update,

	NULL,	// SetOptionBits
	n163_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "N163";
}

#define DEV_CHN_COUNT	8
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT] =
	{
		"Channel 1", "Channel 2", "Channel 3", "Channel 4",
		"Channel 5", "Channel 6", "Channel 7", "Channel 8",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_N163 =
{
	DEVID_N163,
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

#define N163_DEFAULT_CLOCK	1789773	// NTSC
/* The volume of the audio in the mix (BeesNES' mapper 19, submapper 4) */
#define N163_VOLUME			0.65392053127288818359375f
#define N163_RAM_SIZE		128
/* Namco 163 updates one audio channel every 15 CPU cycles. */
#define N163_CHANNEL_CYCLES	15

typedef struct n163_chip
{
	DEV_DATA _devData;

	NES_EXPOUT out;

	UINT8 ram[N163_RAM_SIZE];	/** The 128-byte internal RAM. */
	UINT8 addr;					/** Current address into the internal RAM [0..127]. */
	UINT8 auto_inc;				/** Address auto-increment flag. */
	UINT8 cycle_count;			/** Tracks CPU cycles to generate the 15-cycle channel update clock. */
	UINT8 current_channel;		/** The current channel being processed during multiplexing. */
	float ch_out[8];			/** The output of each channel at its last turn. */
	INT32 level;				/** The output level (NES_EXPOUT_LEVEL_ONE scale). */
	UINT8 enabled;				/** Is sound enabled? */

	UINT8 mute_mask;
} N163_CHIP;


/**
 * Writes the Namco 163 registers: the address port ($F800-$FFFF), the data port ($4800-$4FFF) and
 * the sound disable bit of the PRG bank 0 register ($E000-$E7FF).
 */
static void n163_w(void* info, UINT16 addr, UINT8 data)
{
	N163_CHIP* chip = (N163_CHIP*)info;

	switch(addr & 0xF800)
	{
	case 0xF800:	// internal address port
		chip->addr = data & 0x7F;
		chip->auto_inc = (data & 0x80) != 0;
		break;
	case 0x4800:	// data port
		chip->ram[chip->addr] = data;
		if (chip->auto_inc)
			chip->addr = (chip->addr + 1) & 0x7F;
		break;
	case 0xE000:	// bit 6 disables the sound
		chip->enabled = (data & 0x40) == 0;
		break;
	}
}

/**
 * Reads from the Namco 163 data port ($4800-$4FFF).
 */
static UINT8 n163_r(void* info, UINT16 addr)
{
	N163_CHIP* chip = (N163_CHIP*)info;
	UINT8 ret;

	if ((addr & 0xF800) != 0x4800)
		return 0x00;
	ret = chip->ram[chip->addr];
	if (chip->auto_inc)
		chip->addr = (chip->addr + 1) & 0x7F;
	return ret;
}

/**
 * Processes a single channel and updates the output, the mean of the active channels.
 */
INLINE void n163_audio_tick(N163_CHIP* chip)
{
	UINT8 num_channels;
	UINT8 base;
	UINT32 freq;
	UINT32 phase;
	UINT32 length;
	UINT32 address;
	UINT8 sample_byte;
	UINT8 nibble;
	INT32 sample;
	INT32 vol;
	float out;
	UINT8 ch;

	if (! chip->enabled)
		return;

	// Bits 4-6 of $7F indicate (NumChannels - 1).
	num_channels = ((chip->ram[0x7F] >> 4) & 0x07) + 1;

	// Channels are processed in descending order, ending with 0.
	if (chip->current_channel == 0 || chip->current_channel > num_channels)
		chip->current_channel = num_channels;
	chip->current_channel --;

	// Channel 0 is at $78, Channel 1 is at $70 ... Channel 7 is at $40.
	base = (UINT8)(0x40 + ((7 - chip->current_channel) * 8));

	// 18-bit frequency.
	freq = (UINT32)chip->ram[base + 0] | ((UINT32)chip->ram[base + 2] << 8) | (((UINT32)chip->ram[base + 4] & 0x03) << 16);

	// 24-bit phase.
	phase = (UINT32)chip->ram[base + 1] | ((UINT32)chip->ram[base + 3] << 8) | ((UINT32)chip->ram[base + 5] << 16);

	// Length is bits 2-7 of +4.
	length = 256 - (chip->ram[base + 4] & 0xFC);

	// Advance phase and wrap.
	phase = (phase + freq) % (length << 16);

	// Write phase.
	chip->ram[base + 1] = (UINT8)(phase & 0xFF);
	chip->ram[base + 3] = (UINT8)((phase >> 8) & 0xFF);
	chip->ram[base + 5] = (UINT8)((phase >> 16) & 0xFF);

	address = (UINT32)chip->ram[base + 6] + (phase >> 16);
	address &= 0xFF;

	sample_byte = chip->ram[address >> 1];
	nibble = (address & 1) ? (UINT8)(sample_byte >> 4) : (UINT8)(sample_byte & 0x0F);

	// Convert from unsigned 4-bit [0..15] to signed [-8..+7].
	sample = (INT32)nibble - 8;
	vol = chip->ram[base + 7] & 0x0F;

	// Normalize.
	if (chip->mute_mask & (1 << chip->current_channel))
		chip->ch_out[chip->current_channel] = 0.0f;
	else
		chip->ch_out[chip->current_channel] = (float)(sample * vol) / 120.0f * N163_VOLUME;

	out = 0.0f;
	for (ch = 0; ch < num_channels; ch ++)
		out += chip->ch_out[ch];
	out /= num_channels;
	chip->level = (INT32)(out * NES_EXPOUT_LEVEL_ONE + 0.5f);
}

/**
 * One CPU cycle. Returns 1 when a channel was processed (the level may have changed).
 */
INLINE UINT8 n163_tick(N163_CHIP* chip)
{
	chip->cycle_count ++;
	// Namco 163 updates one audio channel every 15 CPU cycles.
	if (chip->cycle_count >= N163_CHANNEL_CYCLES)
	{
		chip->cycle_count = 0;
		n163_audio_tick(chip);
		return 1;
	}
	return 0;
}

/* Clocks the chip `cycles` times, adding each change of the level as a band-limited step. */
static void n163_run(void* info, UINT32 cycles)
{
	N163_CHIP* chip = (N163_CHIP*)info;
	UINT32 cycle;

	for (cycle = 0; cycle < cycles; cycle ++)
	{
		if (n163_tick(chip))
			nes_expout_add(&chip->out, cycle, chip->level);
	}
}

static void n163_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	N163_CHIP* chip = (N163_CHIP*)info;

	nes_expout_render(&chip->out, chip, n163_run, samples, outputs);
}


//----------------------------------------------------------------------------
// libvgm interface
//----------------------------------------------------------------------------

static UINT8 device_start_n163(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	N163_CHIP* chip;
	UINT32 clock;
	UINT32 rate;

	clock = cfg->clock ? cfg->clock : N163_DEFAULT_CLOCK;
	// The device synthesizes band-limited output at any rate, so it renders at the output rate
	// directly. Above clock/4 it would gain nothing, so libvgm resamples from there instead.
	rate = cfg->smplRate;
	if (rate == 0 || rate > clock / 4)
		rate = clock / 4;

	chip = (N163_CHIP*)calloc(1, sizeof(N163_CHIP));
	if (chip == NULL)
		return 0xFF;
	if (nes_expout_init(&chip->out, clock, rate))
	{
		free(chip);
		return 0xFF;
	}

	n163_reset(chip);
	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef);
	return 0x00;
}

static void device_stop_n163(void* info)
{
	N163_CHIP* chip = (N163_CHIP*)info;

	nes_expout_free(&chip->out);
	free(chip);
}

static void n163_reset(void* info)
{
	N163_CHIP* chip = (N163_CHIP*)info;

	memset(chip->ram, 0x00, sizeof(chip->ram));
	chip->addr = 0;
	chip->auto_inc = 0;
	chip->cycle_count = 0;
	chip->current_channel = 0;
	memset(chip->ch_out, 0x00, sizeof(chip->ch_out));
	chip->level = 0;
	chip->enabled = 1;

	nes_expout_reset(&chip->out, chip->level);
}

static void n163_set_mute_mask(void* info, UINT32 MuteMask)
{
	N163_CHIP* chip = (N163_CHIP*)info;

	chip->mute_mask = (UINT8)MuteMask;
}
