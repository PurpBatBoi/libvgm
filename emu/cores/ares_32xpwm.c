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

    Sega 32X PWM sound

    C port of the PWM part of ares' 32X (ares/md/m32x/pwm.cpp,
    m32x.hpp, io-external.cpp, io-internal.cpp) and of the DC removal filter
    it uses (nall/dsp/iir/dc-removal.hpp).
    https://github.com/ares-emulator/ares

    Port notes:
    - One update sample is one PWM period: (cycle register - 1) PWM clocks
      (at least 1). The sample rate follows the cycle register, as in ares'
      updateFrequency(). A cycle change calls the sample rate change
      callback, with the rate rounded to an integer.
    - The registers are the 32X PWM registers at offset / 2 (the VGM command
      0xB2): 0 = control, 1 = cycle, 2 = left, 3 = right, 4 = mono. A write
      stores the full 12-bit value at once. ares splits it into the two bytes
      of the 16-bit bus, which gives the same result for a whole-word write.
    - The pulse widths go through the 3 entry FIFOs of ares. A write to a
      full FIFO drops the oldest entry; an empty FIFO repeats its last value.
      The ring buffer of nall::queue is a read index and a count here.
      Values are signed 12-bit, as in ares (i12).
    - The output is the float of ares (pulse width / clocks, DC filtered in
      double, clipped to -1.0 ... 1.0). It becomes a DEV_SMPL by scaling 1.0
      to 65536 and rounding to the nearest integer. A full-range pulse swings
      the float by +-0.5, so this gives +-32768: the level of the Gens/GS
      core that the player's volume for this chip (0xE0) was set for.
    - The clip x / abs(x) of ares is -1.0 or 1.0 for any x outside
      -1.0 ... 1.0, so the port sets the value directly.
    - Not ported: the SH-2 interrupt and DREQ lines and the timer register
      bits 8-11 / the period counter that drive them (libvgm has no CPU),
      the mono bit (it is only read back), all register reads and the FIFO
      status bits, the serializer, the scheduler thread and the debugger.
    - Deliberate changes from ares:
      - The output scale above. ares has no integer scale (it outputs floats).
      - The output modes (control register bits 0-3) start as direct stereo
        (left to left, right to right) instead of off, and a control write
        whose mode bits are both 0 only counts as a timer write and leaves
        the modes as they are. VGM rips made with Gens-based loggers store
        only the timer byte of the control register (0x100 in Kolibri and
        Knuckles' Chaotix), so the modes are not in the file; with the modes
        taken from that write the chip would be off and the song silent.
        Modes 1, 2 and 3 in a write are used as in ares. The cost: a song
        cannot switch the PWM off with a mode 0 write.
      - reset clears the last value of the FIFOs (ares keeps it), so a
        restarted song renders the same as a fresh one.
      - After a reset, the DC filter of each channel starts its history at
        the first non-zero input (y = input), so that tick outputs 0. ares
        starts the filter at rest, so the first pulse width is a step of about
        half scale that the filter takes ~1000 periods to remove: a loud
        thump at the start of every song. Until the first non-zero input the
        filter state is x = y = 0 either way, so only that tick differs.
      - The device always runs at the PWM period rate. The player resamples.
      - Muting computes everything and only silences the output.
    - Float arithmetic follows the C++ types (float for the pulse ratio,
      double inside the filter). A compiler that fuses multiply-add
      (for example on AArch64) can differ in the last bit of the filter.

***************************************************************************/

#include <stdlib.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "ares_32xpwm.h"


static void pwm_reset(void* info);
static void pwm_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_pwm(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_pwm(void* info);
static void pwm_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr);
static void pwm_chn_w(void* info, UINT8 Channel, UINT16 data);
static void pwm_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D16, 0, pwm_chn_w},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, pwm_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"32X PWM", "ares", FCC_ARES,

	device_start_pwm,
	device_stop_pwm,
	pwm_reset,
	pwm_update,

	NULL,	// SetOptionBits
	pwm_set_mute_mask,
	NULL,	// SetPanning
	pwm_set_srchg_cb,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "32X PWM";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 1;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_32X_PWM =
{
	DEVID_32X_PWM,
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

#define PWM_DEFAULT_CLOCK	23011361	// NTSC: 53693175 Hz / 7 * 3
#define PWM_FIFO_SIZE		3
#define PWM_OUTPUT_SCALE	65536.0f

// FIFO of 12-bit pulse widths. An empty FIFO keeps returning its last value.
typedef struct _pwm_fifo
{
	INT16 data[PWM_FIFO_SIZE];
	INT16 last;
	UINT8 readPos;
	UINT8 count;
} PWM_FIFO;

// nall::DSP::IIR::DCRemoval
typedef struct _pwm_dc_filter
{
	double x;
	double y;
} PWM_DC_FILTER;

typedef struct _pwm_state
{
	DEV_DATA _devData;

	UINT32 clock;
	UINT32 rate;
	DEVCB_SRATE_CHG smpRateFunc;
	void* smpRateData;

	UINT8 lmode;
	UINT8 rmode;
	UINT16 cycle;

	PWM_FIFO lfifo;
	PWM_FIFO rfifo;
	INT16 lsample;
	INT16 rsample;
	PWM_DC_FILTER lfilter;
	PWM_DC_FILTER rfilter;
	UINT8 lprimed;	// the filter history was set from the first non-zero input
	UINT8 rprimed;

	UINT8 mute;
} pwm_state;


// sign-extend the low 12 bits
INLINE INT16 pwm_i12(UINT16 value)
{
	return (INT16)(((value & 0x0FFF) ^ 0x0800) - 0x0800);
}

INLINE void fifo_flush(PWM_FIFO* fifo)
{
	fifo->readPos = 0;
	fifo->count = 0;
}

INLINE INT16 fifo_read(PWM_FIFO* fifo)
{
	if (fifo->count > 0)
	{
		fifo->last = fifo->data[fifo->readPos];
		fifo->readPos = (UINT8)((fifo->readPos + 1) % PWM_FIFO_SIZE);
		fifo->count --;
	}
	return fifo->last;
}

INLINE void fifo_write(PWM_FIFO* fifo, INT16 data)
{
	if (fifo->count == PWM_FIFO_SIZE)
		fifo_read(fifo);	// purge oldest sample
	fifo->data[(fifo->readPos + fifo->count) % PWM_FIFO_SIZE] = data;
	fifo->count ++;
}

INLINE double dc_filter_process(PWM_DC_FILTER* filter, double in)
{
	filter->x = 0.999 * filter->x + in - filter->y;
	filter->y = in;
	return filter->x;
}

// PWM clocks per period
INLINE UINT32 pwm_clocks(const pwm_state* chip)
{
	UINT32 clocks = (UINT32)(chip->cycle - 1) & 0x0FFF;
	return (clocks == 0) ? 1 : clocks;
}

static void pwm_update_rate(pwm_state* chip)
{
	UINT32 clocks = pwm_clocks(chip);
	UINT32 rate = (chip->clock + clocks / 2) / clocks;

	if (rate == chip->rate)
		return;
	chip->rate = rate;
	if (chip->smpRateFunc != NULL)
		chip->smpRateFunc(chip->smpRateData, rate);
}

INLINE DEV_SMPL pwm_output(float value, UINT8 mute)
{
	float scaled;

	if (mute)
		return 0;
	scaled = value * PWM_OUTPUT_SCALE;
	return (DEV_SMPL)(scaled < 0.0f ? scaled - 0.5f : scaled + 0.5f);
}

static void pwm_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	pwm_state* chip = (pwm_state*)info;
	DEV_SMPL* outL = outputs[0];
	DEV_SMPL* outR = outputs[1];
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
	{
		UINT32 clocks = pwm_clocks(chip);
		INT16 sampleL;
		INT16 sampleR;
		float left;
		float right;

		// cycle 1 is an illegal setting: "PWM will no longer operate"
		if (chip->cycle != 1 && (((chip->lmode ^ (chip->lmode >> 1)) & 1) || ((chip->rmode ^ (chip->rmode >> 1)) & 1)))
		{
			chip->lsample = fifo_read(&chip->lfifo);
			chip->rsample = fifo_read(&chip->rfifo);
		}

		// mode 0 = off, 1 = own channel, 2 = other channel, 3 = illegal
		sampleL = (chip->lmode == 1) ? chip->lsample : (chip->lmode == 2) ? chip->rsample : 0;
		sampleR = (chip->rmode == 1) ? chip->rsample : (chip->rmode == 2) ? chip->lsample : 0;

		left = (float)sampleL / (float)clocks;
		right = (float)sampleR / (float)clocks;

		// start the filter history at the first non-zero input (see the port notes)
		if (! chip->lprimed && left != 0.0f)
		{
			chip->lprimed = 1;
			chip->lfilter.y = left;
		}
		if (! chip->rprimed && right != 0.0f)
		{
			chip->rprimed = 1;
			chip->rfilter.y = right;
		}

		// filter DC offset to normalize output and limit popping
		left = (float)dc_filter_process(&chip->lfilter, left);
		right = (float)dc_filter_process(&chip->rfilter, right);

		// handle clipping due to improper settings
		if (left > 1.0f || left < -1.0f)
			left = (left < 0.0f) ? -1.0f : 1.0f;
		if (right > 1.0f || right < -1.0f)
			right = (right < 0.0f) ? -1.0f : 1.0f;

		outL[curSmpl] = pwm_output(left, chip->mute);
		outR[curSmpl] = pwm_output(right, chip->mute);
	}
}

static void pwm_reset(void* info)
{
	pwm_state* chip = (pwm_state*)info;

	chip->lmode = 1;
	chip->rmode = 1;
	chip->cycle = 0;
	chip->lsample = 0;
	chip->rsample = 0;
	fifo_flush(&chip->lfifo);
	fifo_flush(&chip->rfifo);
	chip->lfifo.last = 0;
	chip->rfifo.last = 0;
	chip->lfilter.x = 0.0;
	chip->lfilter.y = 0.0;
	chip->rfilter.x = 0.0;
	chip->rfilter.y = 0.0;
	chip->lprimed = 0;
	chip->rprimed = 0;
	pwm_update_rate(chip);
}

static UINT8 device_start_pwm(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	pwm_state* chip;

	chip = (pwm_state*)calloc(1, sizeof(pwm_state));
	if (chip == NULL)
		return 0xFF;

	chip->clock = cfg->clock;
	if (! chip->clock)
		chip->clock = PWM_DEFAULT_CLOCK;
	pwm_reset(chip);
	pwm_set_mute_mask(chip, 0x00);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, chip->rate, &devDef);
	return 0x00;
}

static void device_stop_pwm(void* info)
{
	pwm_state* chip = (pwm_state*)info;

	free(chip);
}

static void pwm_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr)
{
	pwm_state* chip = (pwm_state*)info;

	chip->smpRateFunc = CallbackFunc;
	chip->smpRateData = DataPtr;
}

static void pwm_chn_w(void* info, UINT8 Channel, UINT16 data)
{
	pwm_state* chip = (pwm_state*)info;

	switch(Channel)
	{
	case 0x00:	// control register
		if (data & 0x0F)
		{
			chip->lmode = (UINT8)(data & 0x03);
			chip->rmode = (UINT8)((data >> 2) & 0x03);
		}
		break;
	case 0x01:	// cycle register
		chip->cycle = data & 0x0FFF;
		pwm_update_rate(chip);
		break;
	case 0x02:	// left channel pulse width
		fifo_write(&chip->lfifo, pwm_i12(data));
		break;
	case 0x03:	// right channel pulse width
		fifo_write(&chip->rfifo, pwm_i12(data));
		break;
	case 0x04:	// mono pulse width
		fifo_write(&chip->lfifo, pwm_i12(data));
		fifo_write(&chip->rfifo, pwm_i12(data));
		break;
	}
}

static void pwm_set_mute_mask(void* info, UINT32 MuteMask)
{
	pwm_state* chip = (pwm_state*)info;

	chip->mute = (UINT8)((MuteMask >> 0) & 0x01);
}
