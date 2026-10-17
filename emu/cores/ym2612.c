// license:ISC
// copyright-holders:Talarubi, ares team, Near et al
/***************************************************************************

    Yamaha YM2612 (OPN2)

    C port of the ares YM2612 core (ares/component/audio/ym2612).
    Original author: Talarubi

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

    Port notes:
    - ares stores registers in fixed-width bit fields that wrap on
      assignment. Every such assignment here masks to the field width.
    - One update sample is one chip sample (clock / 144).
    - ares filters its output (20 Hz high-pass, 2840 Hz low-pass) outside
      the chip. Those board filters are not part of this core.

***************************************************************************/

#include <stdlib.h>
#include <string.h>	// for memset
#include <math.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"

#include "ym2612.h"
#include "ym2612_ymfm.h"

#ifndef M_PI
#define M_PI	3.14159265358979323846
#endif


static void ym2612_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym2612(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ym2612(void* info);
static void device_reset_ym2612(void* info);
static void ym2612_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2612_r(void* info, UINT8 offset);
static void ym2612_set_mute_mask(void* info, UINT32 MuteMask);


static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2612_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2612_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ym2612_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"YM2612", "ares", FCC_ARES,

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

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && devCfg->flags)
		return "YM3438";
	return "YM2612";
}

#define DEV_CHN_COUNT	7
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return DEV_CHN_COUNT;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT] =
	{
		"FM 1", "FM 2", "FM 3", "FM 4", "FM 5", "FM 6",
		"DAC",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_YM2612 =
{
	DEVID_YM2612,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,	// default
		&devDef_YM2612_YMFM,
		NULL
	}
};


// envelope states
#define EG_ATTACK	0
#define EG_DECAY	1
#define EG_SUSTAIN	2
#define EG_RELEASE	3

// operator output masks
#define MOD_MASK	(-(1 << 1))
#define SUM_MASK	(-(1 << 5))
#define OUT_MASK	(-(1 << 5))

typedef struct _envelope_rate
{
	UINT32 divider;
	UINT32 steps[4];
} ENVELOPE_RATE;

typedef struct _ym2612_operator
{
	UINT8 keyOn;		// 1 bit
	UINT8 keyLine;		// 1 bit
	UINT8 tremoloEnable;	// 1 bit
	UINT8 keyScale;		// 5 bits
	UINT8 detune;		// 3 bits
	UINT8 multiple;		// 4 bits
	UINT8 totalLevel;	// 7 bits

	UINT16 outputLevel;
	INT16 output;
	INT16 prior;
	INT16 priorBuffer;

	struct
	{
		UINT16 value;	// 11 bits
		UINT16 reload;	// 11 bits
		UINT16 latch;	// 11 bits
	} pitch;

	struct
	{
		UINT8 value;	// 3 bits
		UINT8 reload;	// 3 bits
		UINT8 latch;	// 3 bits
	} octave;

	struct
	{
		UINT32 value;	// 20 bits
		UINT32 delta;	// 20 bits
	} phase;

	struct
	{
		UINT8 state;
		INT32 rate;
		INT32 divider;
		UINT32 steps;
		UINT16 value;	// 10 bits

		UINT8 rateScaling;	// 2 bits
		UINT8 attackRate;	// 5 bits
		UINT8 decayRate;	// 5 bits
		UINT8 sustainRate;	// 5 bits
		UINT8 sustainLevel;	// 4 bits
		UINT8 releaseRate;	// 5 bits
	} envelope;

	struct
	{
		UINT8 enable;		// 1 bit
		UINT8 attack;		// 1 bit
		UINT8 alternate;	// 1 bit
		UINT8 hold;			// 1 bit
		UINT8 invert;		// 1 bit
	} ssg;
} YM2612_OPERATOR;

typedef struct _ym2612_channel
{
	UINT8 leftEnable;	// 1 bit
	UINT8 rightEnable;	// 1 bit

	UINT8 algorithm;	// 3 bits
	UINT8 feedback;		// 3 bits
	UINT8 vibrato;		// 3 bits
	UINT8 tremolo;		// 2 bits

	UINT8 mode;			// 2 bits

	YM2612_OPERATOR op[4];
} YM2612_CHANNEL;

typedef struct _ym2612_state
{
	DEV_DATA _devData;

	UINT16 address;	// 9 bits

	struct
	{
		UINT8 enable;	// 1 bit
		UINT8 rate;		// 3 bits
		UINT32 clock;
		UINT32 divider;
	} lfo;

	struct
	{
		UINT8 enable;	// 1 bit
		UINT8 sample;
	} dac;

	struct
	{
		UINT16 clock;	// 12 bits
		UINT32 divider;
	} envelope;

	struct
	{
		UINT8 enable;		// 1 bit
		UINT8 enableLatch;	// 1 bit
		UINT8 irq;			// 1 bit
		UINT8 line;			// 1 bit
		UINT16 period;		// 10 bits
		UINT16 counter;		// 10 bits
	} timerA;

	struct
	{
		UINT8 enable;		// 1 bit
		UINT8 enableLatch;	// 1 bit
		UINT8 irq;			// 1 bit
		UINT8 line;			// 1 bit
		UINT8 period;
		UINT8 counter;
		UINT8 divider;		// 4 bits
	} timerB;

	YM2612_CHANNEL channels[6];

	UINT16 sine[0x400];
	INT16 pow2[0x200];

	UINT32 muteMask;
} YM2612_STATE;


static const UINT8 lfoDividers[8] =
{
	108, 77, 71, 67, 62, 44, 8, 5,
};

static const UINT8 vibratos[8][16] =
{
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
	{0, 0, 0, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0},
	{0, 0, 1, 1, 2, 2, 3, 3, 3, 3, 2, 2, 1, 1, 0, 0},
	{0, 0, 1, 2, 2, 2, 3, 4, 4, 3, 2, 2, 2, 1, 0, 0},
	{0, 0, 2, 3, 4, 4, 5, 6, 6, 5, 4, 4, 3, 2, 0, 0},
	{0, 0, 4, 6, 8, 8,10,12,12,10, 8, 8, 6, 4, 0, 0},
	{0, 0, 8,12,16,16,20,24,24,20,16,16,12, 8, 0, 0},
};

static const UINT8 tremolos[4] =
{
	7, 3, 1, 0,
};

static const UINT8 detunes[3][8] =
{
	{ 5,  6,  6,  7,  8,  8,  9, 10},
	{11, 12, 13, 14, 16, 17, 18, 20},
	{16, 17, 19, 20, 22, 24, 26, 28},
};

static const ENVELOPE_RATE envelopeRates[16] =
{
	{11, {0x00000000, 0x00000000, 0x01010101, 0x01010101}},
	{10, {0x01010101, 0x01010101, 0x01110111, 0x01110111}},
	{ 9, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 8, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 7, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 6, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 5, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 4, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 3, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 2, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 1, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 0, {0x01010101, 0x01011101, 0x01110111, 0x01111111}},
	{ 0, {0x11111111, 0x11121112, 0x12121212, 0x12221222}},
	{ 0, {0x22222222, 0x22242224, 0x24242424, 0x24442444}},
	{ 0, {0x44444444, 0x44484448, 0x48484848, 0x48884888}},
	{ 0, {0x88888888, 0x88888888, 0x88888888, 0x88888888}},
};


//----------------------------------------------------------------------------
// operator
//----------------------------------------------------------------------------

static void op_update_envelope(YM2612_OPERATOR* op)
{
	UINT32 rate = 0;
	const ENVELOPE_RATE* entry;

	if (op->envelope.state == EG_ATTACK)  rate += (op->envelope.attackRate  << 1);
	if (op->envelope.state == EG_DECAY)   rate += (op->envelope.decayRate   << 1);
	if (op->envelope.state == EG_SUSTAIN) rate += (op->envelope.sustainRate << 1);
	if (op->envelope.state == EG_RELEASE) rate += (op->envelope.releaseRate << 1);

	if (rate > 0)
		rate += op->keyScale >> (3 - op->envelope.rateScaling);
	if (rate > 63)
		rate = 63;

	entry = &envelopeRates[rate >> 2];
	op->envelope.rate    = (INT32)rate;
	op->envelope.divider = (INT32)entry->divider;
	op->envelope.steps   = entry->steps[rate & 3];
	return;
}

static void op_update_phase(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	UINT32 tuning;
	UINT32 lfo;
	INT32 pm;
	UINT32 delta;

	if (op->detune & 3)
		tuning = detunes[(op->detune & 3) - 1][op->keyScale & 7] >> (3 - (op->keyScale >> 3));
	else
		tuning = 0;
	lfo = (chip->lfo.clock >> 2) & 0x1f;
	pm = (op->pitch.value * vibratos[ch->vibrato][lfo & 15]) >> 9;
	if (lfo > 15)
		pm = -pm;

	delta = ((UINT32)(op->pitch.value + pm) << 6 >> (7 - op->octave.value)) & 0xfffff;
	delta = ((op->detune & 4) ? delta - tuning : delta + tuning) & 0x1ffff;
	delta = (op->multiple ? delta * op->multiple : delta >> 1) & 0xfffff;
	op->phase.delta = delta;
	return;
}

static void op_update_level(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	UINT32 lfo;
	UINT32 depth;
	UINT32 value;
	UINT8 invert;

	if (chip->lfo.clock & 0x40)
		lfo = chip->lfo.clock & 0x3f;
	else
		lfo = ~chip->lfo.clock & 0x3f;
	depth = tremolos[op->tremoloEnable * ch->tremolo];

	invert = (op->ssg.attack != op->ssg.invert) && op->envelope.state != EG_RELEASE;
	if (op->ssg.enable && invert)
		value = (0x200 - op->envelope.value) & 0x3ff;
	else
		value = op->envelope.value;

	op->outputLevel = (UINT16)((((UINT32)op->totalLevel << 3) + value + (lfo << 1 >> depth)) << 3);
	return;
}

static void op_update_pitch(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	UINT32 key;

	// only channel[2] allows per-operator frequencies
	// implemented by forcing mode to zero (single frequency) for other channels
	// in single frequency mode, operator[3] frequency is used for all operators
	op->pitch.value = ch->mode ? op->pitch.reload : ch->op[3].pitch.reload;
	op->octave.value = ch->mode ? op->octave.reload : ch->op[3].octave.reload;

	key = op->pitch.value;
	if (key < 0x300)
		key = 0x300;
	if (key > 0x4ff)
		key = 0x4ff;
	op->keyScale = (UINT8)(((op->octave.value << 2) + ((key - 0x300) >> 7)) & 0x1f);

	op_update_phase(chip, ch, op);
	op_update_envelope(op);	// due to key scaling
	return;
}

static void op_update_key_state(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	if (op->keyOn == op->keyLine)
		return;	// no change
	op->keyOn = op->keyLine;

	if (op->keyOn)
	{
		// restart phase and envelope generators
		op->phase.value = 0;
		op->ssg.invert = 0;
		op->envelope.state = EG_ATTACK;
		op_update_envelope(op);

		if (op->envelope.rate >= 62)
		{
			// skip attack phase
			op->envelope.value = 0;
		}
	}
	else
	{
		op->envelope.state = EG_RELEASE;
		op_update_envelope(op);

		if (op->ssg.enable && op->ssg.attack != op->ssg.invert)
		{
			// SSG-EG key-off
			op->envelope.value = (0x200 - op->envelope.value) & 0x3ff;
		}
	}

	op_update_level(chip, ch, op);
	return;
}

static void op_run_envelope(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	UINT32 sustain;
	UINT32 value;
	UINT32 step;

	if (chip->envelope.clock & ((1 << op->envelope.divider) - 1))
		return;

	if (op->envelope.state == EG_ATTACK && op->envelope.value == 0)
	{
		op->envelope.state = EG_DECAY;
		op_update_envelope(op);
	}

	sustain = (op->envelope.sustainLevel < 15) ? (op->envelope.sustainLevel << 5) : (0x1f << 5);
	if (op->envelope.state == EG_DECAY && op->envelope.value >= sustain)
	{
		op->envelope.state = EG_SUSTAIN;
		op_update_envelope(op);
	}

	value = (UINT32)chip->envelope.clock >> op->envelope.divider;
	step = (op->envelope.steps >> ((~value & 7) << 2)) & 0xf;

	if (op->envelope.state == EG_ATTACK)
	{
		// will stop updating if attack rate is increased to upper threshold during attack phase (confirmed behavior)
		if (op->envelope.rate < 62)
			op->envelope.value = (UINT16)((op->envelope.value + (~(UINT32)op->envelope.value * step >> 4)) & 0x3ff);
	}
	if (op->envelope.state != EG_ATTACK)
	{
		if (op->ssg.enable)
			step = (op->envelope.value < 0x200) ? (step << 2) : 0;	// SSG results in a 4x faster envelope
		value = op->envelope.value + step;
		op->envelope.value = (UINT16)((value < 0x3ff) ? value : 0x3ff);
	}

	op_update_level(chip, ch, op);
	return;
}

static void op_run_phase(const YM2612_STATE* chip, const YM2612_CHANNEL* ch, YM2612_OPERATOR* op)
{
	op_update_key_state(chip, ch, op);
	op->phase.value = (op->phase.value + op->phase.delta) & 0xfffff;	// advance wave position
	if (! (op->ssg.enable && op->envelope.value >= 0x200))
		return;	// SSG loop check

	if (! op->ssg.hold && ! op->ssg.alternate)
		op->phase.value = 0;
	if (! (op->ssg.hold && op->ssg.invert))
		op->ssg.invert ^= op->ssg.alternate;

	if (op->envelope.state == EG_ATTACK)
	{
		// do nothing; SSG is meant to skip the attack phase
	}
	else if (op->envelope.state != EG_RELEASE && ! op->ssg.hold)
	{
		// if still looping, reset the envelope
		op->envelope.state = EG_ATTACK;
		op_update_envelope(op);

		if (op->envelope.rate >= 62)
		{
			// skip attack phase
			op->envelope.value = 0;
		}
	}
	else if (op->envelope.state == EG_RELEASE || (op->ssg.hold && op->ssg.attack == op->ssg.invert))
	{
		// clear envelope once finished
		op->envelope.value = 0x3ff;
	}

	op_update_level(chip, ch, op);
	return;
}

static INT32 op_wave(const YM2612_STATE* chip, const YM2612_OPERATOR* op, UINT32 modulation)
{
	UINT32 x;
	INT32 y;

	x = (modulation >> 1) + (op->phase.value >> 10);
	y = chip->sine[x & 0x3ff] + op->outputLevel;
	if (y >= 0x1a00)
		return 0;	// -78 dB floor
	return (chip->pow2[y & 0x1ff] * 4) >> (y >> 9);
}


//----------------------------------------------------------------------------
// timers
//----------------------------------------------------------------------------

static void timer_a_run(YM2612_STATE* chip)
{
	chip->timerA.counter = (chip->timerA.counter + 1) & 0x3ff;
	if (! chip->timerA.counter)
		chip->timerA.line |= chip->timerA.irq & chip->timerA.enableLatch;
	if (chip->timerA.enableLatch < chip->timerA.enable || ! chip->timerA.counter)
		chip->timerA.counter = chip->timerA.period;
	chip->timerA.enableLatch = chip->timerA.enable;
	return;
}

static void timer_b_run(YM2612_STATE* chip)
{
	chip->timerB.divider = (chip->timerB.divider + 1) & 0x0f;
	if (! chip->timerB.divider)
	{
		chip->timerB.counter ++;
		if (! chip->timerB.counter)
			chip->timerB.line |= chip->timerB.irq & chip->timerB.enableLatch;
	}
	if (chip->timerB.enableLatch < chip->timerB.enable || (! chip->timerB.counter && ! chip->timerB.divider))
		chip->timerB.counter = chip->timerB.period;	// do not reset divider on reenable
	chip->timerB.enableLatch = chip->timerB.enable;
	return;
}


//----------------------------------------------------------------------------
// chip
//----------------------------------------------------------------------------

static void ym2612_clock(YM2612_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 left = 0;
	INT32 right = 0;
	UINT8 curChn;
	UINT8 curOp;

	timer_a_run(chip);
	timer_b_run(chip);

	if (++chip->envelope.divider == 3)
	{
		chip->envelope.divider = 0;
		// 12-bit counter: 1..4095 - zero-value is skipped (confirmed behavior)
		chip->envelope.clock = (chip->envelope.clock + 1) & 0xfff;
		if (! chip->envelope.clock)
			chip->envelope.clock = 1;
	}

	if (chip->lfo.enable && ++chip->lfo.divider >= lfoDividers[chip->lfo.rate])
	{
		chip->lfo.divider = 0;
		chip->lfo.clock ++;
		for (curChn = 0; curChn < 6; curChn ++)
		{
			YM2612_CHANNEL* ch = &chip->channels[curChn];
			for (curOp = 0; curOp < 4; curOp ++)
			{
				op_update_phase(chip, ch, &ch->op[curOp]);	// due to vibrato
				op_update_level(chip, ch, &ch->op[curOp]);	// due to tremolo
			}
		}
	}

	for (curChn = 0; curChn < 6; curChn ++)
	{
		YM2612_CHANNEL* ch = &chip->channels[curChn];
		YM2612_OPERATOR* op = ch->op;
		INT32 feedback;
		INT32 accumulator = 0;
		INT32 voiceData;
		UINT8 muted;

#define OP_OLD(n)	((INT32)op[n].prior  & MOD_MASK)
#define OP_MOD(n)	((INT32)op[n].output & MOD_MASK)
#define OP_OUT(n)	((INT32)op[n].output & SUM_MASK)
#define OP_WAVE(n, modulation)	(INT16)op_wave(chip, &op[n], (UINT32)(modulation))

		op[0].priorBuffer = op[0].prior;	// only need to buffer the output for feedback
		for (curOp = 0; curOp < 4; curOp ++)
			op[curOp].prior = op[curOp].output;

		for (curOp = 0; curOp < 4; curOp ++)
		{
			op_run_phase(chip, ch, &op[curOp]);
			if (chip->envelope.divider)
				continue;
			op_run_envelope(chip, ch, &op[curOp]);
		}

		feedback = MOD_MASK & ((op[0].prior + op[0].priorBuffer) >> (9 - ch->feedback));

		op[0].output = OP_WAVE(0, ch->feedback > 0 ? feedback : 0);

		// Note: Despite not emulating per cycle, operator pipelining has been accounted for.
		// Given the op order 0-2-1-3, newly generated output is not available for the following op.

		switch(ch->algorithm)
		{
		case 0:
			// 0 -> 1 -> 2 -> 3
			op[1].output = OP_WAVE(1, OP_MOD(0));
			op[2].output = OP_WAVE(2, OP_OLD(1));
			op[3].output = OP_WAVE(3, OP_MOD(2));
			accumulator += OP_OUT(3);
			break;
		case 1:
			// (0 + 1) -> 2 -> 3
			op[1].output = OP_WAVE(1, 0);
			op[2].output = OP_WAVE(2, OP_OLD(0) + OP_OLD(1));
			op[3].output = OP_WAVE(3, OP_MOD(2));
			accumulator += OP_OUT(3);
			break;
		case 2:
			// 0 + (1 -> 2) -> 3
			op[1].output = OP_WAVE(1, 0);
			op[2].output = OP_WAVE(2, OP_OLD(1));
			op[3].output = OP_WAVE(3, OP_MOD(0) + OP_MOD(2));
			accumulator += OP_OUT(3);
			break;
		case 3:
			// (0 -> 1) + 2 -> 3
			op[1].output = OP_WAVE(1, OP_MOD(0));
			op[2].output = OP_WAVE(2, 0);
			op[3].output = OP_WAVE(3, OP_OLD(1) + OP_MOD(2));
			accumulator += OP_OUT(3);
			break;
		case 4:
			// (0 -> 1) + (2 -> 3)
			op[1].output = OP_WAVE(1, OP_MOD(0));
			op[2].output = OP_WAVE(2, 0);
			op[3].output = OP_WAVE(3, OP_MOD(2));
			accumulator += OP_OUT(1) + OP_OUT(3);
			break;
		case 5:
			// 0 -> (1 + 2 + 3)
			op[1].output = OP_WAVE(1, OP_MOD(0));
			op[2].output = OP_WAVE(2, OP_OLD(0));
			op[3].output = OP_WAVE(3, OP_MOD(0));
			accumulator += OP_OUT(1) + OP_OUT(2) + OP_OUT(3);
			break;
		case 6:
			// (0 -> 1) + 2 + 3
			op[1].output = OP_WAVE(1, OP_MOD(0));
			op[2].output = OP_WAVE(2, 0);
			op[3].output = OP_WAVE(3, 0);
			accumulator += OP_OUT(1) + OP_OUT(2) + OP_OUT(3);
			break;
		case 7:
			// 0 + 1 + 2 + 3
			op[1].output = OP_WAVE(1, 0);
			op[2].output = OP_WAVE(2, 0);
			op[3].output = OP_WAVE(3, 0);
			accumulator += OP_OUT(0) + OP_OUT(1) + OP_OUT(2) + OP_OUT(3);
			break;
		}

#undef OP_OLD
#undef OP_MOD
#undef OP_OUT
#undef OP_WAVE

		if (accumulator < -0x2000)
			accumulator = -0x2000;
		else if (accumulator > 0x1fff)
			accumulator = 0x1fff;
		voiceData = accumulator & OUT_MASK;
		muted = (chip->muteMask >> curChn) & 0x01;
		if (chip->dac.enable && curChn == 5)
		{
			voiceData = ((INT32)chip->dac.sample - 0x80) * 64;
			muted = (chip->muteMask >> 6) & 0x01;
		}
		if (muted)
			continue;

		// DAC output + voltage offset (crossover distortion)
		if (voiceData < 0)
		{
			left  += ch->leftEnable  * (voiceData + (1<<5)) - (4<<5);
			right += ch->rightEnable * (voiceData + (1<<5)) - (4<<5);
		}
		else
		{
			left  += ch->leftEnable  * (voiceData + (0<<5)) + (4<<5);
			right += ch->rightEnable * (voiceData + (0<<5)) + (4<<5);
		}
	}

	if (left < -0x8000)
		left = -0x8000;
	else if (left > 0x7fff)
		left = 0x7fff;
	if (right < -0x8000)
		right = -0x8000;
	else if (right > 0x7fff)
		right = 0x7fff;
	*outL = left;
	*outR = right;
	return;
}

static void channel_power(const YM2612_STATE* chip, YM2612_CHANNEL* ch)
{
	UINT8 curOp;

	ch->leftEnable = 1;
	ch->rightEnable = 1;

	ch->algorithm = 0;
	ch->feedback = 0;
	ch->vibrato = 0;
	ch->tremolo = 0;

	ch->mode = 0;

	for (curOp = 0; curOp < 4; curOp ++)
	{
		YM2612_OPERATOR* op = &ch->op[curOp];

		memset(op, 0x00, sizeof(YM2612_OPERATOR));
		op->outputLevel = 0x1fff;
		op->envelope.state = EG_RELEASE;
		op->envelope.divider = 11;
		op->envelope.value = 0x3ff;
		op->envelope.releaseRate = 1;
	}
	for (curOp = 0; curOp < 4; curOp ++)
	{
		op_update_pitch(chip, ch, &ch->op[curOp]);
		op_update_level(chip, ch, &ch->op[curOp]);
	}
	return;
}

static void ym2612_power(YM2612_STATE* chip)
{
	UINT8 curChn;

	chip->address = 0;
	memset(&chip->lfo, 0x00, sizeof(chip->lfo));
	chip->dac.enable = 0;
	chip->dac.sample = 0x80;
	memset(&chip->envelope, 0x00, sizeof(chip->envelope));
	memset(&chip->timerA, 0x00, sizeof(chip->timerA));
	memset(&chip->timerB, 0x00, sizeof(chip->timerB));
	for (curChn = 0; curChn < 6; curChn ++)
		channel_power(chip, &chip->channels[curChn]);
	return;
}

static void ym2612_init_tables(YM2612_STATE* chip)
{
	const UINT32 positive = 0;
	const UINT32 negative = 1;
	INT32 x;
	INT32 y;
	INT32 z;

	for (x = 0; x <= 0xff; x ++)
	{
		y = (INT32)(-256 * log(sin((2 * x + 1) * M_PI / 1024)) / log(2.0) + 0.5);
		chip->sine[0x000 + x] = (UINT16)(positive + (y << 1));
		chip->sine[0x1ff - x] = (UINT16)(positive + (y << 1));
		chip->sine[0x200 + x] = (UINT16)(negative + (y << 1));
		chip->sine[0x3ff - x] = (UINT16)(negative + (y << 1));
	}

	for (y = 0; y <= 0xff; y ++)
	{
		z = (INT32)(1024 * pow(2.0, (0xff - y) / 256.0) + 0.5);
		chip->pow2[positive + (y << 1)] = (INT16)+z;
		chip->pow2[negative + (y << 1)] = (INT16)~z;	// not -z
	}
	return;
}

static void ym2612_write_data(YM2612_STATE* chip, UINT8 data)
{
	UINT16 address = chip->address;
	YM2612_CHANNEL* ch;
	YM2612_OPERATOR* op;
	UINT8 voice;
	UINT8 index;
	UINT8 curOp;

	switch(address)
	{
	case 0x022:	// LFO
		chip->lfo.rate = data & 0x07;
		chip->lfo.enable = (data >> 3) & 0x01;
		if (! chip->lfo.enable)
		{
			chip->lfo.clock = 0;
			chip->lfo.divider = 0;
		}
		break;
	case 0x024:	// timer A period (high)
		chip->timerA.period = (chip->timerA.period & 0x003) | (data << 2);
		break;
	case 0x025:	// timer A period (low)
		chip->timerA.period = (chip->timerA.period & 0x3fc) | (data & 0x03);
		break;
	case 0x026:	// timer B period
		chip->timerB.period = data;
		// Not sure if this is the right place for it, but...
		// handle a specific use case to reset the subticks on timer B
		if (! chip->timerB.enable)
			chip->timerB.divider = 0;
		break;
	case 0x027:	// timer control
		chip->timerA.enable = (data >> 0) & 0x01;
		chip->timerB.enable = (data >> 1) & 0x01;
		chip->timerA.irq = (data >> 2) & 0x01;
		chip->timerB.irq = (data >> 3) & 0x01;

		if (data & 0x10)
			chip->timerA.line = 0;
		if (data & 0x20)
			chip->timerB.line = 0;

		// d7 is enable bit for CSM mode (not yet implemented)
		ch = &chip->channels[2];
		ch->mode = (data >> 6) & 0x03;
		for (curOp = 0; curOp < 4; curOp ++)
			op_update_pitch(chip, ch, &ch->op[curOp]);
		break;
	case 0x028:	// key on/off
		// 0,1,2,4,5,6 => 0,1,2,3,4,5
		index = data & 0x07;
		if (index == 3 || index == 7)
			break;
		if (index >= 4)
			index --;

		ch = &chip->channels[index];
		ch->op[0].keyLine = (data >> 4) & 0x01;
		ch->op[1].keyLine = (data >> 5) & 0x01;
		ch->op[2].keyLine = (data >> 6) & 0x01;
		ch->op[3].keyLine = (data >> 7) & 0x01;
		break;
	case 0x02a:	// DAC sample
		chip->dac.sample = data;
		break;
	case 0x02b:	// DAC enable
		chip->dac.enable = (data >> 7) & 0x01;
		break;
	}

	if ((address & 0x03) == 3)
		return;
	voice = ((address >> 8) & 0x01) * 3 + (address & 0x03);
	index = (address >> 2) & 0x03;
	index = ((index >> 1) | (index << 1)) & 0x03;	// 0,1,2,3 => 0,2,1,3

	ch = &chip->channels[voice];
	op = &ch->op[index];

	switch(address & 0x0f0)
	{
	case 0x030:	// detune, multiple
		op->multiple = data & 0x0f;
		op->detune = (data >> 4) & 0x07;
		op_update_phase(chip, ch, op);
		break;
	case 0x040:	// total level
		op->totalLevel = data & 0x7f;
		op_update_level(chip, ch, op);
		break;
	case 0x050:	// rate scaling, attack rate
		op->envelope.attackRate = data & 0x1f;
		op->envelope.rateScaling = (data >> 6) & 0x03;
		op_update_envelope(op);
		op_update_phase(chip, ch, op);
		break;
	case 0x060:	// LFO AM enable, decay rate
		op->envelope.decayRate = data & 0x1f;
		op->tremoloEnable = (data >> 7) & 0x01;
		op_update_envelope(op);
		op_update_level(chip, ch, op);
		break;
	case 0x070:	// sustain rate
		op->envelope.sustainRate = data & 0x1f;
		op_update_envelope(op);
		break;
	case 0x080:	// sustain level, release rate
		op->envelope.releaseRate = ((data & 0x0f) << 1) | 1;
		op->envelope.sustainLevel = (data >> 4) & 0x0f;
		op_update_envelope(op);
		break;
	case 0x090:	// SSG-EG
		op->ssg.enable = (data >> 3) & 0x01;
		op->ssg.hold      = op->ssg.enable ? ((data >> 0) & 0x01) : 0;
		op->ssg.alternate = op->ssg.enable ? ((data >> 1) & 0x01) : 0;
		op->ssg.attack    = op->ssg.enable ? ((data >> 2) & 0x01) : 0;
		op_update_level(chip, ch, op);
		break;
	}

	switch(address & 0x0fc)
	{
	case 0x0a0:	// pitch (low)
		ch->op[3].pitch.reload = (ch->op[3].pitch.latch | data) & 0x7ff;
		ch->op[3].octave.reload = ch->op[3].octave.latch;
		for (curOp = 0; curOp < 4; curOp ++)
			op_update_pitch(chip, ch, &ch->op[curOp]);
		break;
	case 0x0a4:	// pitch (high)
		ch->op[3].pitch.latch = (data << 8) & 0x7ff;
		ch->op[3].octave.latch = (data >> 3) & 0x07;
		break;
	case 0x0a8:	// per-operator pitch (low)
		if (address == 0x0a9) index = 0;
		if (address == 0x0aa) index = 1;
		if (address == 0x0a8) index = 2;
		ch = &chip->channels[2];
		op = &ch->op[index];
		op->pitch.reload = (op->pitch.latch | data) & 0x7ff;
		op->octave.reload = op->octave.latch;
		op_update_pitch(chip, ch, op);
		break;
	case 0x0ac:	// per-operator pitch (high)
		if (address == 0x0ad) index = 0;
		if (address == 0x0ae) index = 1;
		if (address == 0x0ac) index = 2;
		op = &chip->channels[2].op[index];
		op->pitch.latch = (data << 8) & 0x7ff;
		op->octave.latch = (data >> 3) & 0x07;
		break;
	case 0x0b0:	// algorithm, feedback
		ch->algorithm = data & 0x07;
		ch->feedback = (data >> 3) & 0x07;
		break;
	case 0x0b4:	// panning, tremolo, vibrato
		ch->vibrato = data & 0x07;
		ch->tremolo = (data >> 4) & 0x03;
		ch->rightEnable = (data >> 6) & 0x01;
		ch->leftEnable = (data >> 7) & 0x01;
		for (curOp = 0; curOp < 4; curOp ++)
		{
			op_update_level(chip, ch, &ch->op[curOp]);
			op_update_phase(chip, ch, &ch->op[curOp]);
		}
		break;
	}
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

static UINT8 device_start_ym2612(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YM2612_STATE* chip;

	chip = (YM2612_STATE*)calloc(1, sizeof(YM2612_STATE));
	if (chip == NULL)
		return 0xFF;

	ym2612_init_tables(chip);
	chip->muteMask = 0x00;
	ym2612_power(chip);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / 144, &devDef);
	return 0x00;
}

static void device_stop_ym2612(void* info)
{
	free(info);
	return;
}

static void device_reset_ym2612(void* info)
{
	ym2612_power((YM2612_STATE*)info);
	return;
}

static void ym2612_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2612_clock(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2612_w(void* info, UINT8 offset, UINT8 data)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	// offset 0/1: address/data port 0, offset 2/3: address/data port 1
	if (offset & 0x01)
		ym2612_write_data(chip, data);
	else
		chip->address = ((offset & 0x02) << 7) | data;
	return;
}

static UINT8 ym2612_r(void* info, UINT8 offset)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	// d7 = busy (not emulated, requires cycle timing accuracy)
	return (chip->timerA.line << 0) | (chip->timerB.line << 1);
}

static void ym2612_set_mute_mask(void* info, UINT32 MuteMask)
{
	YM2612_STATE* chip = (YM2612_STATE*)info;

	chip->muteMask = MuteMask;
	return;
}
