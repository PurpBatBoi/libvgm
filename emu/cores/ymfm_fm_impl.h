/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    The per-sample part of the shared FM engine (ymfm_fm.c): clocking and
    output. This is the counterpart of ymfm's ymfm_fm.ipp, which each
    family's .cpp includes so that C++ builds one engine per register
    family. Here each family file includes this header and calls
    fm_engine_clock() and fm_engine_output() with its own const FM_FAMILY
    table. The compiler then sees the table as a constant: the family
    flags fold away and the per-sample family functions (LFO and noise
    clock, phase step) become direct calls. With the table read at run
    time instead, the OPL/OPL2 ran about 20% slower than version 1.
    The rare paths (writes, key state, prepare, timers, reset) stay in
    ymfm_fm.c.

    The family passed in must be the table given to fm_engine_init().

***************************************************************************/

#ifndef __YMFM_FM_IMPL_H__
#define __YMFM_FM_IMPL_H__

#include <stddef.h>	// for NULL

#include "ymfm_fm.h"


// the functions that take the family must be inlined into each family file,
// or the compiler may share one copy between two tables and nothing folds
#if defined(_MSC_VER)
#define FM_FORCE_INLINE	static __forceinline
#elif defined(__GNUC__)
#define FM_FORCE_INLINE	static __inline__ __attribute__((always_inline))
#else
#define FM_FORCE_INLINE	INLINE
#endif

// "quiet" value, used to optimize when we can skip doing work
#define EG_QUIET	0x380

// table of ymfm_fm.c
extern const UINT16 fm_algorithm_ops[8+4];

// rare paths of ymfm_fm.c, called from the per-sample code
void fm_engine_timer_expired(FM_ENGINE* fm, UINT32 tnum);
void fm_engine_prepare(FM_ENGINE* fm, UINT32 chanmask);

//----------------------------------------------------------------------------
// operator
//----------------------------------------------------------------------------

#define op_ssg_eg_enable(op)	bitfield((op)->cache.ssg_eg, 3, 1)
#define op_ssg_eg_mode(op)		bitfield((op)->cache.ssg_eg, 0, 3)

// return the current phase value
#define op_phase(op)	((op)->phase >> 10)

// start the attack phase; called when a keyon happens or when an SSG-EG
// cycle is complete and restarts
INLINE void op_start_attack(FM_OPERATOR* op, UINT8 is_restart)
{
	// don't change anything if already in attack state
	if (op->env_state == EG_ATTACK)
		return;
	op->env_state = EG_ATTACK;

	// generally not inverted at start, except if SSG-EG is enabled and
	// one of the inverted modes is specified; leave this alone on a
	// restart, as it is managed by the clock_ssg_eg_state() code
	if (! is_restart)
		op->ssg_inverted = (UINT8)(op_ssg_eg_enable(op) & bitfield(op_ssg_eg_mode(op), 2, 1));

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

// clock the SSG-EG state; should only be called if SSG-EG is enabled
INLINE void op_clock_ssg_eg_state(FM_OPERATOR* op)
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
	mode = op_ssg_eg_mode(op);

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
			op_start_attack(op, 1);

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
INLINE void op_clock_envelope(FM_OPERATOR* op, UINT32 env_counter)
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
		if (! op_ssg_eg_enable(op))
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

// master clocking function
FM_FORCE_INLINE void op_clock(FM_ENGINE* fm, const FM_FAMILY* family, FM_OPERATOR* op, UINT32 env_counter, INT32 lfo_raw_pm)
{
	UINT32 phase_step;

	// clock the SSG-EG state (OPN/OPNA)
	if (family->eg_has_ssg)
	{
		if (op_ssg_eg_enable(op))
			op_clock_ssg_eg_state(op);
		else
			op->ssg_inverted = 0;
	}

	// clock the envelope if on an envelope cycle; env_counter is a x.2 value
	if (bitfield(env_counter, 0, 2) == 0)
		op_clock_envelope(op, env_counter >> 2);

	// clock the 10.10 phase value; read the step from the cache, or
	// recalculate it if PM is active
	phase_step = op->cache.phase_step;
	if (phase_step == PHASE_STEP_DYNAMIC)
		phase_step = family->compute_phase_step(fm->owner, op->choffs, op->opoffs, &op->cache, lfo_raw_pm);
	op->phase += phase_step;
	return;
}

// return the effective attenuation of the envelope
INLINE UINT32 op_envelope_attenuation(const FM_OPERATOR* op, UINT32 am_offset)
{
	UINT32 result = op->env_attenuation;

	// invert if necessary due to SSG-EG
	if (op->ssg_inverted)
		result = (0x200 - result) & 0x3ff;

	// add in LFO AM modulation
	if (op->cache.lfo_am_enable)
		result += am_offset;

	// add in total level and KSL from the cache
	result += op->cache.total_level;

	// clamp to max and return
	return (result < 0x3ff) ? result : 0x3ff;
}

// compute the 14-bit signed volume of this operator, given a phase
// modulation and an AM LFO offset
INLINE INT32 op_compute_volume(const FM_OPERATOR* op, UINT32 phase, UINT32 am_offset)
{
	UINT32 sin_attenuation;
	UINT32 env_attenuation;
	INT32 result;
	INT32 sign;

	// the low 10 bits of phase represents a full 2*PI period over
	// the full sin wave

	// early out if the envelope is effectively off
	if (op->env_attenuation > EG_QUIET)
		return 0;

	// get the absolute value of the sin, as attenuation, as a 4.8 fixed point value
	sin_attenuation = op->cache.waveform[phase & (FM_WAVEFORM_LENGTH - 1)];

	// get the attenuation from the evelope generator as a 4.6 value, shifted up to 4.8
	env_attenuation = op_envelope_attenuation(op, am_offset) << 2;

	// combine into a 5.8 value, then convert from attenuation to 13-bit linear volume
	result = (INT32)attenuation_to_volume((sin_attenuation & 0x7fff) + env_attenuation);

	// negate if in the negative part of the sin wave (sign bit gives 14 bits);
	// done without a branch (-result == ~result + 1), as the sign flips
	// unpredictably and a mispredicted branch costs more than the arithmetic
	sign = (INT32)bitfield(sin_attenuation, 15, 1);
	return (result ^ -sign) + sign;
}

// compute the 14-bit signed noise volume of this operator (OPM), given an
// AM offset
INLINE INT32 op_compute_noise_volume(const FM_ENGINE* fm, const FM_OPERATOR* op, UINT32 am_offset)
{
	// application manual says the logarithmic transform is not applied here, so we
	// just use the raw envelope attenuation, inverted (since 0 attenuation should be
	// maximum), and shift it up from a 10-bit value to an 11-bit value
	INT32 result = (INT32)((op_envelope_attenuation(op, am_offset) ^ 0x3ff) << 1);
	INT32 sign;

	// QUESTION: is AM applied still?

	// negate based on the noise state; no branch, as the noise is random
	sign = (INT32)bitfield(fm->noise_state, 0, 1);
	return (result ^ -sign) + sign;
}


//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

// AM amount is the same across all operators of a channel
#define channel_am_offset(fm, ch)	\
	(((UINT32)(fm)->lfo_am << (ch)->cache.am_lshift) >> (ch)->cache.am_rshift)

// operator 1 has optional self-feedback
#define channel_feedback_mod(ch)	\
	(((ch)->cache.feedback != 0) ? ((ch)->feedback[0] + (ch)->feedback[1]) >> (10 - (ch)->cache.feedback) : 0)

// master clock of all operators
FM_FORCE_INLINE void channel_clock(FM_ENGINE* fm, const FM_FAMILY* family, FM_CHANNEL* ch, UINT32 env_counter, INT32 lfo_raw_pm)
{
	UINT8 opnum;

	// clock the feedback through
	ch->feedback[0] = ch->feedback[1];
	ch->feedback[1] = ch->feedback_in;

	for (opnum = 0; opnum < 4 && ch->op[opnum] != NULL; opnum ++)
		op_clock(fm, family, ch->op[opnum], env_counter, lfo_raw_pm);
	return;
}

// combine 2 operators according to the specified algorithm, returning a sum
// according to the rshift and clipmax parameters, which vary between
// different implementations
FM_FORCE_INLINE INT32 channel_output_2op(const FM_ENGINE* fm, const FM_FAMILY* family, FM_CHANNEL* ch, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = ch->op[0];
	const FM_OPERATOR* op2 = ch->op[1];
	UINT32 am_offset;
	INT32 opmod;
	INT32 op1value;
	INT32 result;

	am_offset = channel_am_offset(fm, ch);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	opmod = channel_feedback_mod(ch);
	op1value = ch->feedback_in = (INT16)op_compute_volume(op1, op_phase(op1) + opmod, am_offset);

	// now that the feedback has been computed, skip the rest if all volumes
	// are clear; no need to do all this work for nothing
	if (ch->cache.outputs == 0)
		return 0;

	// Algorithms for two-operator case:
	//    0: O1 -> O2 -> out
	//    1: (O1 + O2) -> out
	if (bitfield(ch->cache.algorithm, 0, 1) == 0)
	{
		// some OPL chips use the previous sample for modulation instead of
		// the current sample
		opmod = (family->modulator_delay ? ch->feedback[1] : op1value) >> 1;
		result = op_compute_volume(op2, op_phase(op2) + opmod, am_offset) >> rshift;
	}
	else
	{
		result = (family->modulator_delay ? ch->feedback[1] : op1value) >> rshift;
		result += op_compute_volume(op2, op_phase(op2), am_offset) >> rshift;
		result = clamp(result, -clipmax - 1, clipmax);
	}
	return result;
}

// the 14-bit value of an operator, phase-modulated by the output of another
// operator (the modulation input is halved) or not modulated
#define OP_MODULATED(op, mod)	op_compute_volume(op, op_phase(op) + ((mod) >> 1), am_offset)
#define OP_UNMODULATED(op)		op_compute_volume(op, op_phase(op), am_offset)

// combine 4 operators according to the specified algorithm, returning a sum
// according to the rshift and clipmax parameters, which vary between
// different implementations
INLINE INT32 channel_output_4op(const FM_ENGINE* fm, FM_CHANNEL* ch, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = ch->op[0];
	const FM_OPERATOR* op2 = ch->op[1];
	const FM_OPERATOR* op3 = ch->op[2];
	const FM_OPERATOR* op4 = ch->op[3];
	UINT32 am_offset;
	INT32 opmod;
	INT32 op1value;
	UINT32 algorithm_ops;
	INT32 op2value;
	INT32 op3value;
	INT32 op4mod;
	INT32 result;
	INT32 clipmin;

	am_offset = channel_am_offset(fm, ch);

	// compute the 14-bit volume/value of operator 1 and update the feedback
	opmod = channel_feedback_mod(ch);
	op1value = ch->feedback_in = (INT16)op_compute_volume(op1, op_phase(op1) + opmod, am_offset);

	// now that the feedback has been computed, skip the rest if all volumes
	// are clear; no need to do all this work for nothing
	if (ch->cache.outputs == 0)
		return 0;

	algorithm_ops = fm_algorithm_ops[ch->cache.algorithm];

	// Each algorithm is a fixed wiring of the four operators. ymfm gathers the
	// operator inputs from a table of outputs (opout[]) indexed by the
	// algorithm; here each wiring is spelled out, so that the operator
	// values stay in registers and no table is indexed at run time. The
	// wiring is the one in the comments of fm_algorithm_ops (ymfm_fm.c), and
	// the values are the same (operator volumes are 14-bit, so their sums
	// fit the 16 bits of ymfm's table).
	switch (ch->cache.algorithm)
	{
	default:
	case 0:		// O1 -> O2 -> O3 -> O4
	case 8:
		op2value = OP_MODULATED(op2, op1value);
		op3value = OP_MODULATED(op3, op2value);
		op4mod = op3value;
		break;
	case 1:		// (O1 + O2) -> O3 -> O4
		op2value = OP_UNMODULATED(op2);
		op3value = OP_MODULATED(op3, op1value + op2value);
		op4mod = op3value;
		break;
	case 2:		// (O1 + (O2 -> O3)) -> O4
		op2value = OP_UNMODULATED(op2);
		op3value = OP_MODULATED(op3, op2value);
		op4mod = op1value + op3value;
		break;
	case 3:		// ((O1 -> O2) + O3) -> O4
		op2value = OP_MODULATED(op2, op1value);
		op3value = OP_UNMODULATED(op3);
		op4mod = op2value + op3value;
		break;
	case 4:		// (O1 -> O2) + (O3 -> O4)
	case 10:
		op2value = OP_MODULATED(op2, op1value);
		op3value = OP_UNMODULATED(op3);
		op4mod = op3value;
		break;
	case 5:		// (O1 -> O2) + (O1 -> O3) + (O1 -> O4)
		op2value = OP_MODULATED(op2, op1value);
		op3value = OP_MODULATED(op3, op1value);
		op4mod = op1value;
		break;
	case 6:		// (O1 -> O2) + O3 + O4
		op2value = OP_MODULATED(op2, op1value);
		op3value = OP_UNMODULATED(op3);
		op4mod = 0;
		break;
	case 7:		// O1 + O2 + O3 + O4
		op2value = OP_UNMODULATED(op2);
		op3value = OP_UNMODULATED(op3);
		op4mod = 0;
		break;
	case 9:		// O1 + (O2 -> O3 -> O4)
		op2value = OP_UNMODULATED(op2);
		op3value = OP_MODULATED(op3, op2value);
		op4mod = op3value;
		break;
	case 11:	// O1 + (O2 -> O3) + O4
		op2value = OP_UNMODULATED(op2);
		op3value = OP_MODULATED(op3, op2value);
		op4mod = 0;
		break;
	}

	// compute the 14-bit volume/value of operator 4; this could be a noise
	// value on the OPM; all algorithms consume OP4 output at a minimum
	if (ch->cache.kind == FM_CH_NOISE)
		result = op_compute_noise_volume(fm, op4, am_offset);
	else
		result = OP_MODULATED(op4, op4mod);
	result >>= rshift;

	// optionally add OP1, OP2, OP3
	clipmin = -clipmax - 1;
	if (bitfield(algorithm_ops, 7, 1) != 0)
		result = clamp(result + (op1value >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 8, 1) != 0)
		result = clamp(result + (op2value >> rshift), clipmin, clipmax);
	if (bitfield(algorithm_ops, 9, 1) != 0)
		result = clamp(result + (op3value >> rshift), clipmin, clipmax);
	return result;
}

// OPL channel 6 in rhythm mode, which outputs a Bass Drum instrument
INLINE INT32 channel_output_rhythm_ch6(const FM_ENGINE* fm, FM_CHANNEL* ch, UINT32 mute, UINT32 rshift)
{
	const FM_OPERATOR* op1 = ch->op[0];
	const FM_OPERATOR* op2 = ch->op[1];
	UINT32 am_offset;
	INT32 opmod;
	INT32 opout1;
	INT32 result;

	am_offset = channel_am_offset(fm, ch);

	// Bass Drum: this uses operators 12 and 15 (i.e., channel 6)
	// in an almost-normal way, except that if the algorithm is 1,
	// the first operator is ignored instead of added in

	// compute the 14-bit volume/value of operator 1 and update the feedback
	opmod = channel_feedback_mod(ch);
	opout1 = ch->feedback_in = (INT16)op_compute_volume(op1, op_phase(op1) + opmod, am_offset);

	// compute the 14-bit volume/value of operator 2, which is the result
	opmod = bitfield(ch->cache.algorithm, 0, 1) ? 0 : (opout1 >> 1);
	result = op_compute_volume(op2, op_phase(op2) + opmod, am_offset) >> rshift;
	if (bitfield(mute, FM_MUTE_BD, 1))
		result = 0;
	return result * 2;
}

// OPL channel 7 in rhythm mode, which outputs High Hat and Snare Drum
// instruments
INLINE INT32 channel_output_rhythm_ch7(const FM_ENGINE* fm, const FM_CHANNEL* ch, UINT32 phase_select,
	UINT32 mute, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = ch->op[0];
	const FM_OPERATOR* op2 = ch->op[1];
	UINT32 am_offset;
	UINT32 noise_state;
	UINT32 phase;
	INT32 hihat;
	INT32 snare;

	am_offset = channel_am_offset(fm, ch);
	noise_state = bitfield(fm->noise_state, 0, 1);

	// High Hat: this uses the envelope from operator 13 (channel 7),
	// and a combination of noise and the operator 13/17 phase select
	// to compute the phase
	phase = (phase_select << 9) | (0xd0 >> (2 * (noise_state ^ phase_select)));
	hihat = op_compute_volume(op1, phase, am_offset) >> rshift;
	if (bitfield(mute, FM_MUTE_HH, 1))
		hihat = 0;

	// Snare Drum: this uses the envelope from operator 16 (channel 7),
	// and a combination of noise and operator 13 phase to pick a phase
	phase = (0x100 << bitfield(op_phase(op1), 8, 1)) ^ (noise_state << 8);
	snare = op_compute_volume(op2, phase, am_offset) >> rshift;
	if (bitfield(mute, FM_MUTE_SD, 1))
		snare = 0;

	return clamp(hihat + snare, -clipmax - 1, clipmax) * 2;
}

// OPL channel 8 in rhythm mode, which outputs Tom Tom and Top Cymbal
// instruments
INLINE INT32 channel_output_rhythm_ch8(const FM_ENGINE* fm, const FM_CHANNEL* ch, UINT32 phase_select,
	UINT32 mute, UINT32 rshift, INT32 clipmax)
{
	const FM_OPERATOR* op1 = ch->op[0];
	const FM_OPERATOR* op2 = ch->op[1];
	UINT32 am_offset;
	UINT32 phase;
	INT32 tomtom;
	INT32 cymbal;

	am_offset = channel_am_offset(fm, ch);

	// Tom Tom: this is just a single operator processed normally
	tomtom = op_compute_volume(op1, op_phase(op1), am_offset) >> rshift;
	if (bitfield(mute, FM_MUTE_TT, 1))
		tomtom = 0;

	// Top Cymbal: this uses the envelope from operator 17 (channel 8),
	// and the operator 13/17 phase select to compute the phase
	phase = 0x100 | (phase_select << 9);
	cymbal = op_compute_volume(op2, phase, am_offset) >> rshift;
	if (bitfield(mute, FM_MUTE_CY, 1))
		cymbal = 0;

	return clamp(tomtom + cymbal, -clipmax - 1, clipmax) * 2;
}


//----------------------------------------------------------------------------
// engine
//----------------------------------------------------------------------------

// expire the timers, then clock all channels in chanmask forward one sample;
// returns the envelope counter, as it is used to clock ADPCM-A
FM_FORCE_INLINE UINT32 fm_engine_clock(FM_ENGINE* fm, const FM_FAMILY* family, UINT32 chanmask)
{
	UINT32 chnum;
	UINT32 tnum;
	INT32 lfo_raw_pm;

	// expire timers that run out on this sample
	for (tnum = 0; tnum < 2; tnum ++)
	{
		if (fm->timer_count[tnum] > 0 && --fm->timer_count[tnum] == 0)
			fm_engine_timer_expired(fm, tnum);
	}

	// update the clock counter
	fm->total_clocks ++;

	// if something was modified, prepare
	// also prepare every 4k samples to catch ending notes
	if (fm->modified_channels != 0 || fm->prepare_count++ >= 4096)
		fm_engine_prepare(fm, chanmask);

	// if the envelope clock divider is 1, just increment by 4;
	// otherwise, increment by 1 and manually wrap when we reach the divide count
	if (family->eg_clock_divider == 1)
		fm->env_counter += 4;
	else if (bitfield(++fm->env_counter, 0, 2) == family->eg_clock_divider)
		fm->env_counter += 4 - family->eg_clock_divider;

	// clock the noise generator and the LFO
	lfo_raw_pm = family->clock_noise_and_lfo(fm->owner, &fm->lfo_am, &fm->noise_state);

	// now update the state of all the channels and operators
	for (chnum = 0; chnum < family->channels; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1))
			channel_clock(fm, family, &fm->channel[chnum], fm->env_counter, lfo_raw_pm);
	}

	// return the envelope counter as it is used to clock ADPCM-A
	return fm->env_counter;
}

// precompute the operator 13+17 phase selection value of the OPL rhythm mode
INLINE UINT32 engine_rhythm_phase_select(const FM_ENGINE* fm)
{
	UINT32 op13phase = op_phase(&fm->op[13]);
	UINT32 op17phase = op_phase(&fm->op[17]);

	return (bitfield(op13phase, 2, 1) ^ bitfield(op13phase, 7, 1)) | bitfield(op13phase, 3, 1) |
		(bitfield(op17phase, 5, 1) ^ bitfield(op17phase, 3, 1));
}

// add the outputs of the channels in chanmask to output[], one entry per output
FM_FORCE_INLINE void fm_engine_output(FM_ENGINE* fm, const FM_FAMILY* family, INT32* output, UINT32 rshift, INT32 clipmax, UINT32 chanmask)
{
	UINT32 channels = family->channels;
	UINT32 rhythm_mute = fm->mute_mask >> channels;
	UINT32 chnum;
	UINT32 outputs;
	UINT32 outnum;
	UINT32 phase_select = 0;
	UINT8 have_phase_select = 0;
	INT32 result;
	FM_CHANNEL* ch;

	// mask out inactive channels
	chanmask &= fm->active_channels;

	// sum over all the desired channels
	for (chnum = 0; chnum < channels; chnum ++)
	{
		if (! bitfield(chanmask, chnum, 1))
			continue;
		ch = &fm->channel[chnum];

		// the OPL rhythm channels dedicate their operators to percussion
		switch(ch->cache.kind)
		{
		case FM_CH_RHYTHM_BD:
			result = channel_output_rhythm_ch6(fm, ch, rhythm_mute, rshift);
			break;
		case FM_CH_RHYTHM_HH_SD:
		case FM_CH_RHYTHM_TT_CY:
			// the phase selection is the same for both channels
			if (! have_phase_select)
			{
				phase_select = engine_rhythm_phase_select(fm);
				have_phase_select = 1;
			}
			if (ch->cache.kind == FM_CH_RHYTHM_HH_SD)
				result = channel_output_rhythm_ch7(fm, ch, phase_select, rhythm_mute, rshift, clipmax);
			else
				result = channel_output_rhythm_ch8(fm, ch, phase_select, rhythm_mute, rshift, clipmax);
			break;
		default:
			if (ch->op[2] != NULL)
				result = channel_output_4op(fm, ch, rshift, clipmax);
			else
				result = channel_output_2op(fm, family, ch, rshift, clipmax);
			if (bitfield(fm->mute_mask, chnum, 1))
				result = 0;
			break;
		}

		// add to the outputs the channel is enabled on
		outputs = ch->cache.outputs;
		if (family->outputs <= 2)
		{
			// masked adds instead of a branch per output
			for (outnum = 0; outnum < family->outputs; outnum ++)
				output[outnum] += result & -(INT32)((outputs >> outnum) & 1);
		}
		else
		{
			// with more outputs, most channels use only the first few, and
			// walking the set bits is cheaper than adding to every output
			for (outnum = 0; outputs != 0; outputs >>= 1, outnum ++)
			{
				if (outputs & 1)
					output[outnum] += result;
			}
		}
	}
	return;
}

#endif	// __YMFM_FM_IMPL_H__
