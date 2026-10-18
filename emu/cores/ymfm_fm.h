/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles
***************************************************************************/


#ifndef __YMFM_FM_H__
#define __YMFM_FM_H__

#include "../../stdtype.h"
#include "../../common_def.h"

#define FM_MAX_CHANNELS		18
#define FM_MAX_OPERATORS	36

// this is the size of a full sin waveform
#define FM_WAVEFORM_LENGTH	0x400

// this value is returned from the family write() function for rhythm channels
#define FM_RHYTHM_CHANNEL	0xff

// envelope states
#define EG_ATTACK	1
#define EG_DECAY	2
#define EG_SUSTAIN	3
#define EG_RELEASE	4
#define EG_STATES	5

// three different keyon sources; actual keyon is an OR over all of these
#define KEYON_NORMAL	0
#define KEYON_RHYTHM	1
#define KEYON_CSM		2

// set phase_step to this value to recalculate it each sample; needed
// in the case of PM LFO changes
#define PHASE_STEP_DYNAMIC	1

// how a channel computes its output; set by the family when it caches the channel
#define FM_CH_NORMAL		0	// 2 or 4 operators, from the operator assignment
#define FM_CH_NOISE			1	// 4 operators, operator 4 is the OPM noise generator
#define FM_CH_RHYTHM_BD		2	// OPL channel 6 in rhythm mode: Bass Drum
#define FM_CH_RHYTHM_HH_SD	3	// OPL channel 7 in rhythm mode: High Hat and Snare Drum
#define FM_CH_RHYTHM_TT_CY	4	// OPL channel 8 in rhythm mode: Tom Tom and Top Cymbal

// mute bits of the OPL rhythm instruments, above the channel bits of mute_mask
#define FM_MUTE_BD	0
#define FM_MUTE_SD	1
#define FM_MUTE_TT	2
#define FM_MUTE_CY	3
#define FM_MUTE_HH	4


// operator data that is computed once at the start of clocking and remains
// static during subsequent sound generation
typedef struct _fm_opdata_cache
{
	const UINT16* waveform;		// base of sine table
	UINT32 phase_step;			// phase step, or PHASE_STEP_DYNAMIC if PM is active
	UINT32 total_level;			// total level * 8 + KSL
	UINT32 block_freq;			// raw block frequency value (used to compute phase_step)
	INT32 detune;				// detuning value (used to compute phase_step)
	UINT32 multiple;			// multiple value (x.1, used to compute phase_step)
	UINT32 eg_sustain;			// sustain level, shifted up to envelope values
	UINT8 eg_rate[EG_STATES];	// envelope rate, including KSR
	UINT8 lfo_am_enable;		// non-zero if LFO AM applies to this operator
	UINT8 ssg_eg;				// SSG-EG register: bit 3 = enable, bits 0-2 = mode; 0 if not OPN
} FM_OPDATA_CACHE;

// channel data that is computed once at the start of clocking
typedef struct _fm_channel_cache
{
	UINT8 feedback;		// operator 1 feedback level, 0 = off
	UINT8 algorithm;	// algorithm (0-7, or 8-11 for OPL3 4-operator channels)
	UINT8 outputs;		// bit n set: the channel goes to output n; 0 = no output
	UINT8 am_lshift;	// LFO AM offset = (lfo_am << am_lshift) >> am_rshift
	UINT8 am_rshift;
	UINT8 kind;			// FM_CH_xxx
} FM_CHANNEL_CACHE;

typedef struct _fm_operator
{
	UINT32 choffs;			// channel offset in registers
	UINT32 opoffs;			// operator offset in registers
	UINT32 phase;			// current phase value (10.10 format)
	UINT16 env_attenuation;	// computed envelope attenuation (4.6 format)
	UINT8 env_state;		// current envelope state
	UINT8 ssg_inverted;		// non-zero if the output should be inverted (bit 0)
	UINT8 key_state;		// current key state: on or off (bit 0)
	UINT8 keyon_live;		// live key on state (bit 0 = direct, bit 1 = rhythm, bit 2 = CSM)
	UINT8 keyoff_pending;	// the live key went off since the last prepare
	FM_OPDATA_CACHE cache;	// cached values for performance
} FM_OPERATOR;

typedef struct _fm_channel
{
	UINT32 choffs;			// channel offset in registers
	INT16 feedback[2];		// feedback memory for operator 1
	INT16 feedback_in;		// next input value for op 1 feedback (set in output)
	FM_OPERATOR* op[4];		// up to 4 operators, NULL if unused
	FM_CHANNEL_CACHE cache;	// cached values for performance
} FM_CHANNEL;

// the mode register and timer values, decoded by the family
typedef struct _fm_mode
{
	UINT32 timer_a_value;
	UINT32 timer_b_value;
	UINT8 status_mask;		// status bits masked from reads and IRQs (OPL only)
	UINT8 irq_reset;		// IRQ reset bit (OPL only)
	UINT8 csm;
	UINT8 reset_timer_a;
	UINT8 reset_timer_b;
	UINT8 enable_timer_a;
	UINT8 enable_timer_b;
	UINT8 load_timer_a;
	UINT8 load_timer_b;
} FM_MODE;

// the family-specific parts of the engine; every function gets the owner
// pointer given to fm_engine_init()
typedef struct _fm_family
{
	UINT8 channels;			// number of channels on the chip
	UINT8 outputs;			// number of outputs of the chip
	UINT8 operators;		// number of operators on the chip
	UINT8 default_prescale;	// starting clock prescale
	UINT8 eg_clock_divider;	// clock divider of the envelope generator
	UINT16 reg_mode;		// address of the "mode" register controlling timers
	UINT32 csm_trigger_mask;	// mask of channels to trigger in CSM mode
	UINT8 status_timera;	// status bit to set when timer A fires
	UINT8 status_timerb;	// status bit to set when timer B fires
	UINT8 status_busy;		// status bit to set when the chip is busy
	UINT8 status_irq;		// status bit to set when an IRQ is signalled
	UINT8 dynamic_ops;		// non-zero if ops/channel can be changed at runtime (OPL3)
	UINT8 modulator_delay;	// non-zero if the modulator is delayed by 1 sample (OPL pre-OPL3)
	UINT8 eg_has_ssg;		// non-zero if the operators have SSG-EG (OPN)

	// register offsets of a channel and an operator
	UINT32 (*channel_offset)(UINT32 chnum);
	UINT32 (*operator_offset)(UINT32 opnum);

	// fill map[chnum] with the operators of each channel, 8 bits each, 0xff = none;
	// unused slots must come last (the per-sample code stops at the first one)
	void (*operator_map)(void* owner, UINT32* map);

	// reset the registers
	void (*reset)(void* owner);

	// write a register; return non-zero for a keyon write, with the channel
	// (or FM_RHYTHM_CHANNEL) and the operator mask
	UINT8 (*write)(void* owner, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask);

	// decode the mode register and timer values
	void (*read_mode)(void* owner, FM_MODE* mode);

	// clock the noise generator and the LFO, once per sample; set the LFO AM
	// value and the noise state (bit 0), and return the raw PM value
	INT32 (*clock_noise_and_lfo)(void* owner, UINT8* lfo_am, UINT8* noise_state);

	// cache the channel and operator data at the start of clocking
	void (*cache_channel_data)(void* owner, UINT32 choffs, FM_CHANNEL_CACHE* cache);
	void (*cache_operator_data)(void* owner, UINT32 choffs, UINT32 opoffs, FM_OPDATA_CACHE* cache);

	// compute the phase step while PM is active (cache->phase_step == PHASE_STEP_DYNAMIC)
	UINT32 (*compute_phase_step)(void* owner, UINT32 choffs, UINT32 opoffs, const FM_OPDATA_CACHE* cache, INT32 lfo_raw_pm);
} FM_FAMILY;

typedef struct _fm_engine
{
	const FM_FAMILY* family;
	void* owner;
	UINT32 env_counter;			// envelope counter; low 2 bits are sub-counter
	UINT8 status;				// current status register
	UINT8 clock_prescale;		// prescale factor (2/3/6)
	UINT8 irq_mask;				// mask of which bits signal IRQs
	UINT8 status_mask;			// status bits masked by the mode register (OPL only)
	UINT8 timer_running[2];		// current timer running state
	UINT8 total_clocks;			// low 8 bits of the total number of clocks processed
	UINT8 lfo_am;				// LFO AM value of the current sample
	UINT8 noise_state;			// noise state of the current sample (bit 0)
	INT32 timer_count[2];		// samples until the timer expires, 0 = stopped
	UINT32 all_channels;		// mask of all channels
	UINT32 active_channels;		// mask of active channels (computed by prepare)
	UINT32 modified_channels;	// mask of channels that have been modified
	UINT32 prepare_count;		// counter to do periodic prepare sweeps
	UINT32 mute_mask;			// muted channels, then the FM_MUTE_xxx rhythm bits
	FM_CHANNEL channel[FM_MAX_CHANNELS];
	FM_OPERATOR op[FM_MAX_OPERATORS];
} FM_ENGINE;


//----------------------------------------------------------------------------
// helpers
//----------------------------------------------------------------------------

// extract a bitfield from the given value, starting at bit 'start' for a
// length of 'length' bits
INLINE UINT32 bitfield(UINT32 value, int start, int length)
{
	return (value >> start) & ((1 << length) - 1);
}

// clamp between the minimum and maximum values provided
INLINE INT32 clamp(INT32 value, INT32 minval, INT32 maxval)
{
	if (value < minval)
		return minval;
	if (value > maxval)
		return maxval;
	return value;
}

// encode four operator numbers into a 32-bit value for the operator maps
#define fm_operator_list(o1, o2, o3, o4)	\
	((UINT32)(o1) | ((UINT32)(o2) << 8) | ((UINT32)(o3) << 16) | ((UINT32)(o4) << 24))

// apply KSR to the raw ADSR rate, ignoring ksr if the raw value is 0, and
// clamping to 63
INLINE UINT32 fm_effective_rate(UINT32 rawrate, UINT32 ksr)
{
	if (rawrate == 0)
		return 0;
	return (rawrate + ksr < 63) ? (rawrate + ksr) : 63;
}

// tables of ymfm_fm.c used by the helpers below
extern const UINT16 fm_power_table[256];
extern const UINT32 fm_increment_table[64];

// given a 5.8 fixed point logarithmic attenuation value, return a 13-bit
// linear volume
INLINE UINT32 attenuation_to_volume(UINT32 input)
{
	// look up the fractional part, then shift by the whole
	return fm_power_table[input & 0xff] >> (input >> 8);
}

// given a 6-bit ADSR rate value and a 3-bit stepping index, return a 4-bit
// increment to the attenuation for this step (or for the attack case, the
// fractional scale factor to decrease by)
INLINE UINT32 attenuation_increment(UINT32 rate, UINT32 index)
{
	return bitfield(fm_increment_table[rate], 4 * index, 4);
}

// return the absolute value of sin(input) as a 4.8 attenuation value, where
// the range 0-2*PI is mapped onto 10 bits
UINT32 fm_abs_sin_attenuation(UINT32 input);

// return the 6-bit signed phase displacement for a 3-bit detune value and a
// 5-bit key code (OPM, OPN)
INT32 fm_detune_adjustment(UINT32 detune, UINT32 keycode);

// return the signed PM adjustment to the frequency for the 7 most significant
// frequency number bits, a 3-bit PM depth and a signed 5-bit raw PM value (OPN)
INT32 fm_opn_lfo_pm_phase_adjustment(UINT32 fnum_bits, UINT32 pm_sensitivity, INT32 lfo_raw_pm);

// Many of the Yamaha FM chips emit a floating-point value, which is sent to
// a DAC for processing. The exact format of this floating-point value is
// documented below. This description only makes sense if the "internal"
// format treats sign as 1=positive and 0=negative, so the helpers below
// presume that.
//
// Internal OPx data      16-bit signed data     Exp Sign Mantissa
// =================      =================      === ==== ========
// 1 1xxxxxxxx------  ->  0 1xxxxxxxx------  ->  111   1  1xxxxxxx
// 1 01xxxxxxxx-----  ->  0 01xxxxxxxx-----  ->  110   1  1xxxxxxx
// 1 001xxxxxxxx----  ->  0 001xxxxxxxx----  ->  101   1  1xxxxxxx
// 1 0001xxxxxxxx---  ->  0 0001xxxxxxxx---  ->  100   1  1xxxxxxx
// 1 00001xxxxxxxx--  ->  0 00001xxxxxxxx--  ->  011   1  1xxxxxxx
// 1 000001xxxxxxxx-  ->  0 000001xxxxxxxx-  ->  010   1  1xxxxxxx
// 1 000000xxxxxxxxx  ->  0 000000xxxxxxxxx  ->  001   1  xxxxxxxx
// 0 111111xxxxxxxxx  ->  1 111111xxxxxxxxx  ->  001   0  xxxxxxxx
// 0 111110xxxxxxxx-  ->  1 111110xxxxxxxx-  ->  010   0  0xxxxxxx
// 0 11110xxxxxxxx--  ->  1 11110xxxxxxxx--  ->  011   0  0xxxxxxx
// 0 1110xxxxxxxx---  ->  1 1110xxxxxxxx---  ->  100   0  0xxxxxxx
// 0 110xxxxxxxx----  ->  1 110xxxxxxxx----  ->  101   0  0xxxxxxx
// 0 10xxxxxxxx-----  ->  1 10xxxxxxxx-----  ->  110   0  0xxxxxxx
// 0 0xxxxxxxx------  ->  1 0xxxxxxxx------  ->  111   0  0xxxxxxx

// compute the result of a round trip through the 3.10 floating-point format
// sent to the DAC: the value is clamped to 16 bits and the bits below the
// mantissa are zeroed
INLINE INT32 fm_roundtrip_fp(INT32 value)
{
	UINT32 bits;

	// handle overflows first
	if (value < -32768)
		return -32768;
	if (value > 32767)
		return 32767;

	// the exponent follows the number of leading sign bits after the sign
	// bit, counted from bit 14: invert negative values, and look at the
	// bits above the 8 mantissa bits. The bits below the mantissa are lost,
	// and their number is one less than the bit length of that part (none
	// if it is zero or one); smearing its top bit downwards gives a mask
	// of that bit length, and one bit less is the mask of the lost bits
	bits = (UINT32)(value ^ (value >> 31)) >> 8;
	bits |= bits >> 1;
	bits |= bits >> 2;
	bits |= bits >> 4;
	return value & ~(INT32)(bits >> 1);
}


//----------------------------------------------------------------------------
// engine
//----------------------------------------------------------------------------

// set up the engine for a family; call once, before fm_engine_reset()
void fm_engine_init(FM_ENGINE* fm, const FM_FAMILY* family, void* owner);

// reset the overall state
void fm_engine_reset(FM_ENGINE* fm);

// fm_engine_clock() and fm_engine_output() are in ymfm_fm_impl.h

// write to the FM registers
void fm_engine_write(FM_ENGINE* fm, UINT16 regnum, UINT8 data);

// return the current status
UINT8 fm_engine_status(const FM_ENGINE* fm);

// set/reset bits in the status register, updating the IRQ status
UINT8 fm_engine_set_reset_status(FM_ENGINE* fm, UINT8 set, UINT8 reset);

// set the IRQ mask
void fm_engine_set_irq_mask(FM_ENGINE* fm, UINT8 mask);

// invalidate any caches
#define fm_engine_invalidate_caches(fm)	((fm)->modified_channels = (fm)->all_channels)

// compute the sample rate for the given input clock
#define fm_engine_sample_rate(fm, baseclock)	\
	((baseclock) / ((UINT32)(fm)->clock_prescale * (fm)->family->operators))

#endif	// __YMFM_FM_H__
