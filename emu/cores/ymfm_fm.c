/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    Port notes:
    - ymfm builds one engine per register family with C++ templates. Here
      one engine serves all families: the core file passes an FM_FAMILY
      table with its constants and the functions for the parts that differ
      (register writes, operator mapping, data caches, phase step, LFO and
      noise). The per-sample part (clocking and output) is in
      ymfm_fm_impl.h, which each family file includes and calls with its
      constant table, so the compiler specializes it per family, as C++
      does with ymfm_fm.ipp; this file has the rare paths.
    - ymfm reads some registers on every sample (feedback, algorithm,
      output enables, AM sensitivity, AM enable, SSG-EG, OPL rhythm and OPM
      noise enables). Every register write invalidates the caches, so here
      they are read once at prepare time into the channel and operator
      caches. The result is the same, as writes only happen between
      samples. The LFO AM value and the noise state are latched by the
      engine after each LFO clock.
    - Only the envelope states of the ported chips are kept: no OPLL
      depress and no OPQ/OPZ reverb state, and no envelope shift (OPZ).
      SSG-EG is clocked only on families with eg_has_ssg (ymfm's
      EG_HAS_SSG); elsewhere its register value is 0, so the other SSG-EG
      checks have no effect.
    - ymfm leaves timers to its host. Here they count in chip samples (one
      period unit is one sample: OPERATORS * prescale clocks) and expire at
      the start of fm_engine_clock(), so Timer A also drives CSM key-on.
    - The busy flag and the IRQ line are not emulated; the IRQ status bit
      is. Save states and debug logging are not ported.
    - Muting: muted channels (mute_mask) are computed, so their feedback
      stays current, but not added to the outputs. In OPL rhythm mode,
      channels 6-8 are muted per instrument (FM_MUTE_xxx bits above the
      channel bits) instead.
    - Change from ymfm: ymfm samples the key state once per sample, so a
      key off followed by a key on before the next sample was lost and the
      note did not restart. Sound drivers commonly write them back to back.
      Here a key off is remembered until the next sample, so the operator
      releases and then attacks again, as on the real chip.

***************************************************************************/

#include <string.h>	// for memset

#include "ymfm_fm.h"
#include "ymfm_fm_impl.h"


//----------------------------------------------------------------------------
// tables
//----------------------------------------------------------------------------

// the values here are stored as 4.8 logarithmic values for 1/4 phase
// this matches the internal format of the OPN chip, extracted from the die
static const UINT16 s_sin_table[256] =
{
	0x859,0x6c3,0x607,0x58b,0x52e,0x4e4,0x4a6,0x471,0x443,0x41a,0x3f5,0x3d3,0x3b5,0x398,0x37e,0x365,
	0x34e,0x339,0x324,0x311,0x2ff,0x2ed,0x2dc,0x2cd,0x2bd,0x2af,0x2a0,0x293,0x286,0x279,0x26d,0x261,
	0x256,0x24b,0x240,0x236,0x22c,0x222,0x218,0x20f,0x206,0x1fd,0x1f5,0x1ec,0x1e4,0x1dc,0x1d4,0x1cd,
	0x1c5,0x1be,0x1b7,0x1b0,0x1a9,0x1a2,0x19b,0x195,0x18f,0x188,0x182,0x17c,0x177,0x171,0x16b,0x166,
	0x160,0x15b,0x155,0x150,0x14b,0x146,0x141,0x13c,0x137,0x133,0x12e,0x129,0x125,0x121,0x11c,0x118,
	0x114,0x10f,0x10b,0x107,0x103,0x0ff,0x0fb,0x0f8,0x0f4,0x0f0,0x0ec,0x0e9,0x0e5,0x0e2,0x0de,0x0db,
	0x0d7,0x0d4,0x0d1,0x0cd,0x0ca,0x0c7,0x0c4,0x0c1,0x0be,0x0bb,0x0b8,0x0b5,0x0b2,0x0af,0x0ac,0x0a9,
	0x0a7,0x0a4,0x0a1,0x09f,0x09c,0x099,0x097,0x094,0x092,0x08f,0x08d,0x08a,0x088,0x086,0x083,0x081,
	0x07f,0x07d,0x07a,0x078,0x076,0x074,0x072,0x070,0x06e,0x06c,0x06a,0x068,0x066,0x064,0x062,0x060,
	0x05e,0x05c,0x05b,0x059,0x057,0x055,0x053,0x052,0x050,0x04e,0x04d,0x04b,0x04a,0x048,0x046,0x045,
	0x043,0x042,0x040,0x03f,0x03e,0x03c,0x03b,0x039,0x038,0x037,0x035,0x034,0x033,0x031,0x030,0x02f,
	0x02e,0x02d,0x02b,0x02a,0x029,0x028,0x027,0x026,0x025,0x024,0x023,0x022,0x021,0x020,0x01f,0x01e,
	0x01d,0x01c,0x01b,0x01a,0x019,0x018,0x017,0x017,0x016,0x015,0x014,0x014,0x013,0x012,0x011,0x011,
	0x010,0x00f,0x00f,0x00e,0x00d,0x00d,0x00c,0x00c,0x00b,0x00a,0x00a,0x009,0x009,0x008,0x008,0x007,
	0x007,0x007,0x006,0x006,0x005,0x005,0x005,0x004,0x004,0x004,0x003,0x003,0x003,0x002,0x002,0x002,
	0x002,0x001,0x001,0x001,0x001,0x001,0x001,0x001,0x000,0x000,0x000,0x000,0x000,0x000,0x000,0x000
};

// the values here are 10-bit mantissas with an implied leading bit
// this matches the internal format of the OPN chip, extracted from the die

// as a nod to performance, the implicit 0x400 bit is pre-incorporated, and
// the values are left-shifted by 2 so that a simple right shift is all that
// is needed; also the order is reversed to save a NOT on the input
#define X(a) (((a) | 0x400) << 2)
const UINT16 fm_power_table[256] =
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
const UINT32 fm_increment_table[64] =
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

// this table has been verified against Nuked's equations, but the equations
// are rather complicated, so we'll keep the simplicity of the table
static const UINT8 s_detune_adjustment[32][4] =
{
	{ 0,  0,  1,  2 },  { 0,  0,  1,  2 },  { 0,  0,  1,  2 },  { 0,  0,  1,  2 },
	{ 0,  1,  2,  2 },  { 0,  1,  2,  3 },  { 0,  1,  2,  3 },  { 0,  1,  2,  3 },
	{ 0,  1,  2,  4 },  { 0,  1,  3,  4 },  { 0,  1,  3,  4 },  { 0,  1,  3,  5 },
	{ 0,  2,  4,  5 },  { 0,  2,  4,  6 },  { 0,  2,  4,  6 },  { 0,  2,  5,  7 },
	{ 0,  2,  5,  8 },  { 0,  3,  6,  8 },  { 0,  3,  6,  9 },  { 0,  3,  7, 10 },
	{ 0,  4,  8, 11 },  { 0,  4,  8, 12 },  { 0,  4,  9, 13 },  { 0,  5, 10, 14 },
	{ 0,  5, 11, 16 },  { 0,  6, 12, 17 },  { 0,  6, 13, 19 },  { 0,  7, 14, 20 },
	{ 0,  8, 16, 22 },  { 0,  8, 16, 22 },  { 0,  8, 16, 22 },  { 0,  8, 16, 22 }
};

// this table encodes 2 shift values to apply to the top 7 bits of fnum;
// it is effectively a cheap multiply by a constant value containing 0-2 bits
static const UINT8 s_lfo_pm_shifts[8][8] =
{
	{ 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 },
	{ 0x77, 0x77, 0x77, 0x77, 0x72, 0x72, 0x72, 0x72 },
	{ 0x77, 0x77, 0x77, 0x72, 0x72, 0x72, 0x17, 0x17 },
	{ 0x77, 0x77, 0x72, 0x72, 0x17, 0x17, 0x12, 0x12 },
	{ 0x77, 0x77, 0x72, 0x17, 0x17, 0x17, 0x12, 0x07 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 },
	{ 0x77, 0x77, 0x17, 0x12, 0x07, 0x07, 0x02, 0x01 }
};

// OPM/OPN offer 8 different connection algorithms for 4 operators,
// and OPL3 offers 4 more, which we designate here as 8-11.
//
// The operators are computed in order, with the inputs pulled from
// an array of values (opout) that is populated as we go:
//    0 = 0
//    1 = O1
//    2 = O2
//    3 = O3
//    4 = (O4)
//    5 = O1+O2
//    6 = O1+O3
//    7 = O2+O3
//
// The fm_algorithm_ops table describes the inputs and outputs of each
// algorithm as follows:
//
//      ---------x use opout[x] as operator 2 input
//      ------xxx- use opout[x] as operator 3 input
//      ---xxx---- use opout[x] as operator 4 input
//      --x------- include opout[1] in final sum
//      -x-------- include opout[2] in final sum
//      x--------- include opout[3] in final sum
#define ALGORITHM(op2in, op3in, op4in, op1out, op2out, op3out) \
	((op2in) | ((op3in) << 1) | ((op4in) << 4) | ((op1out) << 7) | ((op2out) << 8) | ((op3out) << 9))
const UINT16 fm_algorithm_ops[8+4] =
{
	ALGORITHM(1,2,3, 0,0,0),    //  0: O1 -> O2 -> O3 -> O4 -> out (O4)
	ALGORITHM(0,5,3, 0,0,0),    //  1: (O1 + O2) -> O3 -> O4 -> out (O4)
	ALGORITHM(0,2,6, 0,0,0),    //  2: (O1 + (O2 -> O3)) -> O4 -> out (O4)
	ALGORITHM(1,0,7, 0,0,0),    //  3: ((O1 -> O2) + O3) -> O4 -> out (O4)
	ALGORITHM(1,0,3, 0,1,0),    //  4: ((O1 -> O2) + (O3 -> O4)) -> out (O2+O4)
	ALGORITHM(1,1,1, 0,1,1),    //  5: ((O1 -> O2) + (O1 -> O3) + (O1 -> O4)) -> out (O2+O3+O4)
	ALGORITHM(1,0,0, 0,1,1),    //  6: ((O1 -> O2) + O3 + O4) -> out (O2+O3+O4)
	ALGORITHM(0,0,0, 1,1,1),    //  7: (O1 + O2 + O3 + O4) -> out (O1+O2+O3+O4)
	ALGORITHM(1,2,3, 0,0,0),    //  8: O1 -> O2 -> O3 -> O4 -> out (O4)         [same as 0]
	ALGORITHM(0,2,3, 1,0,0),    //  9: (O1 + (O2 -> O3 -> O4)) -> out (O1+O4)   [unique]
	ALGORITHM(1,0,3, 0,1,0),    // 10: ((O1 -> O2) + (O3 -> O4)) -> out (O2+O4) [same as 4]
	ALGORITHM(0,2,0, 1,0,1)     // 11: (O1 + (O2 -> O3) + O4) -> out (O1+O3+O4) [unique]
};
#undef ALGORITHM


//----------------------------------------------------------------------------
// table lookups
//----------------------------------------------------------------------------

UINT32 fm_abs_sin_attenuation(UINT32 input)
{
	// if the top bit is set, we're in the second half of the curve
	// which is a mirror image, so invert the index
	if (bitfield(input, 8, 1))
		input = ~input;

	// return the value from the table
	return s_sin_table[input & 0xff];
}

INT32 fm_detune_adjustment(UINT32 detune, UINT32 keycode)
{
	INT32 result = s_detune_adjustment[keycode][detune & 3];
	return bitfield(detune, 2, 1) ? -result : result;
}

// algorithm written to match Nuked behavior
INT32 fm_opn_lfo_pm_phase_adjustment(UINT32 fnum_bits, UINT32 pm_sensitivity, INT32 lfo_raw_pm)
{
	INT32 abs_pm;
	UINT32 shifts;
	INT32 adjust;

	// look up the relevant shifts
	abs_pm = (lfo_raw_pm < 0) ? -lfo_raw_pm : lfo_raw_pm;
	shifts = s_lfo_pm_shifts[pm_sensitivity][bitfield(abs_pm, 0, 3)];

	// compute the adjustment
	adjust = (INT32)((fnum_bits >> bitfield(shifts, 0, 4)) + (fnum_bits >> bitfield(shifts, 4, 4)));
	if (pm_sensitivity > 5)
		adjust <<= pm_sensitivity - 5;
	adjust >>= 2;

	// every 16 cycles it inverts sign
	return (lfo_raw_pm < 0) ? -adjust : adjust;
}


//----------------------------------------------------------------------------
// operator
//----------------------------------------------------------------------------

static void op_reset(FM_OPERATOR* op)
{
	op->phase = 0;
	op->env_attenuation = 0x3ff;
	op->env_state = EG_RELEASE;
	op->ssg_inverted = 0;
	op->key_state = 0;
	op->keyon_live = 0;
	op->keyoff_pending = 0;
	return;
}

// signal a key on/off event from one source; a key off is remembered until
// the next prepare, so that a key off + key on within one sample restarts
static void op_keyonoff(FM_OPERATOR* op, UINT32 on, UINT8 type)
{
	UINT8 old_live = op->keyon_live;

	op->keyon_live = (UINT8)((op->keyon_live & ~(1 << type)) | (bitfield(on, 0, 1) << type));
	if (old_live != 0 && op->keyon_live == 0)
		op->keyoff_pending = 1;
	return;
}

// start the release phase; called when a keyoff happens
static void op_start_release(FM_OPERATOR* op)
{
	// don't change anything if already in release state
	if (op->env_state >= EG_RELEASE)
		return;
	op->env_state = EG_RELEASE;

	// if attenuation if inverted due to SSG-EG, snap the inverted attenuation
	// as the starting point
	if (op->ssg_inverted)
	{
		op->env_attenuation = (0x200 - op->env_attenuation) & 0x3ff;
		op->ssg_inverted = 0;
	}
	return;
}

// clock the keystate to match the incoming keystate
static void op_clock_keystate(FM_OPERATOR* op, UINT32 keystate)
{
	// has the key changed?
	if ((keystate ^ op->key_state) != 0)
	{
		op->key_state = (UINT8)keystate;

		// if the key has turned on, start the attack
		if (keystate != 0)
			op_start_attack(op, 0);
		// otherwise, start the release
		else
			op_start_release(op);
	}
	return;
}

// prepare prior to clocking; returns non-zero while the operator is audible
static UINT8 op_prepare(FM_ENGINE* fm, FM_OPERATOR* op)
{
	// cache the data
	fm->family->cache_operator_data(fm->owner, op->choffs, op->opoffs, &op->cache);

	// clock the key state; a key off since the last prepare releases first
	if (op->keyoff_pending)
	{
		op_clock_keystate(op, 0);
		op->keyoff_pending = 0;
	}
	op_clock_keystate(op, (UINT32)(op->keyon_live != 0));
	op->keyon_live &= ~(1 << KEYON_CSM);

	// we're active until we're quiet after the release
	return (op->env_state != EG_RELEASE || op->env_attenuation < EG_QUIET);
}



//----------------------------------------------------------------------------
// channel
//----------------------------------------------------------------------------

// signal key on/off to our operators
static void channel_keyonoff(FM_CHANNEL* ch, UINT32 states, UINT8 type)
{
	UINT8 opnum;

	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (ch->op[opnum] != NULL)
			op_keyonoff(ch->op[opnum], bitfield(states, opnum, 1), type);
	}
	return;
}

// prepare prior to clocking; returns non-zero if any operator is active
static UINT8 channel_prepare(FM_ENGINE* fm, FM_CHANNEL* ch)
{
	UINT32 active_mask = 0;
	UINT8 opnum;

	fm->family->cache_channel_data(fm->owner, ch->choffs, &ch->cache);

	// prepare all operators and determine if they are active
	for (opnum = 0; opnum < 4; opnum ++)
	{
		if (ch->op[opnum] != NULL && op_prepare(fm, ch->op[opnum]))
			active_mask |= 1 << opnum;
	}
	return (active_mask != 0);
}



//----------------------------------------------------------------------------
// engine
//----------------------------------------------------------------------------

// update the IRQ status bit
static void engine_check_interrupts(FM_ENGINE* fm)
{
	if ((fm->status & fm->irq_mask & ~fm->status_mask) != 0)
		fm->status |= fm->family->status_irq;
	else
		fm->status &= ~fm->family->status_irq;
	return;
}

UINT8 fm_engine_set_reset_status(FM_ENGINE* fm, UINT8 set, UINT8 reset)
{
	fm->status = (fm->status | set) & ~(reset | fm->family->status_busy);
	engine_check_interrupts(fm);
	return fm->status & ~fm->status_mask;
}

void fm_engine_set_irq_mask(FM_ENGINE* fm, UINT8 mask)
{
	fm->irq_mask = mask;
	engine_check_interrupts(fm);
	return;
}

UINT8 fm_engine_status(const FM_ENGINE* fm)
{
	return fm->status & ~fm->family->status_busy & ~fm->status_mask;
}

// update the state of the given timer
static void engine_update_timer(FM_ENGINE* fm, const FM_MODE* mode, UINT32 tnum, UINT32 enable, INT32 delta_clocks)
{
	UINT32 period;

	// if the timer is live, but not currently enabled, set the timer
	if (enable && ! fm->timer_running[tnum])
	{
		// period comes from the registers, and is different for each
		period = (tnum == 0) ? (1024 - mode->timer_a_value) : 16 * (256 - mode->timer_b_value);

		// caller can also specify a delta to account for other effects
		period += delta_clocks;

		// reset it; one period unit is one sample (OPERATORS * prescale clocks)
		fm->timer_count[tnum] = (INT32)period;
		fm->timer_running[tnum] = 1;
	}
	// if the timer is not live, ensure it is not enabled
	else if (! enable)
	{
		fm->timer_count[tnum] = 0;
		fm->timer_running[tnum] = 0;
	}
	return;
}

// timer has expired - signal status and possibly CSM key on
void fm_engine_timer_expired(FM_ENGINE* fm, UINT32 tnum)
{
	const FM_FAMILY* family = fm->family;
	FM_MODE mode;
	UINT32 chnum;

	family->read_mode(fm->owner, &mode);

	// update status
	if (tnum == 0 && mode.enable_timer_a)
		fm_engine_set_reset_status(fm, family->status_timera, 0);
	else if (tnum == 1 && mode.enable_timer_b)
		fm_engine_set_reset_status(fm, family->status_timerb, 0);

	// if timer A fired in CSM mode, trigger CSM on all relevant channels
	if (tnum == 0 && mode.csm)
	{
		for (chnum = 0; chnum < family->channels; chnum ++)
		{
			if (bitfield(family->csm_trigger_mask, chnum, 1))
			{
				channel_keyonoff(&fm->channel[chnum], 0xf, KEYON_CSM);
				fm->modified_channels |= 1 << chnum;
			}
		}
	}

	// reset
	fm->timer_running[tnum] = 0;
	engine_update_timer(fm, &mode, tnum, 1, 0);
	return;
}

// handle a mode register write
static void engine_mode_write(FM_ENGINE* fm, UINT8 data)
{
	const FM_FAMILY* family = fm->family;
	FM_MODE mode;
	UINT32 dummy1;
	UINT32 dummy2;
	UINT8 reset_mask;

	// mark all channels as modified
	fm->modified_channels = fm->all_channels;

	// actually write the mode register now
	family->write(fm->owner, family->reg_mode, data, &dummy1, &dummy2);
	family->read_mode(fm->owner, &mode);
	fm->status_mask = mode.status_mask;

	// reset IRQ status -- when written, all other bits are ignored
	// QUESTION: should this maybe just reset the IRQ bit and not all the bits?
	//   That is, check_interrupts would only set, this would only clear?
	if (mode.irq_reset)
	{
		fm_engine_set_reset_status(fm, 0, 0x78);
	}
	else
	{
		// reset timer status
		reset_mask = 0;
		if (mode.reset_timer_b)
			reset_mask |= family->status_timerb;
		if (mode.reset_timer_a)
			reset_mask |= family->status_timera;
		fm_engine_set_reset_status(fm, 0, reset_mask);

		// load timers; note that timer B gets a small negative adjustment because
		// the *16 multiplier is free-running, so the first tick of the clock
		// is a bit shorter
		engine_update_timer(fm, &mode, 1, mode.load_timer_b, -(INT32)(fm->total_clocks & 15));
		engine_update_timer(fm, &mode, 0, mode.load_timer_a, 0);
	}
	return;
}

void fm_engine_write(FM_ENGINE* fm, UINT16 regnum, UINT8 data)
{
	const FM_FAMILY* family = fm->family;
	UINT32 keyon_channel;
	UINT32 keyon_opmask;

	// special case: writes to the mode register can impact IRQs
	if (regnum == family->reg_mode)
	{
		engine_mode_write(fm, data);
		return;
	}

	// for now just mark all channels as modified
	fm->modified_channels = fm->all_channels;

	// most writes are passive, consumed only when needed
	if (family->write(fm->owner, regnum, data, &keyon_channel, &keyon_opmask))
	{
		// handle writes to the keyon register(s)
		if (keyon_channel < family->channels)
		{
			// normal channel on/off
			channel_keyonoff(&fm->channel[keyon_channel], keyon_opmask, KEYON_NORMAL);
		}
		else if (family->channels >= 9 && keyon_channel == FM_RHYTHM_CHANNEL)
		{
			// special case for the OPL rhythm channels
			channel_keyonoff(&fm->channel[6], bitfield(keyon_opmask, 4, 1) ? 3 : 0, KEYON_RHYTHM);
			channel_keyonoff(&fm->channel[7], bitfield(keyon_opmask, 0, 1) | (bitfield(keyon_opmask, 3, 1) << 1), KEYON_RHYTHM);
			channel_keyonoff(&fm->channel[8], bitfield(keyon_opmask, 2, 1) | (bitfield(keyon_opmask, 1, 1) << 1), KEYON_RHYTHM);
		}
	}
	return;
}

// get the current mapping of operators to channels and assign them all
static void engine_assign_operators(FM_ENGINE* fm)
{
	UINT32 map[FM_MAX_CHANNELS];
	UINT32 chnum;
	UINT32 index;
	UINT32 opnum;
	FM_CHANNEL* ch;

	fm->family->operator_map(fm->owner, map);

	for (chnum = 0; chnum < fm->family->channels; chnum ++)
	{
		ch = &fm->channel[chnum];
		for (index = 0; index < 4; index ++)
		{
			opnum = bitfield(map[chnum], 8 * index, 8);
			if (opnum == 0xff)
			{
				ch->op[index] = NULL;
			}
			else
			{
				ch->op[index] = &fm->op[opnum];
				ch->op[index]->choffs = ch->choffs;
			}
		}
	}
	return;
}

void fm_engine_prepare(FM_ENGINE* fm, UINT32 chanmask)
{
	const FM_FAMILY* family = fm->family;
	UINT32 chnum;

	// reassign operators to channels if dynamic
	if (family->dynamic_ops)
		engine_assign_operators(fm);

	// call each channel to prepare
	fm->active_channels = 0;
	for (chnum = 0; chnum < family->channels; chnum ++)
	{
		if (bitfield(chanmask, chnum, 1) && channel_prepare(fm, &fm->channel[chnum]))
			fm->active_channels |= 1 << chnum;
	}

	// reset the modified channels and prepare count
	fm->modified_channels = fm->prepare_count = 0;
	return;
}

void fm_engine_reset(FM_ENGINE* fm)
{
	UINT32 chnum;
	UINT32 opnum;

	// reset all status bits
	fm_engine_set_reset_status(fm, 0, 0xff);

	// register type-specific initialization
	fm->family->reset(fm->owner);

	// explicitly write to the mode register since it has side-effects
	// QUESTION: old cores initialize this to 0x30 -- who is right?
	fm_engine_write(fm, fm->family->reg_mode, 0);

	// reset the channels
	for (chnum = 0; chnum < fm->family->channels; chnum ++)
	{
		fm->channel[chnum].feedback[0] = fm->channel[chnum].feedback[1] = 0;
		fm->channel[chnum].feedback_in = 0;
	}

	// reset the operators
	for (opnum = 0; opnum < fm->family->operators; opnum ++)
		op_reset(&fm->op[opnum]);
	return;
}

void fm_engine_init(FM_ENGINE* fm, const FM_FAMILY* family, void* owner)
{
	UINT32 chnum;
	UINT32 opnum;

	memset(fm, 0, sizeof(FM_ENGINE));
	fm->family = family;
	fm->owner = owner;
	fm->clock_prescale = family->default_prescale;
	fm->irq_mask = family->status_timera | family->status_timerb;
	fm->all_channels = (1 << family->channels) - 1;
	fm->active_channels = fm->all_channels;
	fm->modified_channels = fm->all_channels;

	// create the channels
	for (chnum = 0; chnum < family->channels; chnum ++)
		fm->channel[chnum].choffs = family->channel_offset(chnum);

	// create the operators
	for (opnum = 0; opnum < family->operators; opnum ++)
	{
		fm->op[opnum].opoffs = family->operator_offset(opnum);
		op_reset(&fm->op[opnum]);
	}

	// do the initial operator assignment
	engine_assign_operators(fm);
	return;
}
