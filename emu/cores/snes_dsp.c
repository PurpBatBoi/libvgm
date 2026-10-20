// MIT License

// Copyright (c) 2026 Eric Tomasso
// Copyright (c) 2026 Purpbatboi (C port)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// S-DSP (SNES sound chip)
//
// C port of Snaggletooth's S-DSP (https://github.com/etroimcasso/Snaggletooth,
// pinned at commit efd5049):
//   include/snaggletooth/apu/dsp.h
//   src/dsp.cpp
//   src/generated/gauss_table.inc and src/generated/envelope_counter_tables.inc
//     (the two tables below, copied value for value)
//
// The chip runs one of a sample's 32 clock slots per call of snes_dsp_step_cycle(),
// exactly as the C++ stepDspCycle() does: BRR decode, Gaussian interpolation,
// ADSR/GAIN envelopes, pitch modulation, noise, the echo unit with its FIR filter
// and the mute gate. The register reads and writes land on the slots the C++ puts
// them on, so a mid-sample register write is seen only if it precedes its consuming
// slot.
//
// Port notes:
// - DspState and its VoiceState / GroupDecode are plain structs (SNES_DSP,
//   SNES_DSP_VOICE, SNES_DSP_GROUP_DECODE); std::array members are fixed arrays,
//   bool members are UINT8 holding 0 or 1, std::size_t voice indices are
//   unsigned int, and `int` members are INT32. The cycle stamps stay 64 bit
//   (UINT64), so a stamp never wraps within any play time, as in the C++.
// - The C++ anonymous-namespace helpers are static functions with the same names
//   in snake case; the tables and constants are #defines and static const arrays.
// - The slot schedule (kSlotSchedule) is a static const table. The C++ builds it at
//   compile time by walking the ten per-voice slot tables
//   (kVoiceSrcnSlot / kVoiceDirSlot / kVoicePitchHighSlot / kVoiceBrrLoadSlot /
//   kVoiceS3Slot / kVoiceS4Slot ... kVoiceS9Slot) in a fixed order; the table here is
//   that result, written out (see the comment above it for the slot values).
// - The RAM is one writable pointer. The C++ passes the same memory twice (a const
//   span the DSP reads and a pointer the echo unit writes) and has a read-only
//   overload for callers that must not write; only the writable path is ported.
// - The state is one frame ahead of the output exactly as in the C++: a freshly
//   seeded state (power-on, or an .spc image) runs its first sample at once
//   (stepDspSampleAtomic) and primes the slot pipeline at its close.
// - Not ported (no supported caller uses them):
//   - decodeBrrBlock() / BrrBlock, stepVoice(), startVoice() as a public call,
//     keyOnVoice(), stepDspSample() (both overloads), the read-only stepDspCycle()
//     overload, nextGlobalCounter() and tickDspSample() as public calls: the
//     machine reaches the DSP through stepDspCycle() and cpuWriteDspRegister() only.
//     The internals they share with the slot path are all ported.
//   - DspState's byte form (asBytes / fromBytes) and its register iteration helpers.
// - Deliberate changes from the C++:
//   - Per-voice mute mask (SNES_DSP.mute_mask, libvgm's RWF_CHN_MUTE convention, bit n
//     = voice n). A muted voice still runs (envelope, OUTX/ENVX, the pitch
//     modulation source and the noise/echo timing are unchanged), but what it adds to
//     the main mix and to the echo send is 0, which is exactly what a VxVOL of 0 does.
//     Mask 0 is identical to the C++.

#include <stddef.h>
#include <string.h>

#include "../../stdtype.h"
#include "../../common_def.h"
#include "snes_dsp.h"


// DSP register addresses the voice stream reads and writes. Per-voice
// registers live at voice*10h plus the per-voice offset.
#define DSP_MVOL_LEFT	0x0C
#define DSP_MVOL_RIGHT	0x1C
#define DSP_EFB			0x0D
#define DSP_EVOL_LEFT	0x2C
#define DSP_EVOL_RIGHT	0x3C
#define DSP_PMON		0x2D
#define DSP_NON			0x3D
#define DSP_EON			0x4D
#define DSP_KON			0x4C
#define DSP_KOFF		0x5C
#define DSP_DIR			0x5D
#define DSP_FLG			0x6C
#define DSP_ESA			0x6D
#define DSP_ENDX		0x7C
#define DSP_EDL			0x7D

// FLG bit masks: bits 0-4 are the noise rate, bit 5 disables echo writes, bit 6
// mutes the output amplifier, bit 7 is the per-sample soft reset.
#define FLG_NOISE_RATE			0x1F
#define FLG_ECHO_WRITE_DISABLE	0x20
#define FLG_MUTE				0x40
#define FLG_SOFT_RESET			0x80

// The per-voice offset of the echo FIR coefficient registers: FIR0..FIR7 live at
// $0F, $1F, ... $7F (tap*10h + this), each a signed 8-bit coefficient.
#define FIR_COEFF	0x0F

// The pitch counter's in-group position ceiling. The low fourteen bits of the
// counter place the interpolation cursor within the four-sample group it is
// consuming (bits 12-13 the sample, bits 0-11 the fraction); a step is added to
// that position and the sum clamps here before the crossed positions are
// counted, so one output sample consumes at most four source samples (128 kHz)
// and a position that hit the ceiling stands at 3FFFh (sample 3 of its group
// at the maximum fraction) once the group turns. The base 14-bit step tops out
// at 3FFFh, so only a PMON-scaled step reaches the ceiling.
#define MAX_GROUP_POSITION	0x7FFFu
#define GROUP_POSITION_MASK	0x3FFFu
#define VOICE_VOL_LEFT		0x00
#define VOICE_VOL_RIGHT		0x01
#define VOICE_PITCH_LOW		0x02
#define VOICE_PITCH_HIGH	0x03
#define VOICE_SRCN			0x04
#define VOICE_ADSR1			0x05
#define VOICE_ADSR2			0x06
#define VOICE_GAIN			0x07
#define VOICE_ENVX			0x08
#define VOICE_OUTX			0x09

// The global counter's power-on / wrap value: it counts down from 0x77FF.
#define GLOBAL_COUNTER_RELOAD	0x77FF

// The startup countdown a key-on arms: the documented five empty samples. The
// keying poll precedes voice 0's compute in the slot they share, so the load's
// own slot is the keyed voice's first silent startup call for voice 0, while
// voices 1-7 take theirs in the following samples.
#define KEY_ON_STARTUP_CALLS	5

// The voice's silent key-on span in compute calls: the five startup calls plus
// the first live compute, whose output is still silent because a sample is
// scaled by the envelope standing before its update. The keying poll absorbs a
// key-on landing inside this span (see poll_keying).
#define KEY_ON_SILENT_CALLS		(KEY_ON_STARTUP_CALLS + 1)

// The decoder runs a full block-priming ahead of the interpolation cursor:
// a key-on decodes the whole first block before the voice sounds, and each
// group the cursor consumes afterward has the decoder filling the buffer one
// group further on. So the decoder enters a block while the cursor is still
// eight samples back in the block before: this is the cursor's in-block
// consumed-sample count at which decoder_address moves on (`Misc/brr early
// end at many pitches` pins the eight; `KON/kon as prev sample ends` bounds
// it from the other side).
#define DECODER_LEAD	8

// The in-group sample index whose consumption schedules the decode of the
// group two on: the third sample, so the group's first data byte lands at the
// load slot two samples after the group boundary's own consumption and its
// second at that sample's S4 (where `Timing/Voice/V3 BRR.sample.lsb/msb`
// measure the hardware reading them) while the header is read at the
// scheduling itself, the sample `Order/pitch after brr` pins.
#define DECODE_TRIGGER_INDEX	2

// The envelope/noise rate tables, parser-generated in the C++ (tools/parse_dsp_tables.py),
// cross-checked against the references. envelope_period[0] == 0 is the
// 'never' sentinel (rate 0 = Infinite); envelope_period[31] == 1 fires every
// sample. envelope_offset delays each rate's phase within its period.
static const UINT16 envelope_period[32] = {
	0, 2048, 1536, 1280, 1024, 768, 640, 512,
	384, 320, 256, 192, 160, 128, 96, 80,
	64, 48, 40, 32, 24, 20, 16, 12,
	10, 8, 6, 5, 4, 3, 2, 1
};

static const UINT16 envelope_offset[32] = {
	0, 0, 1040, 536, 0, 1040, 536, 0,
	1040, 536, 0, 1040, 536, 0, 1040, 536,
	0, 1040, 536, 0, 1040, 536, 0, 1040,
	536, 0, 1040, 536, 0, 1040, 0, 0
};

// The S-DSP's Gaussian interpolation table: hardware ROM data, transcribed in the
// C++ from the references by tools/parse_dsp_tables.py. The four-tap sums land in
// 7FFh..801h rather than a constant 800h; the 801h rows are what make the kernel's
// documented wrap reachable.
static const INT16 gauss_table[512] = {
	0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
	0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, 0x002, 0x002, 0x002, 0x002,
	0x002, 0x002, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005,
	0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x008, 0x008, 0x008, 0x009, 0x009, 0x009, 0x00A, 0x00A, 0x00A,
	0x00B, 0x00B, 0x00B, 0x00C, 0x00C, 0x00D, 0x00D, 0x00E, 0x00E, 0x00F, 0x00F, 0x00F, 0x010, 0x010, 0x011, 0x011,
	0x012, 0x013, 0x013, 0x014, 0x014, 0x015, 0x015, 0x016, 0x017, 0x017, 0x018, 0x018, 0x019, 0x01A, 0x01B, 0x01B,
	0x01C, 0x01D, 0x01D, 0x01E, 0x01F, 0x020, 0x020, 0x021, 0x022, 0x023, 0x024, 0x024, 0x025, 0x026, 0x027, 0x028,
	0x029, 0x02A, 0x02B, 0x02C, 0x02D, 0x02E, 0x02F, 0x030, 0x031, 0x032, 0x033, 0x034, 0x035, 0x036, 0x037, 0x038,
	0x03A, 0x03B, 0x03C, 0x03D, 0x03E, 0x040, 0x041, 0x042, 0x043, 0x045, 0x046, 0x047, 0x049, 0x04A, 0x04C, 0x04D,
	0x04E, 0x050, 0x051, 0x053, 0x054, 0x056, 0x057, 0x059, 0x05A, 0x05C, 0x05E, 0x05F, 0x061, 0x063, 0x064, 0x066,
	0x068, 0x06A, 0x06B, 0x06D, 0x06F, 0x071, 0x073, 0x075, 0x076, 0x078, 0x07A, 0x07C, 0x07E, 0x080, 0x082, 0x084,
	0x086, 0x089, 0x08B, 0x08D, 0x08F, 0x091, 0x093, 0x096, 0x098, 0x09A, 0x09C, 0x09F, 0x0A1, 0x0A3, 0x0A6, 0x0A8,
	0x0AB, 0x0AD, 0x0AF, 0x0B2, 0x0B4, 0x0B7, 0x0BA, 0x0BC, 0x0BF, 0x0C1, 0x0C4, 0x0C7, 0x0C9, 0x0CC, 0x0CF, 0x0D2,
	0x0D4, 0x0D7, 0x0DA, 0x0DD, 0x0E0, 0x0E3, 0x0E6, 0x0E9, 0x0EC, 0x0EF, 0x0F2, 0x0F5, 0x0F8, 0x0FB, 0x0FE, 0x101,
	0x104, 0x107, 0x10B, 0x10E, 0x111, 0x114, 0x118, 0x11B, 0x11E, 0x122, 0x125, 0x129, 0x12C, 0x130, 0x133, 0x137,
	0x13A, 0x13E, 0x141, 0x145, 0x148, 0x14C, 0x150, 0x153, 0x157, 0x15B, 0x15F, 0x162, 0x166, 0x16A, 0x16E, 0x172,
	0x176, 0x17A, 0x17D, 0x181, 0x185, 0x189, 0x18D, 0x191, 0x195, 0x19A, 0x19E, 0x1A2, 0x1A6, 0x1AA, 0x1AE, 0x1B2,
	0x1B7, 0x1BB, 0x1BF, 0x1C3, 0x1C8, 0x1CC, 0x1D0, 0x1D5, 0x1D9, 0x1DD, 0x1E2, 0x1E6, 0x1EB, 0x1EF, 0x1F3, 0x1F8,
	0x1FC, 0x201, 0x205, 0x20A, 0x20F, 0x213, 0x218, 0x21C, 0x221, 0x226, 0x22A, 0x22F, 0x233, 0x238, 0x23D, 0x241,
	0x246, 0x24B, 0x250, 0x254, 0x259, 0x25E, 0x263, 0x267, 0x26C, 0x271, 0x276, 0x27B, 0x280, 0x284, 0x289, 0x28E,
	0x293, 0x298, 0x29D, 0x2A2, 0x2A6, 0x2AB, 0x2B0, 0x2B5, 0x2BA, 0x2BF, 0x2C4, 0x2C9, 0x2CE, 0x2D3, 0x2D8, 0x2DC,
	0x2E1, 0x2E6, 0x2EB, 0x2F0, 0x2F5, 0x2FA, 0x2FF, 0x304, 0x309, 0x30E, 0x313, 0x318, 0x31D, 0x322, 0x326, 0x32B,
	0x330, 0x335, 0x33A, 0x33F, 0x344, 0x349, 0x34E, 0x353, 0x357, 0x35C, 0x361, 0x366, 0x36B, 0x370, 0x374, 0x379,
	0x37E, 0x383, 0x388, 0x38C, 0x391, 0x396, 0x39B, 0x39F, 0x3A4, 0x3A9, 0x3AD, 0x3B2, 0x3B7, 0x3BB, 0x3C0, 0x3C5,
	0x3C9, 0x3CE, 0x3D2, 0x3D7, 0x3DC, 0x3E0, 0x3E5, 0x3E9, 0x3ED, 0x3F2, 0x3F6, 0x3FB, 0x3FF, 0x403, 0x408, 0x40C,
	0x410, 0x415, 0x419, 0x41D, 0x421, 0x425, 0x42A, 0x42E, 0x432, 0x436, 0x43A, 0x43E, 0x442, 0x446, 0x44A, 0x44E,
	0x452, 0x455, 0x459, 0x45D, 0x461, 0x465, 0x468, 0x46C, 0x470, 0x473, 0x477, 0x47A, 0x47E, 0x481, 0x485, 0x488,
	0x48C, 0x48F, 0x492, 0x496, 0x499, 0x49C, 0x49F, 0x4A2, 0x4A6, 0x4A9, 0x4AC, 0x4AF, 0x4B2, 0x4B5, 0x4B7, 0x4BA,
	0x4BD, 0x4C0, 0x4C3, 0x4C5, 0x4C8, 0x4CB, 0x4CD, 0x4D0, 0x4D2, 0x4D5, 0x4D7, 0x4D9, 0x4DC, 0x4DE, 0x4E0, 0x4E3,
	0x4E5, 0x4E7, 0x4E9, 0x4EB, 0x4ED, 0x4EF, 0x4F1, 0x4F3, 0x4F5, 0x4F6, 0x4F8, 0x4FA, 0x4FB, 0x4FD, 0x4FF, 0x500,
	0x502, 0x503, 0x504, 0x506, 0x507, 0x508, 0x50A, 0x50B, 0x50C, 0x50D, 0x50E, 0x50F, 0x510, 0x511, 0x511, 0x512,
	0x513, 0x514, 0x514, 0x515, 0x516, 0x516, 0x517, 0x517, 0x517, 0x518, 0x518, 0x518, 0x518, 0x518, 0x519, 0x519
};


INLINE UINT8 voice_register(unsigned int voice, UINT8 offset)
{
	return (UINT8)(voice * 0x10 + offset);
}

// Converts a signed 4-bit BRR nibble (-8..+7) to a sample: sample = nibble
// scaled by the block's shift, then arithmetic-shifted right one. A shift of
// 13..15 is anomalous: decoding proceeds as if shift were 12 over (nibble SAR
// 3). The scale is a multiply (never a left shift of a negative value) so the
// arithmetic is well-defined and warning-clean across compilers.
static int decode_nibble(int nibble, int shift)
{
	if (shift >= 13)
	{
		shift = 12;
		nibble >>= 3;	// arithmetic: -8..-1 -> -1, 0..+7 -> 0
	}
	return (nibble * (1 << shift)) >> 1;
}

// Extracts one of a BRR data byte's two signed nibbles: the high nibble is the
// first sample, the low nibble the second.
static int signed_nibble(UINT8 byte, int second)
{
	const int raw = second ? (byte & 0x0F) : (byte >> 4);
	return raw - ((raw & 0x08) ? 16 : 0);	// sign-extend to -8..+7
}

// The four BRR filters, in fullsnes's exact integer forms (cross-checked 1:1
// against Anomie's S-DSP doc). `old` is the previous 15-bit output, `older` the
// one before it. The right shifts are arithmetic over signed values.
static int apply_filter(int filter, int sample, int old, int older)
{
	switch(filter)
	{
	case 0:
		return sample;
	case 1:
		return sample + old + ((-old) >> 4);
	case 2:
		return sample + old * 2 + ((-old * 3) >> 5) - older + (older >> 4);
	default:	// filter 3
		return sample + old * 2 + ((-old * 13) >> 6) - older + ((older * 3) >> 4);
	}
}

// Clamps to signed 16 bits, then clips to signed 15 by re-reading bit 14 as the
// sign. This is the hardware's clamp-then-clip: a value below -8000h clamps to
// -8000h and then clips to 0 (the dirt-effect), and a magnitude that overflows
// 15 bits folds to the opposite sign (lost-sign). In-range values pass through.
static INT16 clamp_and_clip(int value)
{
	if (value > 0x7FFF) value = 0x7FFF;
	if (value < -0x8000) value = -0x8000;
	return (INT16)(((value & 0x7FFF) ^ 0x4000) - 0x4000);
}

// The exponential envelope decrease shared by Decay, Sustain and GAIN
// Exp-Decrease: level -= 1, then level -= level >> 8 (the right shift is
// arithmetic, so the two-step form never drives the level below 0).
static int exp_decrease(int level)
{
	const int stepped = level - 1;
	return stepped - (stepped >> 8);
}

// Saturates an envelope value to the unsigned 11-bit range 0..0x7FF (the
// hardware clamps rather than wraps: a decrease past 0 sticks at 0, an attack
// past 0x7FF sticks at 0x7FF).
static UINT16 clamp_envelope(int level)
{
	if (level < 0) return 0;
	if (level > 0x7FF) return 0x7FF;
	return (UINT16)level;
}

// The voice's base 14-bit pitch step, read live from VxPITCHL/H; the register
// pair's bits 14-15 are stored but never used.
static UINT32 voice_pitch(const SNES_DSP* dsp, unsigned int voice)
{
	const UINT32 low = dsp->regs[voice_register(voice, VOICE_PITCH_LOW)];
	const UINT32 high = dsp->regs[voice_register(voice, VOICE_PITCH_HIGH)];
	return ((high << 8) | low) & 0x3FFF;
}

// The pitch step this output sample, including pitch modulation. For a voice in
// 1..7 with its PMON bit set, the base step is scaled by the previous voice's
// amplitude from the previous sample: factor = (prev_amplitude SAR 4) + 400h (the range
// 000h..7FFh, i.e. 0.00..1.99), and step = (base * factor) SAR 10. A silent
// previous voice gives factor 400h, an unmodulated x1.0, so no special case is
// needed. The step itself is never capped (a modulated step reaches 7FEEh); the
// 128 kHz ceiling is applied to the position the step is added to, in
// advance_voice_stream.
// The pitch step a voice's stream advance uses this sample is the value its
// pitch slots read, never the live register pair (see SNES_DSP.pitch_latch).
static UINT32 pitch_step(const SNES_DSP* dsp, unsigned int voice, int prev_amplitude)
{
	UINT32 step = dsp->pitch_latch[voice];
	const int modulate = voice > 0 && ((dsp->latched_pmon >> voice) & 1) != 0;
	if (modulate)
	{
		const int factor = (prev_amplitude >> 4) + 0x400;	// -400h..+3FFh -> 000h..7FFh
		step = (UINT32)(((int)step * factor) >> 10);
	}
	return step;
}

// A key-on's startup keeps no interpolation fraction, and this window is
// where that holds: from the load through the first sounding compute. For a
// walking startup (a sounding voice re-keyed, see the voice's startup_walks)
// the pitch counter's fractional bits are cleared after each advance:
// whole-sample crossings stand, so the cursor still walks at the pitch
// (`KON/kon decoding when another kon`), while the first audible sample
// interpolates from index 0 and the fraction begins accumulating only with
// the advance after it (`KON/pitch at kon`: at pitch $0010 the audible ramp
// reads the Gaussian kernel at indices 0, 1, 2, ...; a retained startup
// fraction starts it six indices deep). For a held startup the same window
// bounds the stream hold itself: the advance is skipped outright, so there
// is no fraction to clear (`Misc/brr addr wrap-around`). The window is one
// call longer than the silent span because the advance precedes the
// interpolation within a compute, so the first sounding call's own advance
// still contributes no fraction. It is counted from the load, so an in-span
// rewind does not reopen it: the rewound stream keeps walking with its
// fraction intact.
INLINE int key_on_pins_fraction(const SNES_DSP_VOICE* v)
{
	return v->computes_since_key_on <= KEY_ON_SILENT_CALLS + 1;
}

// A BRR header's low two bits are its end/loop code. Code 1 (end set, loop clear)
// is End+Mute: the voice releases and its level drops to 0. Code 3 loops without
// muting, and codes 0 and 2 run on into the next block.
INLINE int header_is_end_mute(UINT8 header)
{
	return (header & 0x03) == 0x01;
}

// Advances the shared noise generator one step: a 15-bit right rotation whose new
// top bit is bit0 XOR bit1 of the old level. The level is held as the signed
// sample it outputs (-4000h..+3FFFh), so the step works over its 15-bit pattern.
static INT16 next_noise_level(INT16 level)
{
	const unsigned int pattern = (unsigned int)level & 0x7FFFu;
	const unsigned int feedback = ((pattern ^ (pattern >> 1)) & 1u) << 14;
	const unsigned int next = ((pattern >> 1) & 0x3FFFu) | feedback;
	return (INT16)((next ^ 0x4000u) - 0x4000u);	// sign-extend 15 bits
}

// Shifts one consumed sample into a voice's interpolation window.
static void shift_window(SNES_DSP_VOICE* v, INT16 sample)
{
	v->window.oldest = v->window.older;
	v->window.older = v->window.old;
	v->window.old = v->window.newest;
	v->window.newest = sample;
}

// Reads voice source `srcn`'s four-byte directory entry from the table based at
// DIR*$100: a little-endian start address (offset 0) followed by a little-endian
// loop address (offset 2). Addresses wrap within the 64KB space.
static UINT16 brr_pointer(const UINT8* ram, UINT8 dir, UINT8 srcn, int offset)
{
	const int base = dir * 0x100 + srcn * 4;
	const UINT16 lo = ram[(UINT16)(base + offset)];
	const UINT16 hi = ram[(UINT16)(base + offset + 1)];
	return (UINT16)(lo | (hi << 8));
}

// Interpolates a window at an 8-bit index (pitch counter bits 11-4) through
// the S-DSP's 4-point Gaussian kernel, in the hardware's exact arithmetic.
// Each tap is (gauss * sample) SAR 10 over the transcribed 512-entry hardware
// table. The additions of the older and old taps run in 16 bits with no
// overflow handling (the old tap's addition can wrap, a documented hardware
// bug the kernel reproduces) while the newest tap's addition saturates to
// -8000h/+7FFFh. The result is shifted right once, to 15 bits.
static INT16 gauss_interpolate(const SNES_DSP_WINDOW* window, UINT8 index)
{
	const int i = index;
	int out;
	// The first value cannot exceed 16 bits, and the first addition cannot
	// overflow them. The second addition has no overflow handling: it wraps,
	// reproducing the documented bug the table's 801h rows expose. The third
	// addition saturates.
	out = (gauss_table[0x0FF - i] * window->oldest) >> 10;
	out = (INT16)(out + ((gauss_table[0x1FF - i] * window->older) >> 10));
	out = (INT16)(out + ((gauss_table[0x100 + i] * window->old) >> 10));
	out = out + ((gauss_table[0x000 + i] * window->newest) >> 10);
	if (out > 0x7FFF) out = 0x7FFF;
	if (out < -0x8000) out = -0x8000;
	return (INT16)(out >> 1);
}

// The voice's current interpolated 15-bit sample: the Gaussian kernel over its
// window at the pitch counter's current index. Pure over the state: no
// advance, no memory access.
static INT16 interpolated_sample(const SNES_DSP* dsp, unsigned int voice)
{
	const SNES_DSP_VOICE* v = &dsp->voices[voice];
	return gauss_interpolate(&v->window, (UINT8)((v->pitch_counter >> 4) & 0xFF));
}

// Whether `stamp`, a CPU write's stamp, was issued in the two cycles before the
// cycle now running: the window in which it survives the DSP's own write to
// the same register (see SNES_DSP.cycle_count).
INLINE int cpu_write_stands(const SNES_DSP* dsp, UINT64 stamp)
{
	return stamp != SNES_DSP_NO_CPU_WRITE && dsp->cycle_count >= stamp &&
			dsp->cycle_count - stamp <= 2;
}

// Reads, at a voice's source slot, the `VxSRCN` its next directory reads
// select the entry with.
static void load_source_number(SNES_DSP* dsp, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	v->srcn = dsp->regs[voice_register(voice, VOICE_SRCN)];
	v->srcn_loaded = 1;
}

// The source number a directory read selects the entry with: the source
// slot's capture, or the live register on a state no source slot has run on.
static UINT8 voice_source_number(const SNES_DSP* dsp, unsigned int voice)
{
	const SNES_DSP_VOICE* v = &dsp->voices[voice];
	return v->srcn_loaded ? v->srcn : dsp->regs[voice_register(voice, VOICE_SRCN)];
}

// Reads, at a voice's directory slot, the `VxPITCHL` byte its pitch-high slot
// joins into the compute's step, and the `VxADSR1` the compute's envelope step
// runs under (`Timing/Voice/V2 adsr0.0F`/`.70`/`.80` pulse the register for
// five cycles at every offset and hash which voices' envelopes took it). A
// voice inside its key-on hold keeps the step the key-on's scheduled capture
// loaded (see SNES_DSP.pitch_latch); the `VxADSR1` read has no such hold.
static void load_directory_slot_registers(SNES_DSP* dsp, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	if (v->pitch_capture_hold == 0) v->pitch_low = dsp->regs[voice_register(voice, VOICE_PITCH_LOW)];
	v->adsr1 = dsp->regs[voice_register(voice, VOICE_ADSR1)];
	v->adsr1_loaded = 1;
}

// Reads, at a voice's pitch-high slot, the `VxPITCHH` byte and joins it with
// the directory slot's `VxPITCHL` into the pair the next sample's advance
// takes (SNES_DSP_VOICE.pitch_pending).
static void load_pitch_high(SNES_DSP* dsp, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	UINT32 high;
	if (v->pitch_capture_hold != 0) return;
	high = dsp->regs[voice_register(voice, VOICE_PITCH_HIGH)];
	v->pitch_pending = (UINT16)(((high << 8) | v->pitch_low) & 0x3FFF);
	v->pitch_pending_valid = 1;
}

// Reads PMON at slot T28, and NON, EON and DIR together at T29: the four
// registers that speak for all eight voices, each read once a sample at its own
// slot (see SNES_DSP.latched_pmon). A schedule without those slots reads them
// as it enters, so the frame-at-once sample sees the registers as they stand.
static void load_pitch_modulation(SNES_DSP* dsp)
{
	dsp->latched_pmon = dsp->regs[DSP_PMON];
}

static void load_voice_wide_registers(SNES_DSP* dsp)
{
	dsp->latched_non = dsp->regs[DSP_NON];
	dsp->latched_eon = dsp->regs[DSP_EON];
	dsp->latched_dir = dsp->regs[DSP_DIR];
}

static void latch_global_registers(SNES_DSP* dsp)
{
	load_pitch_modulation(dsp);
	load_voice_wide_registers(dsp);
}

// Reads, at a voice's directory slot, the loop address of its directory entry
// for the loop jump the compute may make this sample.
static void load_loop_pointer(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	v->loop_pointer = brr_pointer(ram, dsp->latched_dir, voice_source_number(dsp, voice), 2);
	v->loop_pointer_loaded = 1;
}

// Loads, at a voice's BRR load slot, the header the sample's end/loop check
// reads and the header and first data byte of every group decode the voice
// has scheduled.
static void load_scheduled_group_bytes(SNES_DSP_VOICE* v, const UINT8* ram)
{
	UINT8 n;
	v->loaded_header = ram[v->header_address];
	v->header_loaded = 1;
	for (n = 0; n < v->scheduled_decode_count; ++ n)
	{
		SNES_DSP_GROUP_DECODE* g = &v->scheduled_decodes[n];
		if (g->bytes_loaded) continue;
		if (! g->header_captured) g->header = ram[g->address];
		g->first_byte = ram[(UINT16)(g->address + 1 + g->offset / 2)];
		g->bytes_loaded = 1;
	}
}

// Decodes samples [from, to) of one scheduled group into a voice's pending
// ring. The header's shift and filter and the group's first data byte are the
// ones the voice's BRR load slot captured (or, for a decode the slot schedule
// has not loaded, read from RAM now); the second data byte is read from RAM at
// the call decoding its samples. The samples are decoded ahead of their
// consumption, so a RAM write after these reads does not reach them. The
// filter history is the decoder's own: the two samples decoded before these,
// which run ahead of the window's consumed taps; halves of one group must
// therefore decode in order with nothing between them, which the split
// schedule guarantees (the S4 half lands before the next compute can decode
// or consume anything newer).
static void decode_group_ahead(SNES_DSP* dsp, const UINT8* ram, unsigned int voice,
								const SNES_DSP_GROUP_DECODE* group, int from, int to)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	const UINT16 address = group->address;
	const int offset = group->offset;
	const UINT8 header =
		(group->bytes_loaded || group->header_captured) ? group->header : ram[address];
	const UINT8 first_byte = group->bytes_loaded
								? group->first_byte
								: ram[(UINT16)(address + 1 + offset / 2)];
	const UINT8 second_byte = ram[(UINT16)(address + 2 + offset / 2)];
	int k;
	for (k = from; k < to; ++ k)
	{
		const int index = offset + k;
		const UINT8 byte = k < 2 ? first_byte : second_byte;
		const int sample = decode_nibble(signed_nibble(byte, (index & 1) != 0), header >> 4);
		const INT16 decoded = clamp_and_clip(
			apply_filter((header >> 2) & 0x03, sample, v->decode_prev1, v->decode_prev2));
		v->decode_prev2 = v->decode_prev1;
		v->decode_prev1 = decoded;
		v->pending[(v->pending_head + v->pending_count) % 12] = decoded;
		++ v->pending_count;
	}
}

// The group scheduled is always the one two on from the cursor's: the
// in-group offset formula is the same for every index inside the group,
// so both scheduling modes share it.
static void schedule_next_group(SNES_DSP_VOICE* v, const UINT8* ram)
{
	const UINT8 offset = (UINT8)(((v->brr_sample_index & ~3) + 8) & 15);
	UINT8 n;
	for (n = 0; n < v->scheduled_decode_count; ++ n)
		if (v->scheduled_decodes[n].address == v->decoder_address &&
			v->scheduled_decodes[n].offset == offset)
			return;	// this group is already on the schedule
	if (v->scheduled_decode_count < 3)
	{
		// The group's header is read at the scheduling itself; the load slot
		// one sample on supplies the first data byte.
		SNES_DSP_GROUP_DECODE* g = &v->scheduled_decodes[v->scheduled_decode_count];
		g->address = v->decoder_address;
		g->offset = offset;
		g->header = ram[v->decoder_address];
		g->first_byte = 0;
		g->bytes_loaded = 0;
		g->decoded_samples = 0;
		g->header_captured = 1;
		++ v->scheduled_decode_count;
	}
}

// Advances a voice's decode cursor by one sample; the DECODER (the read that
// fills the voice's sample buffer) runs a group of four samples ahead of it.
// When the cursor consumes a block's DECODER_LEAD-th sample, the decoder
// leaves that block for the next: the chain is resolved there (the following
// block, or for an end block the loop address read live from DIR and VxSRCN,
// which is how a DIR or VxSRCN change takes effect at the next loop and not
// before). Leaving a block whose header carries the end flag is what sets the
// voice's ENDX bit: the bit is staged when the decoder has decoded the end
// block through and jumps to the loop address, not when it enters the end
// block (`Order/endx after final brr decode` reads the bit clear at the
// cursor's seventh sample of the end block and set at its eighth), and it
// reaches the register at the voice's S7 slot (SNES_DSP.prepared_endx). The
// exhausted cursor follows the address the decoder resolved.
//
// The consumed sample comes from the pending ring, decoded ahead of time: the
// key-on primed the first three groups, and each group-aligned consume here
// schedules the decode of the group eight stream samples on, always inside
// the decoder's block, at in-block offset (index + 8) & 15, which
// advance_voice_stream performs at the voice's next sample, before the cursor
// moves again (a modulated step can cross two boundaries in one sample, and
// both groups are scheduled). The hardware decodes a group in its sample's V4 step and only
// then advances the position (Anomie's V4 order), so a crossing's group is
// read from RAM one sample after the crossing: spc_dsp6 `Order/pitch after
// brr` rewrites a moving voice's header shift sample by sample and reads,
// through the echo tape, that the group crossed into under one header takes
// the shift of the header standing one sample later. The ring therefore holds
// four to twelve decoded samples and RAM writes cannot reach the samples
// already in it. A state seeded mid-stream without priming has an empty ring
// and decodes at consumption, from the cursor's own block with the window as
// filter history: the pre-ring arithmetic, unreachable from a machine-driven
// voice.
//
// The decoder entering an End+Mute block (header code 1: end set, loop clear)
// has no further effect here: the silencing is the per-sample header check's,
// which reads the header standing at each sample's start, so it lands at the
// NEXT sample's check, one sample after the decoder moved in.
static void decode_stream_sample(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	INT16 consumed;
	if (v->brr_sample_index >= 16)
	{
		v->brr_address = v->decoder_address;
		v->brr_sample_index = 0;
	}
	if (v->pending_count == 0)
	{
		const UINT8 header = ram[v->brr_address];
		const UINT8 byte =
			ram[(UINT16)(v->brr_address + 1 + v->brr_sample_index / 2)];
		const int sample = decode_nibble(signed_nibble(byte, (v->brr_sample_index & 1) != 0),
											header >> 4);
		consumed = clamp_and_clip(
			apply_filter((header >> 2) & 0x03, sample, v->decode_prev1, v->decode_prev2));
	}
	else
	{
		if (v->brr_sample_index % 4 == DECODE_TRIGGER_INDEX) schedule_next_group(v, ram);
		consumed = v->pending[v->pending_head];
		v->pending_head = (UINT8)((v->pending_head + 1) % 12);
		-- v->pending_count;
	}
	shift_window(v, consumed);
	++ v->brr_sample_index;
	if (v->brr_sample_index == DECODER_LEAD)
	{
		if ((ram[v->brr_address] & 0x01) != 0)
		{
			// The loop jump takes the address the voice's directory slot read this
			// sample; a schedule without that slot reads the directory here.
			v->decoder_address =
				v->loop_pointer_loaded
					? v->loop_pointer
					: brr_pointer(ram, dsp->regs[DSP_DIR], dsp->regs[voice_register(voice, VOICE_SRCN)], 2);
			dsp->prepared_endx |= (UINT8)(1u << voice);
		}
		else
		{
			v->decoder_address = (UINT16)(v->brr_address + 9);
		}
	}
}

// Finishes, at a voice's S4 slot, the group decode the compute halved this
// sample (its last two samples, the second data byte read from RAM now) and
// keeps anything the compute scheduled since for the next sample's load.
static void perform_loaded_decodes(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	UINT8 kept = 0;
	UINT8 n;
	for (n = 0; n < v->scheduled_decode_count; ++ n)
	{
		if (v->scheduled_decodes[n].decoded_samples == 2)
		{
			decode_group_ahead(dsp, ram, voice, &v->scheduled_decodes[n], 2, 4);
		}
		else
		{
			v->scheduled_decodes[kept ++] = v->scheduled_decodes[n];
		}
	}
	v->scheduled_decode_count = kept;
}

// Advances a voice's stream by the samples this 32 kHz output sample passes: the
// counter's in-group position gains `step`, the sum clamps at
// MAX_GROUP_POSITION, and every whole sample position the clamped sum crosses is
// decoded. The clamp is what bounds the advance at four source samples per
// output sample; a step that would carry the position past the ceiling leaves
// it at 3FFFh within the next group, where a later step of even 1 crosses the
// group boundary at once (`Misc/interp pos clamped at $7FFF`). The group decodes
// the previous advance's crossings scheduled run first, reading RAM now: the
// decode-then-advance order of the hardware's V4 step (see decode_stream_sample).
static void advance_voice_stream(SNES_DSP* dsp, const UINT8* ram, unsigned int voice, UINT32 step)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	UINT32 position;
	UINT32 advanced;
	UINT32 passed;
	UINT32 n;
	UINT8 n8;
	if (! dsp->primed)
	{
		// The frame-at-once path has no slot schedule: decode everything scheduled
		// in full now, every byte read from RAM here.
		for (n8 = 0; n8 < v->scheduled_decode_count; ++ n8)
			decode_group_ahead(dsp, ram, voice, &v->scheduled_decodes[n8], 0, 4);
		v->scheduled_decode_count = 0;
	}
	else
	{
		// The slot-scheduled path decodes the groups whose bytes this sample's
		// load slot captured: two load slots after their trigger. The ordinary
		// case is one group: its first two samples decode now, ahead of this
		// advance, and the S4 slot finishes it, reading the second data byte
		// there (the measured read slots), still ahead of anything the next
		// advance can consume even through the position clamp. A double
		// crossing's pair decodes in full now, in stream order; a group still
		// waiting for its load stays.
		UINT8 loaded = 0;
		for (n8 = 0; n8 < v->scheduled_decode_count; ++ n8)
			if (v->scheduled_decodes[n8].bytes_loaded && v->scheduled_decodes[n8].decoded_samples == 0) ++ loaded;
		if (loaded == 1)
		{
			for (n8 = 0; n8 < v->scheduled_decode_count; ++ n8)
			{
				SNES_DSP_GROUP_DECODE* g = &v->scheduled_decodes[n8];
				if (g->bytes_loaded && g->decoded_samples == 0)
				{
					decode_group_ahead(dsp, ram, voice, g, 0, 2);
					g->decoded_samples = 2;
				}
			}
		}
		else if (loaded > 1)
		{
			UINT8 kept = 0;
			for (n8 = 0; n8 < v->scheduled_decode_count; ++ n8)
			{
				SNES_DSP_GROUP_DECODE* g = &v->scheduled_decodes[n8];
				if (g->bytes_loaded && g->decoded_samples == 0)
				{
					decode_group_ahead(dsp, ram, voice, g, 0, 4);
				}
				else
				{
					v->scheduled_decodes[kept ++] = *g;
				}
			}
			v->scheduled_decode_count = kept;
		}
	}
	position = v->pitch_counter & GROUP_POSITION_MASK;
	advanced = position + step;
	if (advanced > MAX_GROUP_POSITION) advanced = MAX_GROUP_POSITION;
	passed = (advanced >> 12) - (position >> 12);
	if (passed > v->pending_count)
	{
		// A clamp-grade drain can reach a group's second pair one advance after
		// its scheduling, before the S4 slot decodes it. The hardware cannot
		// starve (its decode and advance share that slot, decode first) so the
		// outstanding halves are finished here, their second data byte read one
		// slot early, only when this advance would otherwise outrun the ring.
		perform_loaded_decodes(dsp, ram, voice);
	}
	v->pitch_counter = (UINT16)((v->pitch_counter & ~GROUP_POSITION_MASK) + advanced);
	for (n = 0; n < passed; ++ n) decode_stream_sample(dsp, ram, voice);
}

// Clamps a mix accumulator to signed 16 bits: the hardware clamps (never wraps)
// after each addition in the output sum.
INLINE INT32 clamp_signed16(INT32 value)
{
	if (value > 0x7FFF) return 0x7FFF;
	if (value < -0x8000) return -0x8000;
	return value;
}

// Reads one 16-bit little-endian word of a ring entry. Echo samples are 15 bits
// stored left-justified, so the arithmetic SAR 1 recovers the value the filter
// takes. The address wraps within the 64KB space.
static INT16 read_echo_word(const UINT8* ram, UINT16 entry, int low_byte)
{
	const int word = (INT16)(ram[(UINT16)(entry + low_byte)] | (ram[(UINT16)(entry + low_byte + 1)] << 8));
	return (INT16)(word >> 1);
}

// Captures a run of FIR coefficients into the sample's own copies. Each lands at
// its own slot, so a CPU write reaches this sample's filter only while its
// coefficient is still ahead.
static void latch_fir_coefficients(SNES_DSP* dsp, unsigned int first, unsigned int count)
{
	unsigned int tap;
	for (tap = first; tap < first + count; ++ tap)
		dsp->echo_fir_coeff[tap] = (INT8)dsp->regs[voice_register(tap, FIR_COEFF)];
}

// Slot T23: the left channel's word comes out of the ring, and FIR0 with it. The
// entry this sample reads and writes is fixed here: the ring index only moves at
// T31, so the right read, the feedback and the write-back all address it.
static void load_echo_left(SNES_DSP* dsp, const UINT8* ram, UINT8 base_esa)
{
	const UINT16 base = (UINT16)(base_esa * 0x100);
	dsp->echo_write_entry = (UINT16)(base + dsp->echo_index * 4);
	dsp->echo_fir_left[dsp->echo_fir_pos] = read_echo_word(ram, dsp->echo_write_entry, 0);
	latch_fir_coefficients(dsp, 0, 1);
}

// Slot T24: the right channel's word, and FIR1 and FIR2.
static void load_echo_right(SNES_DSP* dsp, const UINT8* ram)
{
	dsp->echo_fir_right[dsp->echo_fir_pos] = read_echo_word(ram, dsp->echo_write_entry, 2);
	latch_fir_coefficients(dsp, 1, 2);
}

// One channel of the echo FIR filter: taps oldest*FIR0 ... newest*FIR7, each
// product SAR 6. The first seven additions wrap at 16 bits; only the final (newest)
// addition saturates. The newest history slot is `pos`, so tap k reads slot
// (pos+1+k) & 7. The output is a 15-bit sample left-aligned in 16 bits, so the sum's
// low bit is dropped before EVOL and EFB read it.
static int filter_echo_channel(const INT16* history, const INT8* coeff, UINT8 pos)
{
	int sum = 0;
	int tap;
	for (tap = 0; tap < 8; ++ tap)
	{
		const UINT8 slot = (UINT8)((pos + 1 + tap) & 7);
		const int product = (INT16)((history[slot] * coeff[tap]) >> 6);
		sum = tap < 7 ? (INT16)(sum + product) : clamp_signed16(sum + product);
	}
	return sum & ~1;
}

// Slot T26: the last two coefficients arrive and the filter runs, leaving each
// channel's output for the output slots and the feedback to take.
//
// FIR: taps oldest*FIR0 ... newest*FIR7, each product SAR 6. The first seven
// additions wrap at 16 bits; only the final (newest) addition saturates. The
// newest history slot is echo_fir_pos, so tap k reads slot (echo_fir_pos+1+k) & 7.
// The output is a 15-bit sample left-aligned in 16 bits, so the sum's low bit is
// dropped before EVOL and EFB read it. The two channels filter separately with
// the same coefficients.
//
// Each product is itself a 16-bit value and wraps. A coefficient of -128 against
// a sample of -4000h gives +8000h, one past the range, and the wrap makes it
// -8000h. Only the saturating final add can tell the two apart: for the seven
// wrapping adds, truncating the product and truncating the running sum are the
// same arithmetic modulo 2^16.
static void filter_echo(SNES_DSP* dsp)
{
	latch_fir_coefficients(dsp, 6, 2);
	dsp->echo_fir_out_left = filter_echo_channel(dsp->echo_fir_left, dsp->echo_fir_coeff, dsp->echo_fir_pos);
	dsp->echo_fir_out_right = filter_echo_channel(dsp->echo_fir_right, dsp->echo_fir_coeff, dsp->echo_fir_pos);

	// Advance the FIR history cursor. The ring index advances at T31, not here.
	dsp->echo_fir_pos = (UINT8)((dsp->echo_fir_pos + 1) & 7);
}

// Slot T27: EFB is read and the write-back value formed: the EON send
// (send_left/send_right, the EON-enabled voices' post-VxVOL sums) plus fir*EFB
// SAR 7, clamped, bit 0 cleared, because the entry holds a 15-bit sample and the
// volume multiply can reintroduce the low bit.
//
// The volume multiply is 16 bits wide and wraps; the add onto the send is what
// saturates. EFB 80h against a FIR output of -8000h gives +8000h, which the wrap
// takes to -8000h where a saturating product would give 7FFFh.
//
// The bytes do not land here: the buffer writes have their own slots (the left
// word at T30, the right word at T31) so the value is staged and the slot runner
// (or the frame-at-once caller) performs the write when its slot arrives, each
// word under the FLG bit-5 gate loaded one slot before it.
static void stage_echo_write(SNES_DSP* dsp, int send_left, int send_right)
{
	const int efb = (INT8)dsp->regs[DSP_EFB];
	const int write_left =
		clamp_signed16(send_left + (INT16)((dsp->echo_fir_out_left * efb) >> 7)) & ~1;
	const int write_right =
		clamp_signed16(send_right + (INT16)((dsp->echo_fir_out_right * efb) >> 7)) & ~1;
	dsp->echo_write_pending = 1;
	dsp->echo_write_bytes[0] = (UINT8)(write_left & 0xFF);
	dsp->echo_write_bytes[1] = (UINT8)((write_left >> 8) & 0xFF);
	dsp->echo_write_bytes[2] = (UINT8)(write_right & 0xFF);
	dsp->echo_write_bytes[3] = (UINT8)((write_right >> 8) & 0xFF);
}

// The end-of-sample echo ring advance (slot T31): applies the ESA and EDL values
// loaded at T30, then steps the ring index. EDL is applied only when the index is
// 0 (the entry count is EDL<<9, and a count of 0 collapses to the single-entry
// buffer) so a mid-ring EDL change waits for the next wrap, while the applied
// base is what the following sample's read and write address. A CPU write to ESA
// therefore reaches the buffer one sample after the write (anomie's access chart,
// cycles 29-30; spc_dsp6 `Misc/$F0-$FF are not ram` flips ESA mid-ring and its
// echo burst is calibrated to the in-flight sample landing at the old base).
static void advance_echo_ring(SNES_DSP* dsp, UINT8 raw_esa, UINT8 raw_edl)
{
	dsp->echo_applied_esa = raw_esa;
	if (dsp->echo_index == 0)
		dsp->echo_length = (UINT16)((raw_edl & 0x0F) << 9);
	dsp->echo_index = (UINT16)(dsp->echo_index + 1);
	if (dsp->echo_index >= dsp->echo_length) dsp->echo_index = 0;
}

// Wipes a voice's stream state for a key-on, reading no RAM: the addresses
// are taken when the start pointer is read (load_start_pointer) and the ring
// filled when the stream is primed (prime_voice_stream). This is also the
// default VoiceState: Release, computes_since_key_on 0xFF, everything else 0.
static void voice_default(SNES_DSP_VOICE* v)
{
	memset(v, 0x00, sizeof(SNES_DSP_VOICE));
	v->phase = SNES_DSP_PHASE_RELEASE;
	v->computes_since_key_on = 0xFF;
}

static void reset_voice_for_start(SNES_DSP* dsp, unsigned int voice)
{
	voice_default(&dsp->voices[voice]);
}

// Reads a keyed voice's start pointer from its directory entry. On the slot
// schedule this runs at the voice's directory slot of the sample after the
// key-on's consuming compute: T22 for voice 0, T(3v-2) for voices 1-7, the
// slot `Timing/Voice/V2 dir.start.lsb`/`.msb` measure: a five-cycle pulse into
// either start byte is caught only when it covers that slot. The entry is
// selected by the source slot's `VxSRCN` capture (load_source_number); its loop
// address is the directory slot's own per-sample read (load_loop_pointer).
static void load_start_pointer(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	const UINT16 start = brr_pointer(ram, dsp->latched_dir, voice_source_number(dsp, voice), 0);
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	v->brr_address = start;
	v->decoder_address = start;
	v->header_address = start;
}

// Primes a keyed voice's stream from its start block: decodes the block's
// first three groups (twelve samples) into the ring, reading their bytes
// from RAM now, and shifts the first four into the window. A write to those
// bytes after this read does not reach the primed samples (`Misc/brr not
// always decoding` rewrites a parked voice's block and the voice still plays
// all twelve when it moves). On the slot schedule this runs at the voice's
// BRR load slot of the sample the start pointer was read in.
static void prime_voice_stream(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	int group;
	int n;
	for (group = 0; group < 3; ++ group)
	{
		SNES_DSP_GROUP_DECODE g;
		g.address = v->decoder_address;
		g.offset = (UINT8)(group * 4);
		g.header = 0;
		g.first_byte = 0;
		g.bytes_loaded = 0;
		g.decoded_samples = 0;
		g.header_captured = 0;
		decode_group_ahead(dsp, ram, voice, &g, 0, 4);
	}
	for (n = 0; n < 4; ++ n)
	{
		shift_window(v, v->pending[v->pending_head]);
		v->pending_head = (UINT8)((v->pending_head + 1) % 12);
		-- v->pending_count;
	}
	v->brr_sample_index = 4;
	// Priming leaves ENDX alone: the bit is set when the decoder LEAVES an
	// end block (decode_stream_sample), so a start block carrying the end flag
	// sets it four cursor samples on, when the decoder resolves its loop.
}

static void start_voice(SNES_DSP* dsp, const UINT8* ram, unsigned int voice)
{
	// The key-on call has no T29 to read the directory register at, so it
	// reads it here and the pointer read below takes that value.
	load_voice_wide_registers(dsp);
	reset_voice_for_start(dsp, voice);
	load_start_pointer(dsp, ram, voice);
	prime_voice_stream(dsp, ram, voice);
}

// Whether the envelope/noise operation at 5-bit rate `rate` fires at this global
// counter value: (counter + offset[rate]) % period[rate] == 0. Rate 0 (period
// Infinite) never fires; rate 31 (period 1) fires every sample.
static int envelope_rate_fires(UINT16 counter, UINT8 rate)
{
	const int period = envelope_period[rate & 0x1F];
	int offset;
	if (period == 0) return 0;	// rate 0 (Infinite) never fires
	offset = envelope_offset[rate & 0x1F];
	return (counter + offset) % period == 0;
}

// Advances the DSP's per-sample global state to the next 32 kHz sample: the
// global counter (decrement, wrapping to 0x77FF when it would pass below 0) and
// the power-on sample index. The slot schedule runs it at T31 after that
// sample's KON/KOFF poll and before voice 0's compute and the noise step, so
// those checks (and voices 1-7's in the following frame) read the advanced
// counter. The poll reads the pre-tick sample index, so the first sample after
// power-on carries index 0 (even) and polls.
static void tick_dsp_sample(SNES_DSP* dsp)
{
	dsp->global_counter = dsp->global_counter == 0 ? (UINT16)GLOBAL_COUNTER_RELOAD : (UINT16)(dsp->global_counter - 1);
	++ dsp->sample_index;
}

static void run_envelope_mode(SNES_DSP* dsp, unsigned int voice);
static void key_on_voice_impl(SNES_DSP* dsp, const UINT8* ram, unsigned int voice, int prime_now);
static void step_voice_envelope(SNES_DSP* dsp, unsigned int voice, int brr_end_mute);

// One voice's per-sample body: advance it by one 32 kHz output sample and return
// its enveloped amplitude: the internal -4000h..+3FFFh value VxOUTX reports and
// the next voice's PMON reads one sample later. `prev_amplitude` is the previous
// voice's amplitude from the previous sample (for pitch modulation); `soft_reset`
// is FLG bit 7. Writes VxOUTX;
// advances the stream and the envelope (or the key-on countdown). Extracted so the
// frame-at-once path and the per-slot schedule compute a voice bit-for-bit alike.
static int compute_voice_amplitude(SNES_DSP* dsp, const UINT8* ram, unsigned int voice,
									int prev_amplitude, int soft_reset)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	UINT32 step;
	int compute_since_key_on;
	int header_end_mute;
	int key_on_consumed_this_sample;
	int amplitude;

	if (v->computes_since_key_on != 0xFF) ++ v->computes_since_key_on;
	step = pitch_step(dsp, voice, prev_amplitude);

	// The header byte is loaded from RAM every sample, whatever the pitch counter is
	// doing, and its end/loop bits are read the same sample. So a block that turns
	// End+Mute under a voice whose stream has stopped still releases it. This check
	// is the envelope's ONLY End+Mute source, and it reads header_address: the
	// decoder's block as of the voice's live samples, one sample behind them,
	// so a startup whose decoder crossed into an End+Mute block is seen here
	// only from the third live sample's check (see the voice's header_address
	// for the rule and what pins it).
	//
	// The check goes live two sample periods after the keying poll loads a key-on.
	// With the load preceding voice 0's compute in the slot they share, the load's
	// slot is every voice's reference compute and the cutoff is a uniform count:
	// live from the third compute after (and including) the load's own sample.
	// The counter stops at the sixth compute, which is past it for every voice.
	compute_since_key_on = KEY_ON_STARTUP_CALLS + 1 - v->kon_delay;
	header_end_mute = compute_since_key_on >= 3 &&
						header_is_end_mute(v->header_loaded ? v->loaded_header
															: ram[v->header_address]);

	// A key-on the keying poll consumed THIS sample shields its voice from the
	// sample's soft reset: the fresh consumption wins, exactly as KON applied
	// after KOFF wins at the poll itself, and the startup proceeds as if the
	// reset were not standing. One sample later the reset arm below keys the
	// voice off like any other (spc_dsp6 `KON/kon then flg.80`: a one-sample
	// FLG bit-7 pulse over the consuming poll's sample leaves the startup's
	// ENVX schedule untouched; the same pulse one sample later, or anywhere
	// else in or past the countdown, silences the voice for good). The
	// consumption sample is the only one that can see the countdown still at
	// its armed value: every later compute has decremented it.
	key_on_consumed_this_sample = v->kon_delay == KEY_ON_STARTUP_CALLS;

	if (v->restart_pending)
	{
		// The final pre-key-on sample. The keying poll consumed a full restart for
		// this voice and armed its countdown, but the stream, envelope and phase
		// still stand: this compute advances the old stream once more (the final
		// pre-key-on decode) and emits its sample under the standing envelope,
		// and only then applies the restart. The countdown ticks on this very
		// call, so the ENVX schedule is exactly the armed key-on's; the old
		// decode's ENDX side effect is erased by the key-on's clear at the voice's
		// S7 slot, which is the suppression `KON/kon stops endx of prev sample`
		// measures. A standing
		// soft reset does not reach this branch: the fresh consumption wins, as
		// at the poll itself (`KON/kon then flg.80`). Measured against `KON/kon
		// unaffected by pitch`: a re-key on a sounding voice emits the old data
		// once more on the consuming sample, at every pitch.
		const int noise = ((dsp->latched_non >> voice) & 1) != 0;
		int sample;
		UINT8 count;
		UINT8 hold;
		v->restart_pending = 0;
		advance_voice_stream(dsp, ram, voice, step);
		sample = noise ? dsp->noise_level : interpolated_sample(dsp, voice);
		amplitude = (sample * (int)v->envelope) >> 11;
		// The standing envelope takes this sample's own update before the restart
		// reads it (a header standing End+Mute kills it, otherwise the selected
		// mode runs once) so whether the startup walks or holds is decided by
		// the level the voice would have carried into the next sample, not the
		// one it emitted with. `Order/endx after final brr decode` re-keys its
		// sync gadget's voice 4 with the gadget's direct-gain-0 restore landing
		// on the compute before the poll: the gadget's full level stands at the
		// emission and is 0 by the restart, and the ROM's row for that voice is a
		// held startup's, identical to the seven fresh key-ons.
		if (header_end_mute)
		{
			v->envelope = 0;
		}
		else
		{
			run_envelope_mode(dsp, voice);
		}
		// The restart proper: wipe and re-prime the stream, envelope from 0,
		// Attack, ENDX cleared. The counter and the capture hold carry the values
		// the poll left (key_on_voice_impl resets them for its direct callers), and the
		// armed countdown takes this call as the startup's first.
		// On the slot schedule the start pointer is read at the voice's directory
		// slot of the NEXT sample and the stream primed at that sample's load
		// slot, ahead of its compute; the frame-at-once path reads both here.
		count = v->computes_since_key_on;
		hold = v->pitch_capture_hold;
		key_on_voice_impl(dsp, ram, voice, ! dsp->primed);
		v->computes_since_key_on = count;
		v->pitch_capture_hold = hold;
		-- v->kon_delay;
		dsp->regs[voice_register(voice, VOICE_ENVX)] = 0;
	}
	else if (soft_reset && ! key_on_consumed_this_sample)
	{
		// FLG bit 7 keys every voice off and forces its envelope to 0 each sample. BRR
		// decoding keeps running (ENDX and loop transitions still fire). The reset
		// is read AFTER the sample's amplitude is formed, so a live voice still
		// emits this sample under the level it carried in; the zeroed level is
		// what the next sample scales by, the same one-sample lag every envelope
		// update has (Anomie's V3c: the envelope is applied, VxOUTX formed, and
		// FLG bit 7 checked before the update). spc_dsp6's `Order/flg.80 after
		// env used` measures it on the echo tape: a voice keyed on and reset nine
		// samples later writes one more full-scale frame than a model that
		// silences the reset sample itself. A voice still inside its startup
		// countdown emits silence either way.
		v->phase = SNES_DSP_PHASE_RELEASE;
		if (v->kon_delay > 0)
		{
			-- v->kon_delay;
			amplitude = 0;
		}
		else
		{
			const int noise = ((dsp->latched_non >> voice) & 1) != 0;
			int sample;
			advance_voice_stream(dsp, ram, voice, step);
			v->header_address = v->decoder_address;
			sample = noise ? dsp->noise_level : interpolated_sample(dsp, voice);
			amplitude = (sample * (int)v->envelope) >> 11;
		}
		v->envelope = 0;
		dsp->regs[voice_register(voice, VOICE_ENVX)] = 0;
	}
	else if (v->kon_delay > 0)
	{
		// Startup: the voice outputs silence, and whether its stream advances is
		// the key-on's walk split (the voice's startup_walks). A walking startup
		// (a young sounding voice re-keyed) advances at the pitch, except on the first
		// startup call, which decodes nothing (the start pointer is read and the
		// stream primed at the next sample's directory and load slots);
		// measured against spc_dsp6's `KON/kon decoding when another kon`, which
		// freezes the pitch mid-startup of a re-keyed sounding voice and reads
		// where the cursor stood: both published references hold every startup's
		// stream still, which that ROM refutes for this case. A held startup (a
		// silent voice keyed on) stands at its primed start, exactly as those
		// references describe (`Misc/brr addr wrap-around`). The envelope holds
		// through the countdown either way (step_voice_envelope decrements it), and
		// the header check above still runs. The first-call test reads the
		// compute count, not the countdown alone, because an in-span re-key
		// reloads the countdown without re-priming the stream: that reload's next
		// call advances as normal (`KON/kon then another kon`).
		if (v->startup_walks && (v->kon_delay < KEY_ON_STARTUP_CALLS || v->computes_since_key_on != 1))
			advance_voice_stream(dsp, ram, voice, step);
		if (key_on_pins_fraction(v)) v->pitch_counter &= 0xF000;
		step_voice_envelope(dsp, voice, header_end_mute);
		amplitude = 0;
	}
	else
	{
		// The envelope's End+Mute input was read above, before this advance: the
		// hardware checks the header early in the voice's sample and decodes after
		// (Anomie's V3c before V4) and header_address takes the decoder's block
		// only now that the check has passed. So the decoder entering an End+Mute
		// block sets ENDX at once but silences only at the next sample's check,
		// and a startup whose decoder crossed during the countdown silences after
		// its first live steps. The gap is audible: those steps publish their
		// level, which is what spc_dsp6's shared sync spins on (`KON/kon when
		// prev sample at end` races a stream into an End+Mute block mid-startup,
		// then waits for VxENVX != 0; on hardware the wait exits).
		// A non-walking startup's hold extends through this, the first live
		// compute: the sample interpolates the primed stream's first four samples
		// at fraction 0, and advancing begins the sample after.
		const int noise = ((dsp->latched_non >> voice) & 1) != 0;
		int sample;
		int envelope;
		if (v->startup_walks || ! key_on_pins_fraction(v)) advance_voice_stream(dsp, ram, voice, step);
		if (key_on_pins_fraction(v)) v->pitch_counter &= 0xF000;
		// The check's view stands one sample longer than the countdown: the first
		// live compute's advance does not reach the check until the SECOND live
		// sample, so a walking startup whose decoder crossed into an End+Mute
		// block on that advance still publishes its first level before the check
		// lands (`KON/kon as prev sample ends` reads that level after every one
		// of its swept key-ons).
		if (v->computes_since_key_on != KEY_ON_SILENT_CALLS) v->header_address = v->decoder_address;
		// A voice whose NON bit is set outputs the shared noise level in place of its
		// interpolated BRR sample; the stream's advance above is untouched by the
		// substitution, so decoding and ENDX are unaffected.
		sample = noise ? dsp->noise_level : interpolated_sample(dsp, voice);
		// The level scaling this sample is the one already standing; the update below
		// is what the next sample reads. So a voice leaving its startup samples emits
		// one more silent sample as its envelope begins moving, and a level in motion
		// reaches the output a sample after the register driving it is read.
		envelope = (int)v->envelope;
		amplitude = (sample * envelope) >> 11;
		step_voice_envelope(dsp, voice, header_end_mute);
	}

	// VxOUTX returns the high byte of the 15-bit amplitude (-128..+127).
	dsp->regs[voice_register(voice, VOICE_OUTX)] = (UINT8)((amplitude >> 7) & 0xFF);
	return amplitude;
}

// Folds one voice's amplitude into the running left mix and the EON echo send
// through its left volume (the S4 slot's work), clamping to signed 16 bits. A muted
// voice adds nothing (see the port notes).
static void apply_voice_left(SNES_DSP* dsp, unsigned int voice)
{
	const int vol = (INT8)dsp->regs[voice_register(voice, VOICE_VOL_LEFT)];
	int send = (dsp->voice_amplitude[voice] * vol) >> 6;
	if (((dsp->mute_mask >> voice) & 1) != 0) send = 0;
	dsp->mix_left = clamp_signed16(dsp->mix_left + send);
	if (((dsp->latched_eon >> voice) & 1) != 0)
		dsp->echo_send_left = clamp_signed16(dsp->echo_send_left + send);
}

// The S5 slot's work: the same fold for the right channel.
static void apply_voice_right(SNES_DSP* dsp, unsigned int voice)
{
	const int vol = (INT8)dsp->regs[voice_register(voice, VOICE_VOL_RIGHT)];
	int send = (dsp->voice_amplitude[voice] * vol) >> 6;
	if (((dsp->mute_mask >> voice) & 1) != 0) send = 0;
	dsp->mix_right = clamp_signed16(dsp->mix_right + send);
	if (((dsp->latched_eon >> voice) & 1) != 0)
		dsp->echo_send_right = clamp_signed16(dsp->echo_send_right + send);
}

static void poll_keying(SNES_DSP* dsp);

// The frame-at-once pipeline, run for a freshly seeded (unprimed) DspState's first
// sample. An unprimed state has no prepared voice-0 output, so its first sample
// reads every register at once and every voice in-frame (byte-identical to a
// machine with no intra-sample schedule) then primes the schedule for the samples
// that follow. The finished frame lands in slot_frame_left / slot_frame_right.
static void step_dsp_sample_atomic(SNES_DSP* dsp, UINT8* ram)
{
	UINT8 flg;
	int soft_reset;
	INT32 left = 0;
	INT32 right = 0;
	INT32 echo_left = 0;
	INT32 echo_right = 0;
	unsigned int voice;
	int mvol_left;
	int mvol_right;
	int echo_out_left;
	int echo_out_right;
	int evol_left;
	int evol_right;
	int b;

	// A freshly seeded state holds no pitch captures yet: take them from the
	// registers as they stand, so this frame reads the same values a live
	// register read would: byte-identical to the frame-at-once model.
	for (voice = 0; voice < 8; ++ voice)
		dsp->pitch_latch[voice] = (UINT16)voice_pitch(dsp, voice);
	// The voice-wide registers likewise: this frame has no T28/T29 to read them
	// at, so it takes them as they stand.
	latch_global_registers(dsp);
	poll_keying(dsp);

	flg = dsp->regs[DSP_FLG];
	soft_reset = (flg & FLG_SOFT_RESET) != 0;

	for (voice = 0; voice < 8; ++ voice)
	{
		// Pitch modulation reads the previous voice's amplitude from the previous
		// sample, exactly as the slot schedule does.
		const int prev_amplitude = dsp->modulator_amplitude[(voice + 7) & 7];
		int amplitude;
		int vol_left;
		int vol_right;
		int send_left;
		int send_right;
		dsp->modulator_amplitude[voice] = dsp->voice_amplitude[voice];
		amplitude = compute_voice_amplitude(dsp, ram, voice, prev_amplitude, soft_reset);
		// Record each voice's amplitude: voice 0's becomes the value the following
		// slot-scheduled frame applies (its output rides one frame behind), so the
		// first sample's voice-0 advance is not repeated. Seed the prepared OUTX/ENVX
		// too, so the first slot-scheduled frame's visibility slots publish this
		// sample's values rather than an empty scratch.
		dsp->voice_amplitude[voice] = amplitude;
		dsp->prepared_outx[voice] = dsp->regs[voice_register(voice, VOICE_OUTX)];
		dsp->prepared_envx[voice] = dsp->regs[voice_register(voice, VOICE_ENVX)];
		vol_left = (INT8)dsp->regs[voice_register(voice, VOICE_VOL_LEFT)];
		vol_right = (INT8)dsp->regs[voice_register(voice, VOICE_VOL_RIGHT)];
		send_left = (amplitude * vol_left) >> 6;
		send_right = (amplitude * vol_right) >> 6;
		if (((dsp->mute_mask >> voice) & 1) != 0)
		{
			send_left = 0;
			send_right = 0;
		}
		left = clamp_signed16(left + send_left);
		right = clamp_signed16(right + send_right);
		if (((dsp->latched_eon >> voice) & 1) != 0)
		{
			echo_left = clamp_signed16(echo_left + send_left);
			echo_right = clamp_signed16(echo_right + send_right);
		}
	}
	// Voice 0's next compute is at T31 of the following frame, after voice 1's
	// S3 has read the modulator there; this sample's amplitude is what that read
	// must see, so it is published once every voice of this frame has read.
	dsp->modulator_amplitude[0] = dsp->voice_amplitude[0];

	mvol_left = (INT8)dsp->regs[DSP_MVOL_LEFT];
	mvol_right = (INT8)dsp->regs[DSP_MVOL_RIGHT];
	left = (INT16)((left * mvol_left) >> 7);
	right = (INT16)((right * mvol_right) >> 7);

	// The frame-at-once path has no slot interleave, so the echo unit reads the
	// live ESA, the write lands within the call under the live FLG bit-5 gate, and
	// the ring advance applies the live registers, which warms the applied base
	// for the slot-scheduled samples that follow the seed frame.
	load_echo_left(dsp, ram, dsp->regs[DSP_ESA]);
	load_echo_right(dsp, ram);
	latch_fir_coefficients(dsp, 3, 3);
	filter_echo(dsp);
	stage_echo_write(dsp, echo_left, echo_right);
	echo_out_left = dsp->echo_fir_out_left;
	echo_out_right = dsp->echo_fir_out_right;
	if (dsp->echo_write_pending && (dsp->regs[DSP_FLG] & FLG_ECHO_WRITE_DISABLE) == 0)
	{
		for (b = 0; b < 4; ++ b)
			ram[(UINT16)(dsp->echo_write_entry + b)] = dsp->echo_write_bytes[b];
	}
	dsp->echo_write_pending = 0;
	advance_echo_ring(dsp, dsp->regs[DSP_ESA], dsp->regs[DSP_EDL]);
	evol_left = (INT8)dsp->regs[DSP_EVOL_LEFT];
	evol_right = (INT8)dsp->regs[DSP_EVOL_RIGHT];
	left = clamp_signed16(left + (INT16)((echo_out_left * evol_left) >> 7));
	right = clamp_signed16(right + (INT16)((echo_out_right * evol_right) >> 7));

	if ((flg & FLG_MUTE) != 0)
	{
		left = 0;
		right = 0;
	}

	if (envelope_rate_fires(dsp->global_counter, flg & FLG_NOISE_RATE))
		dsp->noise_level = next_noise_level(dsp->noise_level);
	// The frame-at-once sample has no slots for a staged ENDX set or a scheduled
	// clear to land on: every voice's clear applies and every set is readable
	// when the sample is complete.
	dsp->regs[DSP_ENDX] &= (UINT8)(~dsp->pending_endx_clear);
	dsp->prepared_endx &= (UINT8)(~dsp->pending_endx_clear);
	dsp->pending_endx_clear = 0;
	dsp->regs[DSP_ENDX] |= dsp->prepared_endx;
	dsp->prepared_endx = 0;
	tick_dsp_sample(dsp);
	dsp->slot_frame_left = (INT16)left;
	dsp->slot_frame_right = (INT16)right;
}

// The per-voice schedule. s3Slot is where a voice runs its whole compute body;
// s4Slot/s5Slot are where its left/right volume folds into the mix. Voice 0's
// compute sits at slot T31 and feeds the following frame's output at T0..T5: the
// one-update lag the S-DSP's envelope pipeline produces. The visibility slots: a
// voice's ENDX set becomes readable at S7, VxOUTX at S8, VxENVX at S9: three, four
// and five slots after the compute, so voice 0's land in the following sample.
//
// The slot of each voice (voices 0..7) in the C++ tables the schedule below was built from:
//   kVoiceSrcnSlot      = {18, 21,  0,  3,  6,  9, 12, 15}	the VxSRCN read
//   kVoiceDirSlot       = {22,  1,  4,  7, 10, 13, 16, 19}	the directory reads (loop pointer, start pointer, VxPITCHL, VxADSR1)
//   kVoicePitchHighSlot = {23,  2,  5,  8, 11, 14, 17, 20}	the VxPITCHH read
//   kVoiceBrrLoadSlot   = {26,  2,  5,  8, 11, 14, 17, 20}	the BRR load (prime, headers, first data bytes)
//   kVoiceS3Slot        = {31,  2,  5,  8, 11, 14, 17, 20}	the compute (voice 0's is T31's own case)
//   kVoiceS4Slot        = { 0,  3,  6,  9, 12, 15, 18, 21}	the left fold
//   kVoiceS5Slot        = { 1,  4,  7, 10, 13, 16, 19, 22}	the right fold
//   kVoiceS7Slot        = { 3,  6,  9, 12, 15, 18, 21, 24}	the ENDX publish
//   kVoiceS8Slot        = { 4,  7, 10, 13, 16, 19, 22, 25}	the OUTX publish
//   kVoiceS9Slot        = { 5,  8, 11, 14, 17, 20, 23, 26}	the ENVX publish
//
// What a voice does at a slot: its source-number read, its directory reads, its
// pitch-high read, its BRR load, its compute (voices 1-7; voice 0 computes at T31
// in the slot's own case), and its five S-steps: the left fold, the right fold,
// the ENDX publish, the OUTX publish and the ENVX publish.
enum
{
	SA_SOURCE_NUMBER,
	SA_DIRECTORY,
	SA_PITCH_HIGH,
	SA_BRR_LOAD,
	SA_COMPUTE,
	SA_FOLD_LEFT,
	SA_FOLD_RIGHT,
	SA_PUBLISH_ENDX,
	SA_PUBLISH_OUTX,
	SA_PUBLISH_ENVX
};

typedef struct
{
	UINT8 action;
	UINT8 voice;
} SLOT_STEP;

// The most actions one slot carries: T5, T8, ..., T20 each hold a voice's
// pitch-high read, BRR load and compute and another voice's ENVX publish.
#define MOST_STEPS_IN_A_SLOT	4

typedef struct
{
	UINT8 count;
	SLOT_STEP steps[MOST_STEPS_IN_A_SLOT];
} SLOT_STEPS;

// Each slot's voice actions in the order they run: every voice's source read,
// then every voice's directory reads, pitch-high read, BRR load and compute, then
// by voice the S4, S5, S7, S8 and S9 steps. A slot matches at most one voice per
// table; T27-T31 match none and run their own case alone.
static const SLOT_STEPS slot_schedule[32] = {
	{2, {{SA_SOURCE_NUMBER, 2}, {SA_FOLD_LEFT, 0}}},	// T0
	{2, {{SA_DIRECTORY, 1}, {SA_FOLD_RIGHT, 0}}},	// T1
	{3, {{SA_PITCH_HIGH, 1}, {SA_BRR_LOAD, 1}, {SA_COMPUTE, 1}}},	// T2
	{3, {{SA_SOURCE_NUMBER, 3}, {SA_PUBLISH_ENDX, 0}, {SA_FOLD_LEFT, 1}}},	// T3
	{3, {{SA_DIRECTORY, 2}, {SA_PUBLISH_OUTX, 0}, {SA_FOLD_RIGHT, 1}}},	// T4
	{4, {{SA_PITCH_HIGH, 2}, {SA_BRR_LOAD, 2}, {SA_COMPUTE, 2}, {SA_PUBLISH_ENVX, 0}}},	// T5
	{3, {{SA_SOURCE_NUMBER, 4}, {SA_PUBLISH_ENDX, 1}, {SA_FOLD_LEFT, 2}}},	// T6
	{3, {{SA_DIRECTORY, 3}, {SA_PUBLISH_OUTX, 1}, {SA_FOLD_RIGHT, 2}}},	// T7
	{4, {{SA_PITCH_HIGH, 3}, {SA_BRR_LOAD, 3}, {SA_COMPUTE, 3}, {SA_PUBLISH_ENVX, 1}}},	// T8
	{3, {{SA_SOURCE_NUMBER, 5}, {SA_PUBLISH_ENDX, 2}, {SA_FOLD_LEFT, 3}}},	// T9
	{3, {{SA_DIRECTORY, 4}, {SA_PUBLISH_OUTX, 2}, {SA_FOLD_RIGHT, 3}}},	// T10
	{4, {{SA_PITCH_HIGH, 4}, {SA_BRR_LOAD, 4}, {SA_COMPUTE, 4}, {SA_PUBLISH_ENVX, 2}}},	// T11
	{3, {{SA_SOURCE_NUMBER, 6}, {SA_PUBLISH_ENDX, 3}, {SA_FOLD_LEFT, 4}}},	// T12
	{3, {{SA_DIRECTORY, 5}, {SA_PUBLISH_OUTX, 3}, {SA_FOLD_RIGHT, 4}}},	// T13
	{4, {{SA_PITCH_HIGH, 5}, {SA_BRR_LOAD, 5}, {SA_COMPUTE, 5}, {SA_PUBLISH_ENVX, 3}}},	// T14
	{3, {{SA_SOURCE_NUMBER, 7}, {SA_PUBLISH_ENDX, 4}, {SA_FOLD_LEFT, 5}}},	// T15
	{3, {{SA_DIRECTORY, 6}, {SA_PUBLISH_OUTX, 4}, {SA_FOLD_RIGHT, 5}}},	// T16
	{4, {{SA_PITCH_HIGH, 6}, {SA_BRR_LOAD, 6}, {SA_COMPUTE, 6}, {SA_PUBLISH_ENVX, 4}}},	// T17
	{3, {{SA_SOURCE_NUMBER, 0}, {SA_PUBLISH_ENDX, 5}, {SA_FOLD_LEFT, 6}}},	// T18
	{3, {{SA_DIRECTORY, 7}, {SA_PUBLISH_OUTX, 5}, {SA_FOLD_RIGHT, 6}}},	// T19
	{4, {{SA_PITCH_HIGH, 7}, {SA_BRR_LOAD, 7}, {SA_COMPUTE, 7}, {SA_PUBLISH_ENVX, 5}}},	// T20
	{3, {{SA_SOURCE_NUMBER, 1}, {SA_PUBLISH_ENDX, 6}, {SA_FOLD_LEFT, 7}}},	// T21
	{3, {{SA_DIRECTORY, 0}, {SA_PUBLISH_OUTX, 6}, {SA_FOLD_RIGHT, 7}}},	// T22
	{2, {{SA_PITCH_HIGH, 0}, {SA_PUBLISH_ENVX, 6}}},	// T23
	{1, {{SA_PUBLISH_ENDX, 7}}},	// T24
	{1, {{SA_PUBLISH_OUTX, 7}}},	// T25
	{2, {{SA_BRR_LOAD, 0}, {SA_PUBLISH_ENVX, 7}}},	// T26
	{0, {{0, 0}}},	// T27
	{0, {{0, 0}}},	// T28
	{0, {{0, 0}}},	// T29
	{0, {{0, 0}}},	// T30
	{0, {{0, 0}}}	// T31
};

// Computes voice `voice` at its S3 slot, storing the amplitude the S4/S5 slots
// apply. voice n's pitch modulation reads voice n-1's amplitude from the
// PREVIOUS sample: the value standing before voice n-1's compute three slots
// earlier in this sample replaced it (voice 0 reads voice 7's; it does not
// modulate, so the value is inert). The voice's own standing amplitude moves to
// modulator_amplitude as it is replaced, which is what the following voice reads.
// VxOUTX and VxENVX are computed here but held back from the register file until
// the voice's S8/S9 slots: a CPU read before then sees the previous sample's
// value, and a CPU write up to two cycles before the slot outlives the slot's
// own write (see SNES_DSP.cycle_count).
static void compute_voice_slot(SNES_DSP* dsp, const UINT8* ram, unsigned int voice, int soft_reset)
{
	const int prev = dsp->modulator_amplitude[(voice + 7) & 7];
	const UINT8 held_outx = dsp->regs[voice_register(voice, VOICE_OUTX)];
	const UINT8 held_envx = dsp->regs[voice_register(voice, VOICE_ENVX)];
	SNES_DSP_VOICE* v;
	dsp->modulator_amplitude[voice] = dsp->voice_amplitude[voice];
	dsp->voice_amplitude[voice] = compute_voice_amplitude(dsp, ram, voice, prev, soft_reset);
	// The pair this sample's pitch slots read is the next advance's step: the
	// hardware advances after the output is formed, one step after the read
	// (see SNES_DSP.pitch_latch). A key-on's hold on those reads counts down
	// one compute at a time.
	v = &dsp->voices[voice];
	if (v->pitch_pending_valid)
	{
		dsp->pitch_latch[voice] = v->pitch_pending;
		v->pitch_pending_valid = 0;
	}
	if (v->pitch_capture_hold > 0) -- v->pitch_capture_hold;
	dsp->prepared_outx[voice] = dsp->regs[voice_register(voice, VOICE_OUTX)];
	dsp->prepared_envx[voice] = dsp->regs[voice_register(voice, VOICE_ENVX)];
	dsp->regs[voice_register(voice, VOICE_OUTX)] = held_outx;
	dsp->regs[voice_register(voice, VOICE_ENVX)] = held_envx;
}

// Finalizes the left output (slot T27): the dry mix scaled by MVOLL, the echo FIR
// output added through EVOLL, and the mute gate: the same arithmetic the
// frame-at-once path runs, split by channel so MVOLL is consumed one slot before
// MVOLR. Both volume multiplies are 16 bits wide and wrap; only the add onto the
// dry mix saturates.
static void finalize_left(SNES_DSP* dsp)
{
	const int mvol = (INT8)dsp->regs[DSP_MVOL_LEFT];
	const int evol = (INT8)dsp->regs[DSP_EVOL_LEFT];
	INT32 out = (INT16)((dsp->mix_left * mvol) >> 7);
	out = clamp_signed16(out + (INT16)((dsp->echo_fir_out_left * evol) >> 7));
	if ((dsp->regs[DSP_FLG] & FLG_MUTE) != 0) out = 0;
	dsp->slot_frame_left = (INT16)out;
}

// Finalizes the right output (slot T28): MVOLR, EVOLR and the mute gate.
static void finalize_right(SNES_DSP* dsp)
{
	const int mvol = (INT8)dsp->regs[DSP_MVOL_RIGHT];
	const int evol = (INT8)dsp->regs[DSP_EVOL_RIGHT];
	INT32 out = (INT16)((dsp->mix_right * mvol) >> 7);
	out = clamp_signed16(out + (INT16)((dsp->echo_fir_out_right * evol) >> 7));
	if ((dsp->regs[DSP_FLG] & FLG_MUTE) != 0) out = 0;
	dsp->slot_frame_right = (INT16)out;
}

// Runs one 32-clock slot of a primed sample. Each register read lands on its
// documented slot, so a mid-frame DSPDATA write is seen only if it precedes its
// consuming slot; every no-write sample reproduces the frame-at-once output for
// voices 1-7, and voice 0 rides one update behind. The slot's voice actions are
// taken from slot_schedule, which lists for each slot the actions the tables
// place there, in the order they run.
static void run_primed_slot(SNES_DSP* dsp, UINT8* ram, UINT8 slot)
{
	const int soft_reset = (dsp->regs[DSP_FLG] & FLG_SOFT_RESET) != 0;
	const SLOT_STEPS* steps;
	unsigned int n;

	if (slot == 0)
	{
		unsigned int v;
		// Frame start: clear the mix the previous frame delivered, then apply voice 0's
		// amplitude, prepared at the last frame's T31 (the pipeline lag). Take a
		// key-on's scheduled pitch capture (see SNES_DSP.pitch_latch); a live
		// voice's pair is read at its own pitch slots later in the sample.
		dsp->mix_left = 0;
		dsp->mix_right = 0;
		dsp->echo_send_left = 0;
		dsp->echo_send_right = 0;
		for (v = 0; v < 8; ++ v)
		{
			const UINT8 bit = (UINT8)(1u << v);
			if ((dsp->pitch_reload_pending & bit) != 0 && dsp->sample_index % 2 == 0)
			{
				// The capture a consumed key-on scheduled: on the poll-parity sample
				// following the poll, read the register pair (voice 0's compute takes
				// it this sample, voices 1-7's the next; their pending pair lands
				// once this sample's compute has run, as a slot read's does).
				const UINT16 pair = (UINT16)voice_pitch(dsp, v);
				if (v == 0)
				{
					dsp->pitch_latch[v] = pair;
				}
				else
				{
					dsp->voices[v].pitch_pending = pair;
					dsp->voices[v].pitch_pending_valid = 1;
				}
				dsp->pitch_reload_pending = (UINT8)(dsp->pitch_reload_pending & ~bit);
			}
		}
	}

	// The voice actions the tables place at this slot, in the order they run.
	// Each of the three register publishes (S7, S8, S9) loses to a CPU write
	// issued in the two cycles before it (see SNES_DSP.cycle_count): the register
	// keeps the CPU's byte and the DSP's value for the slot is dropped.
	steps = &slot_schedule[slot];
	for (n = 0; n < steps->count; ++ n)
	{
		const unsigned int voice = steps->steps[n].voice;
		switch(steps->steps[n].action)
		{
		case SA_SOURCE_NUMBER:
			// The source read at the voice's source slot: the `VxSRCN` the directory
			// reads that follow select the entry with.
			load_source_number(dsp, voice);
			break;
		case SA_DIRECTORY:
			// The directory read at the voice's directory slot: the loop address the
			// compute's loop jump takes, read ahead of it, and, for a voice whose
			// key-on the previous compute applied, the entry's start pointer. The
			// `VxPITCHL` and `VxADSR1` registers are read in the same slot.
			if (dsp->voices[voice].start_pending) load_start_pointer(dsp, ram, voice);
			load_loop_pointer(dsp, ram, voice);
			load_directory_slot_registers(dsp, voice);
			break;
		case SA_PITCH_HIGH:
			// The `VxPITCHH` read at the voice's pitch-high slot, one after the
			// directory slot, completing the step the compute's advance takes.
			load_pitch_high(dsp, voice);
			break;
		case SA_BRR_LOAD:
			// The BRR load at the voice's load slot: a keyed voice's prime, then the
			// header and first data byte of the group decodes the voice's compute
			// will perform, read ahead of it.
			if (dsp->voices[voice].start_pending)
			{
				prime_voice_stream(dsp, ram, voice);
				dsp->voices[voice].start_pending = 0;
			}
			load_scheduled_group_bytes(&dsp->voices[voice], ram);
			break;
		case SA_COMPUTE:
			// Voice compute at the voice's S3 slot (voice 0's is T31, in the slot's
			// own case below).
			compute_voice_slot(dsp, ram, voice, soft_reset);
			break;
		case SA_FOLD_LEFT:
			// The left volume fold at the voice's S4 slot.
			perform_loaded_decodes(dsp, ram, voice);
			apply_voice_left(dsp, voice);
			break;
		case SA_FOLD_RIGHT:
			// The right volume fold at the voice's S5 slot.
			apply_voice_right(dsp, voice);
			break;
		case SA_PUBLISH_ENDX:
		{
			// The voice's ENDX bit is written at the S7 slot: a key-on's clear, or
			// else a staged set.
			const UINT8 bit = (UINT8)(1u << voice);
			if ((dsp->pending_endx_clear & bit) != 0)
			{
				dsp->regs[DSP_ENDX] &= (UINT8)(~bit);
				dsp->pending_endx_clear = (UINT8)(dsp->pending_endx_clear & ~bit);
				dsp->prepared_endx = (UINT8)(dsp->prepared_endx & ~bit);
			}
			else if ((dsp->prepared_endx & bit) != 0)
			{
				if (! cpu_write_stands(dsp, dsp->endx_write_cycle)) dsp->regs[DSP_ENDX] |= bit;
				dsp->prepared_endx = (UINT8)(dsp->prepared_endx & ~bit);
			}
			break;
		}
		case SA_PUBLISH_OUTX:
			// The computed OUTX byte becomes readable at the S8 slot.
			if (! cpu_write_stands(dsp, dsp->outx_write_cycle[voice]))
				dsp->regs[voice_register(voice, VOICE_OUTX)] = dsp->prepared_outx[voice];
			break;
		case SA_PUBLISH_ENVX:
			// ENVX carries one further pipeline stage: its S9 slot writes the value
			// computed one sample earlier and stages the fresh one (see
			// SNES_DSP.envx_stage).
			if (! cpu_write_stands(dsp, dsp->envx_write_cycle[voice]))
				dsp->regs[voice_register(voice, VOICE_ENVX)] = dsp->envx_stage[voice];
			dsp->envx_stage[voice] = dsp->prepared_envx[voice];
			break;
		}
	}

	switch(slot)
	{
	case 23:
		// The echo unit's ladder opens: the left channel's word leaves the ring,
		// addressing the APPLIED base, the ESA the previous sample's T31 applied,
		// and FIR0 is captured with it. The EON sends are all folded by T22, so
		// nothing the voices owe the echo is still outstanding.
		load_echo_left(dsp, ram, dsp->echo_applied_esa);
		break;
	case 24:
		// The right channel's word, and FIR1 and FIR2.
		load_echo_right(dsp, ram);
		break;
	case 25:
		// FIR3, FIR4 and FIR5.
		latch_fir_coefficients(dsp, 3, 3);
		break;
	case 26:
		// FIR6 and FIR7 complete the set, and the filter runs over the eight
		// captured coefficients, leaving each channel's output for the slots below.
		filter_echo(dsp);
		break;
	case 27:
		finalize_left(dsp);
		// EFB is read here, and the feedback it scales is staged for the write
		// slots.
		stage_echo_write(dsp, dsp->echo_send_left, dsp->echo_send_right);
		break;
	case 28:
		finalize_right(dsp);
		// PMON is read here, for every voice at once.
		load_pitch_modulation(dsp);
		break;
	case 29:
		// The write gate for T30's left word: FLG bit 5 is loaded one slot before
		// the word it governs.
		dsp->echo_gate_left = (dsp->regs[DSP_FLG] & FLG_ECHO_WRITE_DISABLE) == 0;
		// NON, EON and DIR are read here, in one act, for every voice at once.
		load_voice_wide_registers(dsp);
		break;
	case 30:
		// The left echo word lands at its write slot, T30, under the gate loaded at
		// T29: the value was formed at T27, where EFB is read, and held since.
		if (dsp->echo_write_pending && dsp->echo_gate_left)
		{
			ram[dsp->echo_write_entry] = dsp->echo_write_bytes[0];
			ram[(UINT16)(dsp->echo_write_entry + 1)] = dsp->echo_write_bytes[1];
		}
		// Load the raw ESA/EDL for T31's ring advance to apply, and the right
		// word's own FLG bit-5 gate.
		dsp->echo_latched_esa = dsp->regs[DSP_ESA];
		dsp->echo_latched_edl = dsp->regs[DSP_EDL];
		dsp->echo_gate_right = (dsp->regs[DSP_FLG] & FLG_ECHO_WRITE_DISABLE) == 0;
		// The internal key-on loses the bits the previous poll took, one slot
		// before this sample's own poll reads it, and a KON write issued in the
		// two cycles before this slot goes with them.
		if (dsp->sample_index % 2 == 0 && cpu_write_stands(dsp, dsp->kon_write_cycle))
			dsp->internal_kon = (UINT8)(dsp->internal_kon & ~dsp->consumed_kon);
		break;
	case 31:
		// The KON/KOFF load runs first (the poll keeps the even-sample parity:
		// it reads the pre-tick sample index, as the frame-at-once entry did),
		// then the global counter advances, and only then does voice 0 compute:
		// its envelope check, and the noise step below, read the value this
		// slot just produced, the same value voices 1-7 read at their compute
		// slots in the following frame. So all eight voices' checks between two
		// advances see one counter value (measured against spc_dsp6
		// `Misc/counter rate synchronizations`, which locks the counter phase
		// through a voice-4 gadget and measures voice 0's time-to-first-step at
		// every rate). A keyed voice 0 takes this very slot as the first of its
		// five silent startup calls, which is what makes the eight voices'
		// key-on startup read-uniform through VxENVX (spc_dsp6 `KON/envx during
		// kon`).
		poll_keying(dsp);
		tick_dsp_sample(dsp);
		// The noise step precedes voice 0's compute as well: the rate in FLG
		// bits 0-4 is read here and the level advances before voice 0 takes it,
		// so all eight voices' amplitudes for one delivered sample carry the
		// same noise level: voice 0's from this slot, voices 1-7's from their
		// compute slots in the frame that follows. A voice 0 that read the
		// pre-step level would trail the others by one step (spc_dsp6
		// `Order/noise rate flg.1F` spins on the echo tape until voice 0's noise
		// sample reads back as zero, then stops the rate: the level it leaves
		// standing, and the frame the next stepped level reaches the tape on,
		// are both one sample earlier than a trailing voice 0 produces).
		if (envelope_rate_fires(dsp->global_counter, dsp->regs[DSP_FLG] & FLG_NOISE_RATE))
			dsp->noise_level = next_noise_level(dsp->noise_level);
		compute_voice_slot(dsp, ram, 0, soft_reset);
		// The right echo word lands at its write slot, T31, under the gate loaded
		// at T30; then the ring advance applies the T30-loaded ESA/EDL.
		if (dsp->echo_write_pending && dsp->echo_gate_right)
		{
			ram[(UINT16)(dsp->echo_write_entry + 2)] = dsp->echo_write_bytes[2];
			ram[(UINT16)(dsp->echo_write_entry + 3)] = dsp->echo_write_bytes[3];
		}
		dsp->echo_write_pending = 0;
		advance_echo_ring(dsp, dsp->echo_latched_esa, dsp->echo_latched_edl);
		break;
	default:
		break;
	}
}

// Runs one slot. An unprimed state runs its whole first sample at the wrap slot
// T31, the same cycle the frame-at-once model delivered on, so a write during the
// first sample's slots reaches it exactly as before, then primes the schedule for
// voice 0; every later sample is slot-scheduled. Slot T0 (the wrap) delivers the
// frame the previous sample finalized.
int snes_dsp_step_cycle(SNES_DSP* dsp, UINT8* ram)
{
	const UINT8 slot = dsp->slot_cursor;
	++ dsp->cycle_count;	// the clock a CPU write's stamp is measured against

	if (! dsp->primed)
	{
		if (slot == 31)
		{
			// The frame-at-once path advances voice 0 exactly once and records its
			// amplitude in voice_amplitude[0]; the next (slot-scheduled) frame applies
			// that value at T0..T5, so voice 0's output rides one frame behind while its
			// state trajectory stays the frame-at-once one.
			step_dsp_sample_atomic(dsp, ram);
			dsp->primed = 1;
		}
	}
	else
	{
		run_primed_slot(dsp, ram, slot);
	}

	dsp->slot_cursor = (UINT8)((slot + 1) & 31);
	return dsp->slot_cursor == 0;	// the sample's 32 slots are complete
}

// The key-on's restart. `prime_now` reads the start pointer and primes the
// stream at once (a state without a slot schedule); the slot-scheduled restart
// (compute_voice_amplitude's restart_pending branch) leaves the voice start_pending,
// and the next sample's directory and load slots read the pointer and prime.
static void key_on_voice_impl(SNES_DSP* dsp, const UINT8* ram, unsigned int voice, int prime_now)
{
	// Whether the startup walks is decided by the voice the key-on lands on, as
	// the restart applies: it walks only when it interrupts a voice whose level
	// stands above zero while its old OUTPUT is silent. A re-key of a sounding
	// voice holds its stream, whatever its age or level, and so does a key-on
	// at level zero (see the voice's startup_walks). Which sample the silence is
	// read from (this compute's predecessor here, through voice_amplitude) and
	// whether the test sees the full amplitude or its OUTX byte is constrained
	// by nothing that passes; the two sub-tests sensitive to the walk fail
	// under every reading (docs/s-dsp-behavior.md, "What remains open").
	const int walks = dsp->voices[voice].envelope != 0 && dsp->voice_amplitude[voice] == 0;
	SNES_DSP_VOICE* v;
	if (prime_now)
	{
		start_voice(dsp, ram, voice);	// reads the pointer, primes, resets the voice state
	}
	else
	{
		reset_voice_for_start(dsp, voice);
		dsp->voices[voice].start_pending = 1;
	}
	v = &dsp->voices[voice];
	v->startup_walks = (UINT8)walks;
	v->phase = SNES_DSP_PHASE_ATTACK;
	// The startup countdown: the documented five empty samples, uniform for every
	// voice. The poll precedes voice 0's compute in the slot they share, so voice
	// 0's first silent call is the load's own slot and voices 1-7's follow in the
	// next samples; the count of five plus that shared-slot asymmetry is exactly
	// what makes the eight voices' startup read-uniform through VxENVX
	// (spc_dsp6 `KON/envx during kon`) while keeping voice 0's first live compute
	// at the instant `Envelope/hidden env 0 at kon` pins.
	v->kon_delay = KEY_ON_STARTUP_CALLS;
	v->computes_since_key_on = 0;
	// Key-on clears this voice's ENDX bit, staged or readable: a same-sample
	// end-block set does not override the clear. On the slot schedule the clear
	// is written at the voice's S7 slot, four slots after this compute, like the
	// set it can erase (see SNES_DSP.pending_endx_clear); the direct call and the
	// frame-at-once path clear at once.
	if (prime_now)
	{
		dsp->regs[DSP_ENDX] &= (UINT8)(~(1u << voice));
		dsp->prepared_endx &= (UINT8)(~(1u << voice));
	}
	else
	{
		dsp->pending_endx_clear |= (UINT8)(1u << voice);
	}
}

void snes_dsp_cpu_write_register(SNES_DSP* dsp, UINT8 reg, UINT8 value)
{
	unsigned int voice;
	UINT8 offset;

	if (reg > 0x7F) return;	// DSPDATA writes beyond $7F are ignored
	if (reg == DSP_ENDX)
	{
		// ENDX: any write acknowledges all end flags. A set still staged for its
		// voice's S7 slot is left to land there, unless this write is what the
		// slot's write then loses to.
		dsp->regs[DSP_ENDX] = 0;
		dsp->endx_write_cycle = dsp->cycle_count;
		return;
	}
	dsp->regs[reg] = value;
	// A KON write also arms the internal key-on the poll consumes. The value
	// replaces whatever was pending, so of two writes between polls only the
	// second one keys anything on. The write carries its cycle: the T30 strike
	// and the poll each ask whether it landed in the two cycles before them.
	if (reg == DSP_KON)
	{
		dsp->internal_kon = value;
		dsp->kon_write_cycle = dsp->cycle_count;
	}
	// The DSP-written voice registers carry the write's cycle, for the S8/S9
	// write that may have to yield to it.
	voice = reg >> 4;
	offset = (UINT8)(reg & 0x0F);
	if (offset == VOICE_OUTX) dsp->outx_write_cycle[voice] = dsp->cycle_count;
	if (offset == VOICE_ENVX) dsp->envx_write_cycle[voice] = dsp->cycle_count;
}

// Keys voice `voice` (0-7) off: the envelope enters Release, decreasing by 8 each
// sample regardless of the ADSR/GAIN settings. The stream is untouched: BRR
// decoding never stops for a keyed-off voice.
INLINE void key_off_voice(SNES_DSP* dsp, unsigned int voice)
{
	dsp->voices[voice].phase = SNES_DSP_PHASE_RELEASE;
}

// Runs the KON/KOFF poll. On even-indexed samples it reads the KON ($4C) and
// KOFF ($5C) registers: a set KON bit latches internal-KON and keys the voice on
// (envelope 0, Attack, the 5-sample startup, its ENDX bit cleared, its stream
// primed), and a set KOFF bit moves the voice to Release. On odd samples it does
// nothing. Voices keyed on two samples ago clear from the internal-KON latch. A
// key-on landing on a voice still inside its silent key-on span (the five
// startup calls plus the first live compute) is absorbed at the poll right
// after the keying (countdown, stream and envelope schedule all stand), and at
// any later in-span poll re-arms the countdown and drops the envelope while the
// stream keeps the position it has walked to.
static void poll_keying(SNES_DSP* dsp)
{
	UINT8 kon;
	UINT8 koff;
	unsigned int voice;

	// The poll itself touches no RAM: a consumed restart's stream re-prime (the
	// one RAM-reading key-on action) runs at the voice's own compute (see
	// compute_voice_amplitude's restart_pending branch).
	// The poll runs on even-indexed samples counted from power-on: a fixed
	// choice that settles the hardware's probabilistic power-on poll phase.
	if (dsp->sample_index % 2 != 0) return;

	// KOFF is read from the register, which exerts its influence at every poll
	// for as long as the bit stays set. KON is read from the internal value a
	// write arms, and the poll clears it, so a key-on happens once per write,
	// however long the register keeps the bit.
	kon = dsp->internal_kon;
	koff = dsp->regs[DSP_KOFF];
	for (voice = 0; voice < 8; ++ voice)
	{
		const UINT8 bit = (UINT8)(1u << voice);
		if ((koff & bit) != 0) key_off_voice(dsp, voice);
		// KON is applied after KOFF, so a voice with both bits set keys on. A
		// key-on landing on a voice still inside its silent key-on span (the five
		// startup calls plus the first live compute, whose output is still silent)
		// splits by which poll takes delivery of it. At the poll immediately after
		// the one that keyed the voice, it is absorbed outright: countdown, stream
		// and envelope schedule all stand (`KON/kon clears independent`). At any
		// later poll inside the span, the stream still stands (the cursor keeps
		// walking at the pitch, un-re-primed (`KON/kon decoding when another kon`))
		// but the silent hold rewinds: the countdown reloads and the envelope
		// drops to 0, so the level emerges as late as a full restart would place
		// it (`KON/kon then another kon`). A re-key past the span restarts the
		// voice in full (the documented click/pop case).
		//
		// Absorption covers a write that was standing before the poll's slot came
		// round. One issued in the two cycles before the poll rewinds the hold
		// instead, at the poll that takes it: `Timing/Misc/29 kon cleared`'s T30
		// row reads its voices reaching the tape later than the rows whose write
		// the strike took, and earlier than the rows the next poll serves.
		//
		// Both in-span tiers protect a STANDING startup. A voice in Release has no
		// startup left to absorb into or rewind: a soft reset landing inside the
		// span keys the voice off and kills the startup for good, so a key-on
		// consumed on a Released voice restarts it in full even inside the span
		// (`KON/kon then flg.80 then kon`). The same arm is what lets KON win over
		// KOFF when one poll delivers both to an in-span voice.
		//
		// The full restart arms its countdown and counter here, but the wipe waits
		// for the voice's own compute: that compute still prepares one sample from
		// the standing stream and envelope (the final pre-key-on sample, which
		// still sounds) before the restart applies (`KON/kon unaffected by
		// pitch`: a re-key on a sounding voice emits the old data once more on the
		// consuming sample, at every pitch; Anomie's key-on account places the
		// final pre-KON decode on that same sample).
		if ((kon & bit) != 0)
		{
			SNES_DSP_VOICE* v = &dsp->voices[voice];
			if (v->computes_since_key_on > KEY_ON_SILENT_CALLS || v->phase == SNES_DSP_PHASE_RELEASE)
			{
				v->kon_delay = KEY_ON_STARTUP_CALLS;
				v->computes_since_key_on = 0;
				v->restart_pending = 1;
			}
			else if (v->computes_since_key_on > 2 || cpu_write_stands(dsp, dsp->kon_write_cycle))
			{
				v->kon_delay = KEY_ON_STARTUP_CALLS;
				v->envelope = 0;
			}
			// Every consumed key-on (restarting, rewinding, or absorbed) schedules
			// a pitch capture for the voice at the next poll-parity sample's start.
			// That is the only path a pitch write reaches a voice through while the
			// key-on's capture hold stands (`KON/kon decoding when another kon`'s
			// freezes ride their key-on; `KON/kon then change pitch`'s bare writes
			// do not), and its timing is pinned from both sides: a register write
			// nine cycles behind the KON write must be seen (the freezes), while one
			// landing three cycles after the parity sample's first slot must not be
			// (that ROM's earliest pulse). The hold itself is counted in the voice's
			// own computes (the six of its silent span) so the seventh compute's
			// slot reads are the first live ones and the eighth compute the first
			// advance at a live pitch, uniform for the eight voices: voice 0's
			// first compute follows this poll in the slot they share, the others'
			// come in the next sample.
			// A consumed key-on clears the voice's ENDX bit: restarting the voice,
			// rewinding a standing startup, or absorbed into one alike. Both
			// references state the clear for every key-on the poll takes, without
			// qualification. It is written at the voice's S7 slot, where a
			// same-sample end-block set cannot override it (see
			// SNES_DSP.pending_endx_clear). No spc_dsp6 row that passes distinguishes
			// this from clearing on a restart alone; `Random/kon pitch` is the one
			// row that moves, and it fails either way.
			dsp->pending_endx_clear |= bit;
			dsp->pitch_reload_pending |= bit;
			dsp->voices[voice].pitch_capture_hold = KEY_ON_SILENT_CALLS;
		}
	}
	// The poll disarms itself, and records what it took for the next sample's
	// strike to remove from a value written since.
	dsp->consumed_kon = kon;
	dsp->internal_kon = 0;
}

// Advances voice `voice`'s envelope by one 32 kHz sample and writes ENVX ($x8,
// the level's high 7 bits). During the post-key-on startup the level holds at 0
// and the countdown decrements. Otherwise one envelope operation is applied,
// gated by the global counter and shaped by the ADSR/GAIN registers: Attack
// (+32, or +1024 at attack rate 15) to Decay when the level exceeds 0x7FF, Decay
// and Sustain as exponential decreases, the four GAIN modes, and Direct Gain.
// `brr_end_mute` (a BRR End+Mute block entered this sample) forces Release and
// drops the level to 0.
static void step_voice_envelope(SNES_DSP* dsp, unsigned int voice, int brr_end_mute)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];

	// A BRR End+Mute block standing at the sample's start moves the voice to
	// Release and drops the level to 0 before any envelope operation runs (the
	// documented -0x800 step reaches 0 from any level), so the kill sample
	// publishes 0 (spc_dsp6 `KON/kon then set sample's end flag`: an End+Mute
	// pulse landing on the first live step reads as a voice that never sounded).
	// It is read before the startup countdown below, because the header check
	// that raises it goes live while the countdown still has samples left to run.
	if (brr_end_mute)
	{
		v->phase = SNES_DSP_PHASE_RELEASE;
		v->envelope = 0;
		v->bent_gain_ref = 0;
		if (v->kon_delay > 0) -- v->kon_delay;
		dsp->regs[voice_register(voice, VOICE_ENVX)] = (UINT8)(v->envelope >> 4);
		return;
	}

	// Post-key-on startup: the level holds at 0 (set at key-on) while the
	// countdown runs. The call that takes the counter to 0 is itself silent, so a
	// fresh key-on yields five silent calls before the first live step (see
	// key_on_voice_impl for the count).
	if (v->kon_delay > 0)
	{
		-- v->kon_delay;
		dsp->regs[voice_register(voice, VOICE_ENVX)] = (UINT8)(v->envelope >> 4);
		return;
	}

	run_envelope_mode(dsp, voice);
	dsp->regs[voice_register(voice, VOICE_ENVX)] = (UINT8)(v->envelope >> 4);
}

// Runs the selected envelope mode once for voice `voice`: the candidate level,
// the rate counter's decision, the phase switches and the Bent-Increase
// reference. The countdown and End+Mute cases around it are step_voice_envelope's;
// a consumed restart runs this alone to settle the standing envelope before the
// restart reads it.
static void run_envelope_mode(SNES_DSP* dsp, unsigned int voice)
{
	SNES_DSP_VOICE* v = &dsp->voices[voice];
	// `VxADSR1` is the directory slot's read (see SNES_DSP_VOICE.adsr1); `VxADSR2`
	// and `VxGAIN` are read here, at the compute.
	const UINT8 adsr1 = v->adsr1_loaded ? v->adsr1 : dsp->regs[voice_register(voice, VOICE_ADSR1)];
	const UINT8 adsr2 = dsp->regs[voice_register(voice, VOICE_ADSR2)];
	const UINT8 gain = dsp->regs[voice_register(voice, VOICE_GAIN)];
	const int level = v->envelope;
	int sustain_boundary;
	int attack_to_decay;
	int decay_to_sustain;
	int already_sustaining;

	// The selected mode computes a candidate level every sample. The rate counter
	// decides only whether the envelope takes that candidate; the Attack->Decay
	// switch below reads it either way, which is what lets a voice change phase
	// while a rate of 0 holds its level still.
	int new_level = level;
	int fires = 0;

	if (v->phase == SNES_DSP_PHASE_RELEASE)
	{
		fires = 1;	// Release runs every sample (rate 31)
		new_level = level - 8;
	}
	else if ((adsr1 & 0x80) != 0)
	{
		switch(v->phase)
		{
		case SNES_DSP_PHASE_ATTACK:
		{
			const UINT8 rate = (UINT8)((adsr1 & 0x0F) * 2 + 1);
			fires = envelope_rate_fires(dsp->global_counter, rate);
			new_level = level + ((adsr1 & 0x0F) == 0x0F ? 1024 : 32);
			break;
		}
		case SNES_DSP_PHASE_DECAY:
		{
			const UINT8 rate = (UINT8)(((adsr1 >> 4) & 0x07) * 2 + 16);
			fires = envelope_rate_fires(dsp->global_counter, rate);
			new_level = exp_decrease(level);
			break;
		}
		case SNES_DSP_PHASE_SUSTAIN:
		{
			fires = envelope_rate_fires(dsp->global_counter, (UINT8)(adsr2 & 0x1F));
			new_level = exp_decrease(level);
			break;
		}
		case SNES_DSP_PHASE_RELEASE:
			break;	// handled above
		}
	}
	else if ((gain & 0x80) == 0)
	{
		fires = 1;	// Direct Gain: a fixed level, no rate
		new_level = (gain & 0x7F) << 4;
	}
	else
	{
		fires = envelope_rate_fires(dsp->global_counter, (UINT8)(gain & 0x1F));
		switch((gain >> 5) & 0x03)
		{
		case 0: new_level = level - 32; break;			// Linear Decrease
		case 1: new_level = exp_decrease(level); break;	// Exp Decrease
		case 2: new_level = level + 32; break;			// Linear Increase
		default:										// Bent Increase
			new_level = level + (v->bent_gain_ref < 0x600 ? 32 : 8);
			break;
		}
	}

	// The Attack->Decay switch fires when the candidate leaves the unsigned 11-bit
	// range: an attack overshooting 0x7FF, or a decreasing mode driven below zero.
	// The level itself is clamped, so the overshoot is only visible here.
	attack_to_decay = (v->phase == SNES_DSP_PHASE_ATTACK) && ((new_level & ~0x7FF) != 0);

	// The boundary Decay->Sustain compares against comes from whichever register
	// is driving the envelope: ADSR2's sustain level under ADSR, and GAIN's top
	// three bits while a GAIN mode drives the level. Those bits are the gain
	// mode and its enable, not a sustain level, so a voice keyed on under GAIN
	// reaches Sustain at a boundary its author never chose.
	sustain_boundary =
		(adsr1 & 0x80) != 0 ? ((adsr2 >> 5) & 0x07) : ((gain >> 5) & 0x07);

	// Decay->Sustain reads the candidate, and reads it every sample: a voice
	// parked in a rate-0 mode never takes the value, yet still changes phase when
	// the value the mode computes reaches the boundary. A candidate outside the
	// 11-bit range is the Attack->Decay case above and matches no boundary.
	decay_to_sustain = (v->phase == SNES_DSP_PHASE_DECAY) && ((new_level & ~0x7FF) == 0) &&
						((new_level >> 8) == sustain_boundary);

	// Under ADSR, a decay whose stored level already sits in the sustain band is
	// in Sustain before its step: the phase changes and the level holds, whatever
	// the decay rate does this sample. spc_dsp6 `Random/voice volumes` keys eight
	// voices at ADSR $EF/$E0 (the attack ends at 0x7FF, sustain level 7, sustain
	// rate 0, decay rate 28 firing one sample in four) from two arrival phases
	// two samples apart, and its tape carries no -8 step from either; a store
	// gated by the decay rate steps on one of them. The stored-level check is
	// ADSR's alone: under a GAIN mode the ROM's `Envelope/attack->decay during
	// gain` reads the candidate, not the stored level. Which rate gates that one
	// store is unconstrained: gating it by the sustain rate instead leaves every
	// passing sub-test green and every assertion here green, and the single
	// sub-test sensitive to the choice is reproduced by neither reading; gating it
	// by both rates at once is byte-identical to suppressing it. Suppressing it
	// keeps the documented step-then-check order everywhere the level is still
	// above the band.
	already_sustaining = (v->phase == SNES_DSP_PHASE_DECAY) && (adsr1 & 0x80) != 0 &&
							(level >> 8) == sustain_boundary;

	// The counter decides only whether the level takes the candidate.
	if (fires && ! already_sustaining) v->envelope = clamp_envelope(new_level);

	// The Bent-Increase reference is the value the mode computes, saved every
	// sample whether or not the counter fires, so a voice parked in a rate-0
	// mode still hands the next Bent-Increase step that mode's own value rather
	// than the level standing still. It is saved unclipped: a candidate driven
	// below zero and one carried past 0x7FF both read at or past 0x600 and take
	// the +8 branch.
	v->bent_gain_ref = (UINT16)new_level;
	if (decay_to_sustain || already_sustaining) v->phase = SNES_DSP_PHASE_SUSTAIN;
	if (attack_to_decay) v->phase = SNES_DSP_PHASE_DECAY;
}

void snes_dsp_init(SNES_DSP* dsp)
{
	unsigned int n;

	memset(dsp, 0x00, sizeof(SNES_DSP));
	for (n = 0; n < 8; ++ n)
	{
		dsp->outx_write_cycle[n] = SNES_DSP_NO_CPU_WRITE;
		dsp->envx_write_cycle[n] = SNES_DSP_NO_CPU_WRITE;
		voice_default(&dsp->voices[n]);
	}
	dsp->endx_write_cycle = SNES_DSP_NO_CPU_WRITE;
	dsp->kon_write_cycle = SNES_DSP_NO_CPU_WRITE;
	dsp->noise_level = -0x4000;
}

