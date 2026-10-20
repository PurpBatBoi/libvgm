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

#ifndef __SNES_DSP_H__
#define __SNES_DSP_H__

#include "../../stdtype.h"

#ifdef __cplusplus
extern "C" {
#endif

// The S-DSP, the sound chip the SPC700 drives through the DSPADDR/DSPDATA
// registers (C port of Snaggletooth's include/snaggletooth/apu/dsp.h and
// src/dsp.cpp, pinned at commit efd5049; see snes_dsp.c for the port notes).
//
// The state is a value: the 128-byte register file the CPU reaches through
// DSPADDR/DSPDATA lives here (the APU carries only the DSPADDR latch beside it),
// with the envelope counter, the noise generator, the echo unit and the voices'
// streaming state. The comments on the fields are the C++ comments.

// The volume envelope's phase. Even in a GAIN mode the hardware tracks an
// Attack/Decay/Sustain phase, but that phantom tracking has no audible effect
// until a mid-note switch to ADSR mode, so only the ADSR path drives phase
// transitions here. A voice powers on in Release (envelope 0), matching the
// post-reset state where FLG holds every voice keyed off.
#define SNES_DSP_PHASE_ATTACK	0
#define SNES_DSP_PHASE_DECAY	1
#define SNES_DSP_PHASE_SUSTAIN	2
#define SNES_DSP_PHASE_RELEASE	3

// The four most recently decoded samples of a voice's BRR stream: the window
// the Gaussian kernel interpolates over (the hardware references name the taps
// new/old/older/oldest).
typedef struct snes_dsp_window
{
	INT16 newest;
	INT16 old;
	INT16 older;
	INT16 oldest;
} SNES_DSP_WINDOW;

// One group decode the cursor's advances have scheduled, naming the block the
// group lies in and its first in-block sample index (see the voice's
// scheduled_decodes). bytes_loaded is clear for a decode the slot schedule has not
// loaded, which reads the data bytes at the decode.
typedef struct snes_dsp_group_decode
{
	UINT16 address;
	UINT8 offset;
	UINT8 header;
	UINT8 first_byte;
	UINT8 bytes_loaded;
	// Samples of the group already decoded: the compute decodes the first two
	// (the loaded first byte's) and the voice's S4 slot the last two, reading
	// the second data byte from RAM there, each half ahead of any sample the
	// cursor can reach.
	UINT8 decoded_samples;
	// Set when the header was read at the group's scheduling itself (the
	// trigger-2 form): the load slot then supplies only the first data byte.
	UINT8 header_captured;
} SNES_DSP_GROUP_DECODE;

// A voice's streaming position through its BRR data. The pitch counter's bits
// 15-12 are the sample position within the current block, bits 11-4 the
// Gaussian interpolation index, and the low bits accumulate the fractional
// step. The decode cursor runs three samples ahead of the counter's position,
// so the window's `newest` at position p is stream sample p+3. brr_sample_index
// is the next sample the cursor decodes within its block; 16 means the block is
// exhausted and the next decode performs the block transition (chaining to the
// next block, or jumping to the loop address after an end block) before decoding.
// The remaining fields follow the C++ VoiceState (include/snaggletooth/apu/dsp.h),
// where each one is documented at length.
typedef struct snes_dsp_voice
{
	UINT16 brr_address;
	UINT16 decoder_address;		// the block the DECODER is in
	UINT16 header_address;		// the block whose header the per-sample end/loop check reads
	UINT8 brr_sample_index;
	UINT16 pitch_counter;
	SNES_DSP_WINDOW window;
	INT16 pending[12];			// the decoded-sample ring between the decoder and the window
	UINT8 pending_head;
	UINT8 pending_count;
	SNES_DSP_GROUP_DECODE scheduled_decodes[3];
	UINT8 scheduled_decode_count;
	UINT8 loaded_header;		// the header the BRR load slot read for this sample's end/loop check
	UINT8 header_loaded;
	UINT16 loop_pointer;		// the loop address the directory slot read this sample
	UINT8 loop_pointer_loaded;
	UINT8 srcn;					// the VxSRCN value the source slot read this sample
	UINT8 srcn_loaded;
	UINT8 pitch_low;			// the VxPITCHL byte the directory slot read this sample
	UINT16 pitch_pending;		// the pair the pitch slots read this sample
	UINT8 pitch_pending_valid;
	UINT8 adsr1;				// the VxADSR1 the directory slot read this sample
	UINT8 adsr1_loaded;
	INT16 decode_prev1;			// the decoder's filter history
	INT16 decode_prev2;
	UINT16 envelope;			// the 11-bit level (0..0x7FF)
	UINT8 phase;				// SNES_DSP_PHASE_*
	UINT8 kon_delay;			// the 5-sample post-key-on startup countdown
	UINT8 computes_since_key_on;	// saturating high; 0xFF after a reset
	UINT8 pitch_capture_hold;
	UINT8 restart_pending;
	UINT8 start_pending;
	UINT8 startup_walks;
	UINT16 bent_gain_ref;
} SNES_DSP_VOICE;

typedef struct snes_dsp
{
	UINT8 regs[128];

	// The global counter that gates every envelope/noise rate. It powers on at 0,
	// decrements once per 32 kHz sample, and wraps to 0x77FF when it would pass
	// below 0 (the first sample after reset wraps).
	UINT16 global_counter;

	// DSP samples elapsed since power-on. Only its parity is load-bearing: the
	// KON/KOFF poll runs on even-indexed samples.
	UINT32 sample_index;

	// The internal key-on value (one bit per voice). A write to the KON register
	// arms it with the value written; the KON/KOFF poll keys the armed voices on
	// and clears it.
	UINT8 internal_kon;

	// The bits the last poll keyed on. They are struck from internal_kon at T30 of
	// the next polling sample.
	UINT8 consumed_kon;

	// The four registers that describe every voice at once, each read once a
	// sample at its own slot: PMON at T28, and NON, EON and DIR together at T29.
	UINT8 latched_pmon;
	UINT8 latched_non;
	UINT8 latched_eon;
	UINT8 latched_dir;

	// The extra stage of the VxENVX read-back pipeline.
	UINT8 envx_stage[8];

	// ENDX bits computed but not yet readable, one per voice.
	UINT8 prepared_endx;

	// ENDX clears a key-on has scheduled, one bit per voice.
	UINT8 pending_endx_clear;

	// DSP clock cycles run (one per snes_dsp_step_cycle call, counted before the slot
	// runs) and, per DSP-written register, the count at the CPU's last write to it
	// (SNES_DSP_NO_CPU_WRITE until one lands). A CPU write to the register issued in
	// the two cycles before the DSP's own write survives it.
	UINT64 cycle_count;
	UINT64 outx_write_cycle[8];
	UINT64 envx_write_cycle[8];
	UINT64 endx_write_cycle;
	// The same two-cycle window governs the internal key-on twice over: the T30
	// strike takes a KON write issued inside it, and a key-on the poll takes from a
	// write issued inside it is never absorbed into a standing startup.
	UINT64 kon_write_cycle;

	// The one shared noise generator's 15-bit level, in the internal sample range
	// -4000h..+3FFFh, seeded to -4000h at power-on and reset.
	INT16 noise_level;

	// The echo unit's state. echo_index is the ring position (in 4-byte entries) that
	// walks the echo buffer in APU RAM; echo_length is the applied entry count
	// (EDL<<9), taken only when echo_index is 0. echo_applied_esa is the base the
	// unit's reads and writes address (ESA*100h). The per-channel FIR history
	// holds the last eight entries read from the buffer, newest at echo_fir_pos.
	// echo_fir_coeff holds the eight coefficients as the sample captured them.
	UINT16 echo_index;
	UINT16 echo_length;
	UINT8 echo_applied_esa;
	INT16 echo_fir_left[8];
	INT16 echo_fir_right[8];
	INT8 echo_fir_coeff[8];
	UINT8 echo_fir_pos;

	// The eight voices' streaming state, beside the register file.
	SNES_DSP_VOICE voices[8];

	// The intra-sample slot pipeline. slot_cursor is the next slot Tk (0..31) a
	// snes_dsp_step_cycle call runs. primed marks whether the pipeline scratch is
	// live: a freshly seeded state has no prepared voice-0 output, so its first frame
	// computes at once and primes the pipeline at its close.
	UINT8 slot_cursor;
	UINT8 primed;

	// The frame under construction: the running left/right voice mix and the
	// EON-enabled echo send, each clamped to signed 16 bits after every voice's
	// volume step, and the finished stereo frame the wrap delivers.
	INT32 mix_left;
	INT32 mix_right;
	INT32 echo_send_left;
	INT32 echo_send_right;
	INT16 slot_frame_left;
	INT16 slot_frame_right;

	// The echo write in flight.
	UINT8 echo_write_pending;
	UINT16 echo_write_entry;
	UINT8 echo_write_bytes[4];

	// Each voice's amplitude, computed at the voice's S3 slot and applied at its
	// S4/S5 volume slots.
	INT32 voice_amplitude[8];

	// Each voice's amplitude from the sample BEFORE the one in voice_amplitude: the
	// value the next voice's pitch modulation reads.
	INT32 modulator_amplitude[8];

	// VxOUTX and VxENVX are computed at a voice's S3 slot but do not become readable
	// in the register file until its later output slots (S8/S9).
	UINT8 prepared_outx[8];
	UINT8 prepared_envx[8];

	// The step each voice's next compute advances by.
	UINT16 pitch_latch[8];

	// One bit per voice: a consumed key-on scheduled the voice's pitch capture
	// for the next poll-parity sample's start.
	UINT8 pitch_reload_pending;

	// The echo unit's FIR output for the frame, filtered at T26 once the last two
	// coefficients are captured.
	INT32 echo_fir_out_left;
	INT32 echo_fir_out_right;

	// The echo unit's one-slot staging.
	UINT8 echo_gate_left;
	UINT8 echo_gate_right;
	UINT8 echo_latched_esa;
	UINT8 echo_latched_edl;

	// Per-voice mute mask (libvgm's RWF_CHN_MUTE convention, bit n = voice n): a
	// muted voice still runs, but adds nothing to the main and echo sums. Not part of
	// the C++ state; 0 is identical to the C++.
	UINT8 mute_mask;
} SNES_DSP;

// The stamp a DSP-written register carries before the CPU has ever written it.
#define SNES_DSP_NO_CPU_WRITE	(~(UINT64)0)

// Sets the DSP to the C++ DspState defaults: all zero except the noise level
// (-4000h), the voices (Release, computes_since_key_on 0xFF) and the "never
// written" stamps. The mute mask is cleared with the rest: a caller that keeps a
// mask across a re-init sets it again.
void snes_dsp_init(SNES_DSP* dsp);

// Runs exactly one 32-clock slot of the current sample: the slot at dsp->slot_cursor,
// which is then advanced. Slot T0 (the wrap) delivers the frame the previous sample
// finalized: the function returns 1 and the frame is dsp->slot_frame_left / right;
// every other slot returns 0. `ram` is the APU's 64KB RAM, which the echo unit
// reaches directly (its delay line lives there).
int snes_dsp_step_cycle(SNES_DSP* dsp, UINT8* ram);

// A CPU write to DSP register `reg` through DSPDATA (a `reg` above $7F is
// ignored, as the machine ignores it). The value is stored; an ENDX ($7C) write
// stores nothing and acknowledges every end flag instead; a KON ($4C) write also
// arms the internal key-on the next poll consumes. A write to a register the DSP
// itself writes (VxOUTX, VxENVX, ENDX) is stamped with the DSP clock, so the DSP's
// own write in the following two cycles loses to it. A direct dsp->regs[reg] store
// neither arms nor stamps.
void snes_dsp_cpu_write_register(SNES_DSP* dsp, UINT8 reg, UINT8 value);

#ifdef __cplusplus
}
#endif

#endif	// __SNES_DSP_H__
