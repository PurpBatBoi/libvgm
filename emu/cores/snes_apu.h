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

#ifndef __SNES_APU_H__
#define __SNES_APU_H__

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "snes_spc700.h"
#include "snes_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

// The APU machine: the SPC700 wrapped in its 64KB RAM and the $F0-$FF register
// overlay, with the S-DSP beside it (C port of Snaggletooth's apu.h / apu.cpp and
// tools/spc/spc_loader.cpp, pinned at commit efd5049; see snes_apu.c for the port
// notes), plus the libvgm device that plays it (sndDev_SNES_APU).

// One timer's mutable stage counters. The enable bit lives in the CONTROL
// register (SNES_APU.control); the stage-1 ticks come off the machine's master
// counter, SNES_APU.divider.
typedef struct snes_apu_timer
{
	UINT8 stage2;	// 0-255 counter compared against the target
	UINT8 stage3;	// 4-bit output counter; an overlay read of TnOUT clears it
	UINT8 target;	// TnTARGET; a target of 0 divides by 256
} SNES_APU_TIMER;

// The whole machine as a value (the C++ ApuState plus its live CPU).
struct snes_apu
{
	SNES_SPC700 cpu;
	UINT8 ram[65536];
	UINT8 test;
	UINT8 control;
	UINT8 dsp_addr;
	SNES_DSP dsp;
	UINT8 input_ports[4];	// host -> SPC700 (the CPU reads these at $F4-$F7)
	UINT8 output_ports[4];	// SPC700 -> host (the CPU writes these at $F4-$F7)
	// The auxiliary port bytes the CPU reads at $F8/$F9 (the S-SMP's unconnected
	// P4/P5 pins). Storage of their own, not a view of RAM: a CPU write lands here
	// as well as in the RAM beneath, a CPU read returns this value, and the S-DSP's
	// echo buffer writes, which reach only the RAM, never alter it. The
	// unconnected pins read back as written; power-on leaves them high ($FF).
	UINT8 aux_ports[2];
	UINT16 divider;			// the master cycle counter: timer ticks and sample boundaries
	SNES_APU_TIMER timers[3];
	// The speed of the timers (16.16 fixed point, 0 = normal: they tick off the master counter). A music driver
	// counts its tempo with the timers, so a faster speed is a faster tempo with the same pitch (the DSP and the
	// CPU run as before). Not a part of the loaded state: it survives a load.
	UINT32 timer_speed;
	UINT32 timer_acc01;		// the progress of the 128 cycles of timers 0 and 1, in 1/65536 cycle
	UINT32 timer_acc2;		// the progress of the 16 cycles of timer 2
};

// snes_apu_load_spc() results.
#define SNES_APU_LOAD_OK		0
#define SNES_APU_LOAD_TOO_SHORT	1	// shorter than the machine-complete 66,048 bytes
#define SNES_APU_LOAD_BAD_MAGIC	2	// not an SPC700 sound file
#define SNES_APU_LOAD_BAD_MARKER	3	// missing 26,26 marker at offset 0x21

// The size of the machine-complete part of an .spc image (everything the loader reads).
#define SNES_APU_SPC_MIN_SIZE	0x10200

// The seeded post-IPL power-on machine: zeroed RAM, SP at $01EF, TEST $0A, CONTROL
// $B0, the $AA/$BB ready bytes posted to output ports 0 and 1, FLG $E0 (muted) and the
// timers at their power-on values (TnTARGET $00, TnOUT $F). The DSP mute mask is
// cleared with the rest.
void snes_apu_init(SNES_APU* apu);

// Maps an SPC700 Sound File image onto the machine. Validates the magic (through the
// 'v'; the two version digits vary between revisions and are not checked), the 26,26
// marker at offset 0x21 and the machine-complete minimum length; on a rejection the
// machine is left as it was and the reason is returned. The CPU registers, the RAM
// and the DSP registers map verbatim from their offsets; CONTROL, DSPADDR, the timer
// targets and outputs, and both comm-port faces are recovered from the RAM image; the
// extra-RAM field overwrites $FFC0-$FFFF only when the dumped CONTROL bit 7 was set.
// Everything the format does not carry is seeded to power-on values. The DSP mute mask
// is kept.
UINT8 snes_apu_load_spc(SNES_APU* apu, const UINT8* data, UINT32 len);

// Runs exactly one machine cycle: the master counter advances, the timer ticks and the
// DSP slot that land on the new count are taken, and then the CPU makes its one bus
// access. Returns 1 when the DSP delivered a frame on this cycle (every 32 cycles), the
// frame being apu->dsp.slot_frame_left / slot_frame_right.
int snes_apu_machine_cycle(SNES_APU* apu);

// Runs machine cycles until exactly `frames` 32 kHz stereo frames are out, writing them
// straight into the caller's buffers (DEV_SMPL, unscaled 16-bit values). The stop lands
// on the cycle that delivers the last frame, mid-instruction included: render(a) then
// render(b) is bitwise render(a+b).
void snes_apu_render(SNES_APU* apu, UINT32 frames, INT32* out_left, INT32* out_right);

extern const DEV_DECL sndDev_SNES_APU;

#ifdef __cplusplus
}
#endif

#endif	// __SNES_APU_H__
