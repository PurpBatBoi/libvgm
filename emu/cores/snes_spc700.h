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

#ifndef __SNES_SPC700_H__
#define __SNES_SPC700_H__

#include "../../stdtype.h"

#ifdef __cplusplus
extern "C" {
#endif

// The SPC700, the SNES audio CPU's instruction-set core (C port of Snaggletooth's
// include/snaggletooth/apu/spc700.h, pinned at commit efd5049).

// The machine around the CPU (snes_apu.h). The CPU never looks inside it: every
// access it makes goes through the two bus functions below.
typedef struct snes_apu SNES_APU;

// The SPC700's view of its 64KB address space. The owner of the CPU (snes_apu.c)
// defines these: an 8-bit read for any 16-bit address and an 8-bit write to one.
UINT8 snes_apu_bus_read(SNES_APU* apu, UINT16 address);
void snes_apu_bus_write(SNES_APU* apu, UINT16 address, UINT8 value);

// Run states.
#define SPC700_RUN_RUNNING	0
#define SPC700_RUN_SLEEPING	1
#define SPC700_RUN_STOPPED	2

// PSW flag masks: the program status word is a single packed byte.
#define SPC700_FLAG_N	0x80	// negative (high bit of the result)
#define SPC700_FLAG_V	0x40	// signed overflow
#define SPC700_FLAG_P	0x20	// direct page: moves the page to $0100
#define SPC700_FLAG_B	0x10	// break
#define SPC700_FLAG_H	0x08	// half-carry (nibble boundary)
#define SPC700_FLAG_I	0x04	// interrupt enable (no interrupts on the S-SMP)
#define SPC700_FLAG_Z	0x02	// zero
#define SPC700_FLAG_C	0x01	// carry

// The whole CPU state as a value. Instruction progress is part of it (ir, tcu and
// the scratch fields), so the CPU can rest at any cycle, mid-instruction included.
// An all-zero struct is the C++ default Spc700State (Running, tcu 0).
typedef struct snes_spc700
{
	UINT16 pc;
	UINT8 a;
	UINT8 x;
	UINT8 y;
	UINT8 sp;
	UINT8 psw;
	UINT8 run;		// SPC700_RUN_*

	// How far into an instruction the core is. These carry no architectural
	// meaning: they are the progress a running instruction has made.
	UINT8 ir;		// the instruction being executed
	UINT8 tcu;		// cycle index within it; 0 means the next cycle fetches
	UINT16 ea;		// effective-address scratch
	UINT16 ptr;		// pointer / second-address scratch
	UINT16 tmp;		// data scratch (an operand byte, or a value in flight)
	UINT8 taken;	// a branch condition, settled before the cycles it prices (0 or 1)
} SNES_SPC700;

// Whether the core sits between instructions: the only point at which an
// instruction can begin.
#define snes_spc700_at_boundary(cpu)	((cpu)->tcu == 0)

// Executes exactly one chip cycle. The cycle either reaches memory (one read or one
// write, through the bus functions above) or is internal and touches the bus not at
// all. A halted core (sleeping or stopped) touches neither state nor bus.
void snes_spc700_step_cycle(SNES_SPC700* cpu, SNES_APU* apu);

#ifdef __cplusplus
}
#endif

#endif	// __SNES_SPC700_H__
