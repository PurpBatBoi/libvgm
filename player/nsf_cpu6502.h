#ifndef __NSF_CPU6502_H__
#define __NSF_CPU6502_H__

#include "../stdtype.h"

#ifdef __cplusplus
extern "C" {
#endif

// The MOS 6502 as used by the NES (the 2A03: no decimal mode), the instruction-set core
// of the NSF player. C port of ares' ares/component/processor/mos6502 (ISC license, see
// nsf_cpu6502.c).
//
// The core is cycle exact in its bus use: every cycle of an instruction is one call of
// the read or the write function, including the dummy reads and writes, in the order of
// the chip. The owner counts the cycles by counting these calls (an internal cycle is a
// read whose result is dropped), which is how the NSF player times register writes.

typedef struct nsf_cpu6502 NSF_CPU6502;

// The 16-bit address space of the CPU. The owner of the CPU provides both functions,
// `ctx` is the pointer stored in the CPU.
typedef UINT8 (*NSFCPU_READ)(void* ctx, UINT16 address);
typedef void (*NSFCPU_WRITE)(void* ctx, UINT16 address, UINT8 value);

struct nsf_cpu6502
{
	NSFCPU_READ read;
	NSFCPU_WRITE write;
	void* ctx;

	UINT8 a;
	UINT8 x;
	UINT8 y;
	UINT8 s;
	// the flags of the P register, each 0 or 1 (bits 4 and 5 of P exist only on the stack)
	UINT8 c;	// carry
	UINT8 z;	// zero
	UINT8 i;	// interrupt disable
	UINT8 d;	// decimal mode (can be set and read back, has no effect)
	UINT8 v;	// overflow
	UINT8 n;	// negative
	UINT16 pc;
	UINT16 mar;	// the address of the access an instruction is working on
	UINT8 mdr;	// the data of that access
};

// Registers to the power-on state of ares: A = X = Y = S = 0, P = 0x04. The bus functions
// and the context stay as they are.
void nsf_cpu6502_power(NSF_CPU6502* cpu);

// The P register as a byte (bits 4 and 5 read as 0) and back (bits 4 and 5 are ignored).
UINT8 nsf_cpu6502_get_p(const NSF_CPU6502* cpu);
void nsf_cpu6502_set_p(NSF_CPU6502* cpu, UINT8 value);

// Fetches and executes one instruction at PC. The undocumented opcodes work as in ares;
// the JAM opcodes keep PC where it is (the CPU is stuck until the owner changes PC).
void nsf_cpu6502_instruction(NSF_CPU6502* cpu);

#ifdef __cplusplus
}
#endif

#endif	// __NSF_CPU6502_H__
