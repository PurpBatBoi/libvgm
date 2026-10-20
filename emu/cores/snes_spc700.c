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

// SPC700 (SNES audio CPU)
//
// C port of Snaggletooth's SPC700 (https://github.com/etroimcasso/Snaggletooth,
// pinned at commit efd5049):
//   include/snaggletooth/apu/spc700.h (header-only, templated on the ApuBus concept)
//   src/spc700.cpp (the state's byte form only)
//
// Port notes:
// - The core is cycle-stepped exactly as in the C++: snes_spc700_step_cycle() runs one
//   chip cycle, and the instruction progress (ir, tcu and the scratch fields) is part of
//   the state. The C++ class and its Spc700State are one struct here (SNES_SPC700).
// - template <ApuBus B>: the only bus is Apu::Bus, so there is no function pointer table.
//   The core calls snes_apu_bus_read() / snes_apu_bus_write() (declared in
//   snes_spc700.h, defined by snes_apu.c) with the machine it was given.
// - The C++ enum classes (AddrMode, MemAccess, DestinationCycle, WordForm, CtrlKind) are
//   plain enums, the MemForm / CtrlForm structs are filled by the same routing switches
//   (memoryForm, wordForm, controlForm) on every cycle, as in the C++.
// - C++ bool fields and locals are UINT8 / int holding 0 or 1.
// - Not ported (no supported caller uses them):
//   - stepInstruction() and Spc700State's byte form (asBytes / fromBytes): the
//     machine steps single cycles, and snapshots are not supported.
//   - The default member values of a halted core are not needed: an all-zero struct is
//     the C++ default state.
// - Deliberate changes from the C++: none.

#include <stddef.h>

#include "../../stdtype.h"
#include "../../common_def.h"
#include "snes_spc700.h"


// ---- the cycle engine's forms ---------------------------------------------

// Where a memory instruction's operand lives. Each mode is a fixed cycle sequence:
// the operand bytes it fetches, the internal cycles it spends, and the pointer
// reads it makes before its address is settled.
enum
{
	AM_IMPLIED,				// no operand at all
	AM_IMMEDIATE,			// the byte after the opcode
	AM_DP,					// dp + offset
	AM_DPX,					// dp + (offset + X), wrapping inside the page
	AM_DPY,					// dp + (offset + Y), wrapping inside the page
	AM_ABS,					// a two-byte address
	AM_ABSX,				// that address plus X
	AM_ABSY,				// that address plus Y
	AM_ABSBIT,				// a two-byte operand: an address in the low 13 bits, a bit index
							// in the top 3
	AM_INDIRECT,			// (X): dp + X
	AM_INDIRECT_INC,		// (X)+: the same, and X steps after
	AM_INDEXED_INDIRECT,	// [dp+X]: X indexes the offset, then a pointer
	AM_INDIRECT_INDEXED,	// [dp]+Y: a pointer, then Y indexes the address
	AM_INDIRECT_TO_INDIRECT,	// (X),(Y): the Y side is the source, the X side the target
	AM_DP_TO_DP,			// two direct-page offsets: source then destination
	AM_IMMEDIATE_TO_DP		// an immediate byte and a direct-page offset
};

// What an instruction does once its address is settled.
enum
{
	MA_READ,		// read the byte and apply it to a register or to the carry flag
	MA_WRITE,		// put a value there
	MA_MODIFY,		// read the byte, compute from it, and write the result back
	MA_INTERNAL		// read the byte and settle on it inside the chip, writing nothing
};

// What an instruction that reaches a destination does on the cycles between settling
// its address and writing. Most forms read the byte once: a modify needs it, and a
// plain write discards it but still makes the access, so a register that clears when
// read sees it. The two test-and-set instructions and the bit store read it and then
// spend a cycle inside the chip; one store spends its only such cycle inside the chip;
// and the two-operand moves have no such cycle at all.
enum
{
	DC_NONE,
	DC_INTERNAL,
	DC_READ,
	DC_READ_THEN_WAIT,
	DC_READ_THEN_READ
};

// The shape of one memory instruction: where its operand lives, what it does there,
// how it reaches its destination, and how many internal cycles it spends.
// The C++ defaults are mode Implied, access Read, destination Read, internals 0.
typedef struct
{
	UINT8 mode;
	UINT8 access;
	UINT8 destination;
	UINT8 internals;	// extra internal cycles, on the implied forms only
} MEM_FORM;

// Where a direct-page word instruction's cycles go. One offset names both of its
// bytes (the second wraps inside the page) and the shapes differ in what happens
// between them.
enum
{
	WF_NONE,		// not a direct-page word instruction
	WF_READ,		// the low byte, a cycle inside the chip, the high byte
	WF_COMPARE,		// the low byte, the high byte, with no cycle between them
	WF_MODIFY,		// the low byte, write it back, the high byte, write it back
	WF_STORE		// read the low byte, write the low byte, write the high byte
};

// What an instruction that moves the program counter does with it. Each kind is
// its own cycle sequence: a branch reads a displacement and adds it, a jump takes
// a destination, a call puts the return address on the stack around the same work,
// and a return takes one back off it.
enum
{
	CK_BRANCH,				// the eight conditional relative branches, and BRA
	CK_BIT_BRANCH,			// BBS/BBC dp.b and CBNE dp: a direct-page byte, then the offset
	CK_COMPARE_INDEXED,		// CBNE dp+X: the same, with the indexing cycle before the byte
	CK_DECREMENT_DP,		// DBNZ dp: the decremented byte is written before the offset
	CK_DECREMENT_Y,			// DBNZ Y: the offset's own address is read a second time
	CK_JUMP,				// JMP !abs
	CK_JUMP_INDEXED,		// JMP [!abs+X]: an indexed pointer, its halves a linear byte apart
	CK_CALL,				// CALL !abs
	CK_CALL_PAGE,			// PCALL up: the destination is one byte into page $FF
	CK_CALL_VECTOR,			// TCALL n: the destination comes from the table below $FFDE
	CK_BREAK,				// BRK: the same table, with the status byte pushed as well
	CK_RETURN,				// RET
	CK_RETURN_INTERRUPT,	// RETI: the status byte comes back under the return address
	CK_PUSH,				// PUSH A/X/Y/PSW
	CK_POP,					// POP A/X/Y/PSW
	CK_HALT					// SLEEP / STOP
};

// The shape of one control-flow instruction. A relative branch also carries the
// status bit it tests and the state of that bit which takes it.
// The C++ defaults are kind Branch, flag 0, whenSet false.
typedef struct
{
	UINT8 kind;
	UINT8 flag;			// the status bit the branch tests; 0 for BRA
	UINT8 when_set;		// whether the branch is taken with that bit set
} CTRL_FORM;


// ---- the flags and arithmetic cores ---------------------------------------

// The direct-page base: the P flag selects $0100 over $0000.
INLINE UINT16 dp_base(const SNES_SPC700* cpu)
{
	return (cpu->psw & SPC700_FLAG_P) ? 0x0100u : 0x0000u;
}

// The direct-page address an offset names: the base plus the offset, which wraps
// inside the page.
INLINE UINT16 dp_addr(const SNES_SPC700* cpu, UINT8 offset)
{
	return (UINT16)(dp_base(cpu) + offset);
}

// Sets N and Z from a result, leaving the other flags untouched.
INLINE void set_nz(SNES_SPC700* cpu, UINT8 v)
{
	cpu->psw = (UINT8)((cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_Z)) |
						(v & SPC700_FLAG_N) | (v == 0 ? SPC700_FLAG_Z : 0));
}

// Sets the carry flag from a boolean, leaving the others untouched.
INLINE void set_carry(SNES_SPC700* cpu, int carry)
{
	cpu->psw = (UINT8)(carry ? (cpu->psw | SPC700_FLAG_C) : (cpu->psw & ~SPC700_FLAG_C));
}

// Add-with-carry core: adds rhs and the current carry to lhs, sets N/V/H/Z/C
// from the binary result, and returns it. V is signed overflow (both inputs
// agree in sign but the result disagrees); H is the carry out of bit 3.
static UINT8 adc_op(SNES_SPC700* cpu, UINT8 lhs, UINT8 rhs)
{
	const unsigned int carry_in = (cpu->psw & SPC700_FLAG_C) ? 1u : 0u;
	const unsigned int sum = (unsigned int)lhs + (unsigned int)rhs + carry_in;
	const UINT8 result = (UINT8)sum;
	const int half_carry = ((lhs & 0x0Fu) + (rhs & 0x0Fu) + carry_in) > 0x0Fu;
	const int overflow = (~(lhs ^ rhs) & (lhs ^ result) & 0x80u) != 0;
	UINT8 psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_V | SPC700_FLAG_H |
									SPC700_FLAG_Z | SPC700_FLAG_C));
	if (result & 0x80u) psw |= SPC700_FLAG_N;
	if (overflow) psw |= SPC700_FLAG_V;
	if (half_carry) psw |= SPC700_FLAG_H;
	if (result == 0) psw |= SPC700_FLAG_Z;
	if (sum > 0xFFu) psw |= SPC700_FLAG_C;
	cpu->psw = psw;
	return result;
}

// Subtract-with-borrow reuses the adder against the ones-complement operand:
// A - M - !C == A + ~M + C, so carry, half-carry and overflow all fall out of
// the same binary addition (carry set means no borrow).
static UINT8 sbc_op(SNES_SPC700* cpu, UINT8 lhs, UINT8 rhs)
{
	return adc_op(cpu, lhs, (UINT8)(rhs ^ 0xFFu));
}

// Compare: sets N and Z from lhs - rhs and C when no borrow occurs
// (lhs >= rhs). The difference is discarded; no operand is written and H and V
// are untouched.
static void cmp_op(SNES_SPC700* cpu, UINT8 lhs, UINT8 rhs)
{
	const UINT8 result = (UINT8)(lhs - rhs);
	UINT8 psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_Z | SPC700_FLAG_C));
	if (result & 0x80u) psw |= SPC700_FLAG_N;
	if (result == 0) psw |= SPC700_FLAG_Z;
	if (lhs >= rhs) psw |= SPC700_FLAG_C;
	cpu->psw = psw;
}

// Shift/rotate cores: each moves the bit leaving the byte into carry, then
// sets N and Z from the result. LSR shifts a zero in, so it always clears N;
// ROL/ROR feed the old carry into the vacated bit.
static UINT8 asl_op(SNES_SPC700* cpu, UINT8 v)
{
	UINT8 result;
	set_carry(cpu, (v & 0x80u) != 0);
	result = (UINT8)(v << 1);
	set_nz(cpu, result);
	return result;
}

static UINT8 lsr_op(SNES_SPC700* cpu, UINT8 v)
{
	UINT8 result;
	set_carry(cpu, (v & 0x01u) != 0);
	result = (UINT8)(v >> 1);
	set_nz(cpu, result);
	return result;
}

static UINT8 rol_op(SNES_SPC700* cpu, UINT8 v)
{
	const UINT8 carry_in = (cpu->psw & SPC700_FLAG_C) ? 1u : 0u;
	UINT8 result;
	set_carry(cpu, (v & 0x80u) != 0);
	result = (UINT8)((v << 1) | carry_in);
	set_nz(cpu, result);
	return result;
}

static UINT8 ror_op(SNES_SPC700* cpu, UINT8 v)
{
	const UINT8 carry_in = (cpu->psw & SPC700_FLAG_C) ? 0x80u : 0u;
	UINT8 result;
	set_carry(cpu, (v & 0x01u) != 0);
	result = (UINT8)((v >> 1) | carry_in);
	set_nz(cpu, result);
	return result;
}

// 16-bit N and Z: N is bit 15 of the word, Z is set when the whole word is
// zero. The other flags are untouched.
static void set_nz16(SNES_SPC700* cpu, UINT16 v)
{
	cpu->psw = (UINT8)((cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_Z)) |
						((v & 0x8000u) ? SPC700_FLAG_N : 0) |
						(v == 0 ? SPC700_FLAG_Z : 0));
}

// 16-bit add core: two chained byte adds with the carry threaded from the low
// byte into the high byte. N/V/H/C are taken from the high-byte add (H is the
// nibble carry at bit 11->12, V its signed overflow); Z is the full 16-bit
// result. ADDW passes carry_in 0; SUBW passes the ones-complement addend with
// carry_in 1, so borrow, half-borrow and overflow all fall out of this one add.
static UINT16 addw_core(SNES_SPC700* cpu, UINT16 ya, UINT16 addend, unsigned int carry_in)
{
	const unsigned int lo_a = ya & 0xFFu, lo_b = addend & 0xFFu;
	const unsigned int hi_a = (ya >> 8) & 0xFFu, hi_b = (addend >> 8) & 0xFFu;
	const unsigned int lo = lo_a + lo_b + carry_in;
	const unsigned int hi = hi_a + hi_b + (lo >> 8);
	const UINT16 result = (UINT16)(((hi & 0xFFu) << 8) | (lo & 0xFFu));
	const int half_carry = ((hi_a & 0x0Fu) + (hi_b & 0x0Fu) + (lo >> 8)) > 0x0Fu;
	const int overflow = (~(hi_a ^ hi_b) & (hi_a ^ hi) & 0x80u) != 0;
	UINT8 psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_V | SPC700_FLAG_H |
									SPC700_FLAG_Z | SPC700_FLAG_C));
	if (result & 0x8000u) psw |= SPC700_FLAG_N;
	if (overflow) psw |= SPC700_FLAG_V;
	if (half_carry) psw |= SPC700_FLAG_H;
	if (result == 0) psw |= SPC700_FLAG_Z;
	if (hi > 0xFFu) psw |= SPC700_FLAG_C;
	cpu->psw = psw;
	return result;
}

static UINT16 addw_op(SNES_SPC700* cpu, UINT16 ya, UINT16 word)
{
	return addw_core(cpu, ya, word, 0);
}

static UINT16 subw_op(SNES_SPC700* cpu, UINT16 ya, UINT16 word)
{
	return addw_core(cpu, ya, (UINT16)(~word), 1);
}

// 16-bit compare: N and Z from YA - word and C on no borrow (YA >= word). The
// difference is discarded; H and V are untouched.
static void cmpw_op(SNES_SPC700* cpu, UINT16 ya, UINT16 word)
{
	const UINT16 diff = (UINT16)(ya - word);
	UINT8 psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_Z | SPC700_FLAG_C));
	if (diff & 0x8000u) psw |= SPC700_FLAG_N;
	if (diff == 0) psw |= SPC700_FLAG_Z;
	if (ya >= word) psw |= SPC700_FLAG_C;
	cpu->psw = psw;
}

// Reads the byte at PC and advances it (a program fetch).
INLINE UINT8 fetch(SNES_SPC700* cpu, SNES_APU* apu)
{
	return snes_apu_bus_read(apu, cpu->pc ++);
}

// The stack lives in page $01, addressed by SP as its low byte. `step` counts
// entries from the one SP names (a push reaches 0 then -1, a pop reaches +1 then
// +2) and the count wraps inside the page, because SP is an 8-bit register. The
// register itself settles on the instruction's last cycle, so every address in a
// stack instruction is measured from where SP began.
INLINE UINT16 stack_addr(const SNES_SPC700* cpu, int step)
{
	return (UINT16)(0x0100u + ((cpu->sp + step) & 0xFF));
}

// The 16-bit accumulator pair: Y is the high byte, A the low.
INLINE UINT16 get_ya(const SNES_SPC700* cpu)
{
	return (UINT16)(cpu->a | (cpu->y << 8));
}

INLINE void set_ya(SNES_SPC700* cpu, UINT16 word)
{
	cpu->a = (UINT8)word;
	cpu->y = (UINT8)(word >> 8);
}

// A bit-addressing instruction's operand names an address in its low 13 bits and a
// bit index in its top 3. The whole operand stays in the pointer scratch, so the
// index is still there when the byte arrives.
INLINE UINT8 operand_bit_mask(const SNES_SPC700* cpu)
{
	return (UINT8)(1u << (cpu->ptr >> 13));
}

INLINE int operand_bit(const SNES_SPC700* cpu, UINT8 value)
{
	return (value & operand_bit_mask(cpu)) != 0;
}

// Reads the byte after the opcode and discards it. A one-byte instruction still
// issues that read (on real hardware it has the side effects any read has) but the
// program counter does not step over it.
INLINE void discard_next_byte(SNES_SPC700* cpu, SNES_APU* apu)
{
	(void)snes_apu_bus_read(apu, cpu->pc);
}


// ---- the routing tables ---------------------------------------------------

#define MEMFORM(m, a, d, i) \
	(form->mode = (m), form->access = (a), form->destination = (d), form->internals = (i))

// The routing table for the cycle engine: every opcode it carries, with the mode whose
// cycle sequence it runs and what it does at the end of it. Fills in the shape of a
// memory instruction and answers whether the opcode is one the cycle engine carries.
// Instructions sharing a mode share its cycle sequence, so an opcode's only
// per-instruction contributions are this shape and the payload applied once its operand
// has arrived.
static int memory_form(UINT8 opcode, MEM_FORM* form)
{
	switch(opcode)
	{
	// ---- 8-bit move: memory to register ----
	case 0xE8: case 0xCD: case 0x8D:	// MOV A/X/Y,#imm
		MEMFORM(AM_IMMEDIATE, MA_READ, DC_READ, 0);
		return 1;
	case 0xE4: case 0xF8: case 0xEB:	// MOV A/X/Y,dp
		MEMFORM(AM_DP, MA_READ, DC_READ, 0);
		return 1;
	case 0xF4: case 0xFB:				// MOV A/Y,dp+X
		MEMFORM(AM_DPX, MA_READ, DC_READ, 0);
		return 1;
	case 0xF9:							// MOV X,dp+Y
		MEMFORM(AM_DPY, MA_READ, DC_READ, 0);
		return 1;
	case 0xE5: case 0xE9: case 0xEC:	// MOV A/X/Y,!abs
		MEMFORM(AM_ABS, MA_READ, DC_READ, 0);
		return 1;
	case 0xF5:							// MOV A,!abs+X
		MEMFORM(AM_ABSX, MA_READ, DC_READ, 0);
		return 1;
	case 0xF6:							// MOV A,!abs+Y
		MEMFORM(AM_ABSY, MA_READ, DC_READ, 0);
		return 1;
	case 0xE7:							// MOV A,[dp+X]
		MEMFORM(AM_INDEXED_INDIRECT, MA_READ, DC_READ, 0);
		return 1;
	case 0xF7:							// MOV A,[dp]+Y
		MEMFORM(AM_INDIRECT_INDEXED, MA_READ, DC_READ, 0);
		return 1;
	case 0xE6:							// MOV A,(X)
		MEMFORM(AM_INDIRECT, MA_READ, DC_READ, 0);
		return 1;
	case 0xBF:							// MOV A,(X)+
		MEMFORM(AM_INDIRECT_INC, MA_READ, DC_READ, 0);
		return 1;

	// ---- 8-bit move: register to memory ----
	case 0xC4: case 0xCB: case 0xD8:	// MOV dp,A/Y/X
		MEMFORM(AM_DP, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xD4: case 0xDB:				// MOV dp+X,A/Y
		MEMFORM(AM_DPX, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xD9:							// MOV dp+Y,X
		MEMFORM(AM_DPY, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xC5: case 0xC9: case 0xCC:	// MOV !abs,A/X/Y
		MEMFORM(AM_ABS, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xD5:							// MOV !abs+X,A
		MEMFORM(AM_ABSX, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xD6:							// MOV !abs+Y,A
		MEMFORM(AM_ABSY, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xC7:							// MOV [dp+X],A
		MEMFORM(AM_INDEXED_INDIRECT, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xD7:							// MOV [dp]+Y,A
		MEMFORM(AM_INDIRECT_INDEXED, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xC6:							// MOV (X),A
		MEMFORM(AM_INDIRECT, MA_WRITE, DC_READ, 0);
		return 1;
	case 0xAF:							// MOV (X)+,A: the one store that spends its
		MEMFORM(AM_INDIRECT_INC,		// destination cycle
				MA_WRITE,				// internally
				DC_INTERNAL, 0);
		return 1;

	// ---- 8-bit move: register to register, and the two-operand direct-page moves ----
	case 0x7D: case 0xDD: case 0x5D: case 0xFD: case 0x9D: case 0xBD:
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 0);
		return 1;
	case 0xFA:							// MOV dp,dp: the one two-operand form that
		MEMFORM(AM_DP_TO_DP,			// reaches its destination
				MA_WRITE,				// only to write
				DC_NONE, 0);
		return 1;
	case 0x8F:							// MOV dp,#imm
		MEMFORM(AM_IMMEDIATE_TO_DP, MA_WRITE, DC_READ, 0);
		return 1;

	// ---- 8-bit arithmetic, logic and comparison against a register ----
	// Every one of these reads a byte and applies it to A, X or Y; they differ only in
	// the payload, so they share the read sequence of the move that fetches the same way.
	case 0x88: case 0xA8: case 0x68: case 0xC8: case 0xAD:	// ADC/SBC/CMP A,#imm,
	case 0x28: case 0x08: case 0x48:						// CMP X/Y,#imm, AND/OR/EOR
		MEMFORM(AM_IMMEDIATE, MA_READ, DC_READ, 0);
		return 1;
	case 0x84: case 0xA4: case 0x64: case 0x3E: case 0x7E:	// ...,dp
	case 0x24: case 0x04: case 0x44:
		MEMFORM(AM_DP, MA_READ, DC_READ, 0);
		return 1;
	case 0x94: case 0xB4: case 0x74: case 0x34: case 0x14: case 0x54:	// ...,dp+X
		MEMFORM(AM_DPX, MA_READ, DC_READ, 0);
		return 1;
	case 0x85: case 0xA5: case 0x65: case 0x1E: case 0x5E:	// ...,!abs
	case 0x25: case 0x05: case 0x45:
		MEMFORM(AM_ABS, MA_READ, DC_READ, 0);
		return 1;
	case 0x95: case 0xB5: case 0x75: case 0x35: case 0x15: case 0x55:	// ...,!abs+X
		MEMFORM(AM_ABSX, MA_READ, DC_READ, 0);
		return 1;
	case 0x96: case 0xB6: case 0x76: case 0x36: case 0x16: case 0x56:	// ...,!abs+Y
		MEMFORM(AM_ABSY, MA_READ, DC_READ, 0);
		return 1;
	case 0x87: case 0xA7: case 0x67: case 0x27: case 0x07: case 0x47:	// ...,[dp+X]
		MEMFORM(AM_INDEXED_INDIRECT, MA_READ, DC_READ, 0);
		return 1;
	case 0x97: case 0xB7: case 0x77: case 0x37: case 0x17: case 0x57:	// ...,[dp]+Y
		MEMFORM(AM_INDIRECT_INDEXED, MA_READ, DC_READ, 0);
		return 1;
	case 0x86: case 0xA6: case 0x66: case 0x26: case 0x06: case 0x46:	// ...,(X)
		MEMFORM(AM_INDIRECT, MA_READ, DC_READ, 0);
		return 1;

	// ---- 8-bit increment, decrement, shift and rotation of a register ----
	case 0xBC: case 0x3D: case 0xFC: case 0x9C: case 0x1D: case 0xDC:	// INC/DEC A/X/Y
	case 0x1C: case 0x5C: case 0x3C: case 0x7C:							// ASL/LSR/ROL/ROR A
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 0);
		return 1;
	case 0x9F:							// XCN A: the exchange runs on internal cycles
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 3);
		return 1;

	// ---- 8-bit increment, decrement, shift and rotation of a byte in memory ----
	// The read-modify-write seat: the byte is read, the result computed from it, and
	// the result written back to the same address.
	case 0xAB: case 0x8B: case 0x0B: case 0x4B: case 0x2B: case 0x6B:	// ...dp
		MEMFORM(AM_DP, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0xBB: case 0x9B: case 0x1B: case 0x5B: case 0x3B: case 0x7B:	// ...dp+X
		MEMFORM(AM_DPX, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0xAC: case 0x8C: case 0x0C: case 0x4C: case 0x2C: case 0x6C:	// ...!abs
		MEMFORM(AM_ABS, MA_MODIFY, DC_READ, 0);
		return 1;

	// ---- 8-bit arithmetic and logic between two bytes in memory ----
	// The Y side is the source and the X side both the other operand and the target.
	case 0x99: case 0xB9: case 0x39: case 0x19: case 0x59:	// ADC/SBC/AND/OR/EOR (X),(Y)
		MEMFORM(AM_INDIRECT_TO_INDIRECT, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0x79:							// CMP (X),(Y): compares and writes nothing
		MEMFORM(AM_INDIRECT_TO_INDIRECT, MA_INTERNAL, DC_READ, 0);
		return 1;
	case 0x89: case 0xA9: case 0x29: case 0x09: case 0x49:	// ...dp,dp
		MEMFORM(AM_DP_TO_DP, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0x69:							// CMP dp,dp
		MEMFORM(AM_DP_TO_DP, MA_INTERNAL, DC_READ, 0);
		return 1;
	case 0x98: case 0xB8: case 0x38: case 0x18: case 0x58:	// ...dp,#imm
		MEMFORM(AM_IMMEDIATE_TO_DP, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0x78:							// CMP dp,#imm
		MEMFORM(AM_IMMEDIATE_TO_DP, MA_INTERNAL, DC_READ, 0);
		return 1;

	// ---- multiply, divide and decimal adjust ----
	// Each reads the byte after the opcode, discards it, and spends every remaining
	// cycle computing inside the chip.
	case 0xCF:							// MUL YA
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 7);
		return 1;
	case 0x9E:							// DIV YA,X
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 10);
		return 1;
	case 0xDF: case 0xBE:				// DAA / DAS A
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 1);
		return 1;

	// ---- the status flags, and the instruction that does nothing at all ----
	// Each reads the byte after the opcode and discards it, and the three that settle
	// a cycle later spend that cycle inside the chip.
	case 0x00:							// NOP
	case 0x20: case 0x40:				// CLRP / SETP
	case 0x60: case 0x80:				// CLRC / SETC
	case 0xE0:							// CLRV
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 0);
		return 1;
	case 0xED: case 0xA0: case 0xC0:	// NOTC / EI / DI
		MEMFORM(AM_IMPLIED, MA_READ, DC_READ, 1);
		return 1;

	// ---- one bit of a direct-page byte, set or cleared ----
	// The read-modify-write seat once more: the byte is read and written back with a
	// single bit changed, and no flag moves.
	case 0x02: case 0x22: case 0x42: case 0x62:	// SET1 dp.0..7
	case 0x82: case 0xA2: case 0xC2: case 0xE2:
	case 0x12: case 0x32: case 0x52: case 0x72:	// CLR1 dp.0..7
	case 0x92: case 0xB2: case 0xD2: case 0xF2:
		MEMFORM(AM_DP, MA_MODIFY, DC_READ, 0);
		return 1;

	// ---- test and set or clear the bits of an absolute byte ----
	case 0x0E: case 0x4E:				// TSET1 / TCLR1 !abs: the operand is reached
		MEMFORM(AM_ABS,					// twice and the first byte kept,
				MA_MODIFY,				// then the changed byte is written
				DC_READ_THEN_READ, 0);
		return 1;

	// ---- the carry flag against one bit of an absolute byte ----
	case 0x4A: case 0x6A: case 0xAA:	// AND1 C,m.b and C,/m.b; MOV1 C,m.b
		MEMFORM(AM_ABSBIT, MA_READ, DC_READ, 0);
		return 1;
	case 0x0A: case 0x2A: case 0x8A:	// OR1 C,m.b and C,/m.b; EOR1 C,m.b: each pays
		MEMFORM(AM_ABSBIT,				// a last cycle inside the chip
				MA_INTERNAL, DC_READ, 0);
		return 1;
	case 0xEA:							// NOT1 m.b
		MEMFORM(AM_ABSBIT, MA_MODIFY, DC_READ, 0);
		return 1;
	case 0xCA:							// MOV1 m.b,C: reads its byte, spends a cycle
		MEMFORM(AM_ABSBIT,				// inside the chip, then writes
				MA_MODIFY,
				DC_READ_THEN_WAIT, 0);
		return 1;

	default:
		return 0;
	}
}

#undef MEMFORM

#define CTRLFORM(k, f, w) \
	(form->kind = (k), form->flag = (f), form->when_set = (w))

// The routing table for the instructions that move the program counter. Every opcode
// the memory and word tables do not carry is one of these, so between the three every
// opcode has a cycle sequence. Fills in the shape of a control-flow instruction and
// answers whether the opcode is one.
static int control_form(UINT8 opcode, CTRL_FORM* form)
{
	switch(opcode)
	{
	// ---- the relative branches. Two cycles when the condition fails, and two more
	//      inside the chip when it holds ----
	case 0x2F:	// BRA: no flag, so always taken
		CTRLFORM(CK_BRANCH, 0, 0);
		return 1;
	case 0xF0:	// BEQ
		CTRLFORM(CK_BRANCH, SPC700_FLAG_Z, 1);
		return 1;
	case 0xD0:	// BNE
		CTRLFORM(CK_BRANCH, SPC700_FLAG_Z, 0);
		return 1;
	case 0xB0:	// BCS
		CTRLFORM(CK_BRANCH, SPC700_FLAG_C, 1);
		return 1;
	case 0x90:	// BCC
		CTRLFORM(CK_BRANCH, SPC700_FLAG_C, 0);
		return 1;
	case 0x70:	// BVS
		CTRLFORM(CK_BRANCH, SPC700_FLAG_V, 1);
		return 1;
	case 0x50:	// BVC
		CTRLFORM(CK_BRANCH, SPC700_FLAG_V, 0);
		return 1;
	case 0x30:	// BMI
		CTRLFORM(CK_BRANCH, SPC700_FLAG_N, 1);
		return 1;
	case 0x10:	// BPL
		CTRLFORM(CK_BRANCH, SPC700_FLAG_N, 0);
		return 1;

	// ---- the branches that read a byte first: one bit of a direct-page byte, or
	//      that byte against A. They share a cycle sequence and differ in the test ----
	case 0x03: case 0x23: case 0x43: case 0x63:	// BBS dp.0..7,rel
	case 0x83: case 0xA3: case 0xC3: case 0xE3:
	case 0x13: case 0x33: case 0x53: case 0x73:	// BBC dp.0..7,rel
	case 0x93: case 0xB3: case 0xD3: case 0xF3:
	case 0x2E:									// CBNE dp,rel
		CTRLFORM(CK_BIT_BRANCH, 0, 0);
		return 1;
	case 0xDE:									// CBNE dp+X,rel
		CTRLFORM(CK_COMPARE_INDEXED, 0, 0);
		return 1;
	case 0x6E:									// DBNZ dp,rel
		CTRLFORM(CK_DECREMENT_DP, 0, 0);
		return 1;
	case 0xFE:									// DBNZ Y,rel
		CTRLFORM(CK_DECREMENT_Y, 0, 0);
		return 1;

	// ---- jumps, calls and returns ----
	case 0x5F: CTRLFORM(CK_JUMP, 0, 0); return 1;			// JMP !abs
	case 0x1F: CTRLFORM(CK_JUMP_INDEXED, 0, 0); return 1;	// JMP [!abs+X]
	case 0x3F: CTRLFORM(CK_CALL, 0, 0); return 1;			// CALL !abs
	case 0x4F: CTRLFORM(CK_CALL_PAGE, 0, 0); return 1;		// PCALL up
	case 0x01: case 0x11: case 0x21: case 0x31:	// TCALL 0..7
	case 0x41: case 0x51: case 0x61: case 0x71:
	case 0x81: case 0x91: case 0xA1: case 0xB1:	// TCALL 8..15
	case 0xC1: case 0xD1: case 0xE1: case 0xF1:
		CTRLFORM(CK_CALL_VECTOR, 0, 0);
		return 1;
	case 0x0F: CTRLFORM(CK_BREAK, 0, 0); return 1;				// BRK
	case 0x6F: CTRLFORM(CK_RETURN, 0, 0); return 1;				// RET
	case 0x7F: CTRLFORM(CK_RETURN_INTERRUPT, 0, 0); return 1;	// RETI

	// ---- the stack ----
	case 0x2D: case 0x4D: case 0x6D: case 0x0D:	// PUSH A/X/Y/PSW
		CTRLFORM(CK_PUSH, 0, 0);
		return 1;
	case 0xAE: case 0xCE: case 0xEE: case 0x8E:	// POP A/X/Y/PSW
		CTRLFORM(CK_POP, 0, 0);
		return 1;

	// ---- the halts ----
	case 0xEF: case 0xFF:						// SLEEP / STOP
		CTRLFORM(CK_HALT, 0, 0);
		return 1;

	default:
		return 0;
	}
}

#undef CTRLFORM

// The direct-page word instructions, which the cycle engine carries beside the memory
// forms: one offset, two bytes, and a shape that says what happens between them.
static UINT8 word_form(UINT8 opcode)
{
	switch(opcode)
	{
	case 0x7A: case 0x9A: case 0xBA: return WF_READ;	// ADDW/SUBW/MOVW YA,dp
	case 0x5A: return WF_COMPARE;						// CMPW YA,dp
	case 0x1A: case 0x3A: return WF_MODIFY;				// DECW / INCW dp
	case 0xDA: return WF_STORE;							// MOVW dp,YA
	default: return WF_NONE;
	}
}

// How many cycles a mode spends before it reaches memory: its operand fetches, its
// internal cycles, and its pointer reads.
static UINT8 setup_cycles(UINT8 mode)
{
	switch(mode)
	{
	case AM_IMMEDIATE: return 0;	// the operand fetch is the access itself
	case AM_IMPLIED:
	case AM_DP:
	case AM_INDIRECT:
	case AM_INDIRECT_INC: return 1;
	case AM_DPX:
	case AM_DPY:
	case AM_ABS:
	case AM_ABSBIT:
	case AM_INDIRECT_TO_INDIRECT:
	case AM_IMMEDIATE_TO_DP: return 2;
	case AM_ABSX:
	case AM_ABSY:
	case AM_DP_TO_DP: return 3;
	case AM_INDEXED_INDIRECT:
	case AM_INDIRECT_INDEXED: return 4;
	}
	return 0;
}

// How many cycles a form spends between settling its address and writing.
static UINT8 destination_cycles(UINT8 destination)
{
	switch(destination)
	{
	case DC_NONE: return 0;
	case DC_INTERNAL:
	case DC_READ: return 1;
	case DC_READ_THEN_WAIT:
	case DC_READ_THEN_READ: return 2;
	}
	return 0;
}

// Whether an indirect mode spends its internal cycle before reading its pointer.
// The read forms and [dp+X] do; the write form of [dp]+Y spends it after.
INLINE int waits_before_pointer(const MEM_FORM* form)
{
	return form->mode != AM_INDIRECT_INDEXED || form->access == MA_READ;
}

// The address of one byte of an indirect mode's pointer. Both bytes live in the
// direct page, and the second wraps inside it.
INLINE UINT16 pointer_addr(const SNES_SPC700* cpu, const MEM_FORM* form, UINT8 byte)
{
	const UINT8 index = form->mode == AM_INDEXED_INDIRECT ? cpu->x : (UINT8)0;
	return dp_addr(cpu, (UINT8)(cpu->tmp + index + byte));
}

// Whether a mode carries a second operand byte, which lives in the pointer scratch
// while the byte at the destination lives in the data scratch.
INLINE int two_operand(UINT8 mode)
{
	return mode == AM_INDIRECT_TO_INDIRECT || mode == AM_DP_TO_DP ||
			mode == AM_IMMEDIATE_TO_DP;
}

// Whether a relative branch's condition holds. BRA carries no flag and is always
// taken.
INLINE int branch_taken(const SNES_SPC700* cpu, const CTRL_FORM* form)
{
	return form->flag == 0 || (((cpu->psw & form->flag) != 0) == form->when_set);
}

// Whether one of the instructions that reads a byte before its displacement takes
// its branch. The two compare forms branch when A differs from the byte; BBS and
// BBC test one of its bits, the index in the opcode's top three bits and the
// polarity in the fourth: an even high nibble branches on the bit set.
INLINE int byte_branch_taken(const SNES_SPC700* cpu, UINT8 opcode, UINT8 value)
{
	UINT8 high;
	int set;
	if (opcode == 0x2E || opcode == 0xDE) return cpu->a != value;
	high = (UINT8)(opcode >> 4);
	set = ((value >> (high >> 1)) & 1u) != 0;
	return set == ((high & 1u) == 0u);
}

// Adds the displacement in the data scratch to the program counter, which already
// points past the whole instruction.
INLINE void take_branch(SNES_SPC700* cpu)
{
	cpu->pc = (UINT16)(cpu->pc + (INT8)cpu->tmp);
}

// The table entry a call through the top of memory reads its destination from. The
// table ends at $FFDE and counts downwards, two bytes per entry; BRK takes the same
// entry as TCALL 0.
INLINE UINT16 call_vector(const SNES_SPC700* cpu)
{
	const unsigned int entry = cpu->ir == 0x0F ? 0u : (unsigned int)cpu->ir >> 4;
	return (UINT16)(0xFFDEu - 2u * entry);
}

// The register a push writes.
static UINT8 push_value(const SNES_SPC700* cpu, UINT8 opcode)
{
	switch(opcode)
	{
	case 0x2D: return cpu->a;
	case 0x4D: return cpu->x;
	case 0x6D: return cpu->y;
	default: return cpu->psw;	// PUSH PSW
	}
}

// Lands a popped byte in its register. None of the three register forms sets a
// flag; POP PSW replaces the whole status byte, the flags it carries included.
static void apply_pop(SNES_SPC700* cpu, UINT8 opcode, UINT8 value)
{
	switch(opcode)
	{
	case 0xAE: cpu->a = value; break;
	case 0xCE: cpu->x = value; break;
	case 0xEE: cpu->y = value; break;
	default: cpu->psw = value; break;	// POP PSW
	}
}


// ---- the instruction payloads ---------------------------------------------

// Applies an instruction that has finished reading its operand.
static void apply_memory_read(SNES_SPC700* cpu, UINT8 opcode, UINT8 value)
{
	switch(opcode)
	{
	case 0xE8: case 0xE6: case 0xBF: case 0xE4: case 0xF4:
	case 0xE5: case 0xF5: case 0xF6: case 0xE7: case 0xF7:
		cpu->a = value;
		set_nz(cpu, value);
		break;
	case 0xCD: case 0xF8: case 0xF9: case 0xE9:
		cpu->x = value;
		set_nz(cpu, value);
		break;
	case 0x8D: case 0xEB: case 0xFB: case 0xEC:
		cpu->y = value;
		set_nz(cpu, value);
		break;

	// ---- the same reads, applied arithmetically to A ----
	case 0x88: case 0x86: case 0x84: case 0x94: case 0x85:
	case 0x95: case 0x96: case 0x87: case 0x97:				// ADC A,operand
		cpu->a = adc_op(cpu, cpu->a, value);
		break;
	case 0xA8: case 0xA6: case 0xA4: case 0xB4: case 0xA5:
	case 0xB5: case 0xB6: case 0xA7: case 0xB7:				// SBC A,operand
		cpu->a = sbc_op(cpu, cpu->a, value);
		break;
	case 0x68: case 0x66: case 0x64: case 0x74: case 0x65:
	case 0x75: case 0x76: case 0x67: case 0x77:				// CMP A,operand
		cmp_op(cpu, cpu->a, value);
		break;
	case 0xC8: case 0x3E: case 0x1E:						// CMP X,operand
		cmp_op(cpu, cpu->x, value);
		break;
	case 0xAD: case 0x7E: case 0x5E:						// CMP Y,operand
		cmp_op(cpu, cpu->y, value);
		break;
	case 0x28: case 0x26: case 0x24: case 0x34: case 0x25:
	case 0x35: case 0x36: case 0x27: case 0x37:				// AND A,operand
		cpu->a = (UINT8)(cpu->a & value);
		set_nz(cpu, cpu->a);
		break;
	case 0x08: case 0x06: case 0x04: case 0x14: case 0x05:
	case 0x15: case 0x16: case 0x07: case 0x17:				// OR A,operand
		cpu->a = (UINT8)(cpu->a | value);
		set_nz(cpu, cpu->a);
		break;
	case 0x48: case 0x46: case 0x44: case 0x54: case 0x45:
	case 0x55: case 0x56: case 0x47: case 0x57:				// EOR A,operand
		cpu->a = (UINT8)(cpu->a ^ value);
		set_nz(cpu, cpu->a);
		break;

	// ---- one bit of that byte, into the carry flag ----
	case 0x4A:												// AND1 C,m.b
		set_carry(cpu, (cpu->psw & SPC700_FLAG_C) && operand_bit(cpu, value));
		break;
	case 0x6A:												// AND1 C,/m.b
		set_carry(cpu, (cpu->psw & SPC700_FLAG_C) && ! operand_bit(cpu, value));
		break;
	case 0xAA:												// MOV1 C,m.b
		set_carry(cpu, operand_bit(cpu, value));
		break;

	default:
		// Every form that reads a byte into a register or into the carry flag is
		// listed above; the tables send nothing else here.
		break;
	}
}

// The value a store writes.
static UINT8 memory_store_value(const SNES_SPC700* cpu, UINT8 opcode)
{
	switch(opcode)
	{
	case 0xC6: case 0xAF: case 0xC4: case 0xD4: case 0xC5:
	case 0xD5: case 0xD6: case 0xC7: case 0xD7:
		return cpu->a;
	case 0xD8: case 0xD9: case 0xC9:
		return cpu->x;
	case 0xCB: case 0xDB: case 0xCC:
		return cpu->y;
	default:
		// As for apply_memory_read: every store form is listed above.
		return 0;
	}
}

// Applies an implied instruction: a register-to-register move, a register's own
// increment, shift or nibble exchange, or one of the multi-cycle computations the
// chip runs entirely inside itself.
static void apply_implied(SNES_SPC700* cpu, UINT8 opcode)
{
	switch(opcode)
	{
	case 0x7D: cpu->a = cpu->x;  set_nz(cpu, cpu->a); break;	// MOV A,X
	case 0xDD: cpu->a = cpu->y;  set_nz(cpu, cpu->a); break;	// MOV A,Y
	case 0x5D: cpu->x = cpu->a;  set_nz(cpu, cpu->x); break;	// MOV X,A
	case 0xFD: cpu->y = cpu->a;  set_nz(cpu, cpu->y); break;	// MOV Y,A
	case 0x9D: cpu->x = cpu->sp; set_nz(cpu, cpu->x); break;	// MOV X,SP
	case 0xBD: cpu->sp = cpu->x; break;							// MOV SP,X (no flags)

	// ---- a register's own increment, decrement, shift and rotation ----
	case 0xBC: ++ cpu->a; set_nz(cpu, cpu->a); break;			// INC A
	case 0x3D: ++ cpu->x; set_nz(cpu, cpu->x); break;			// INC X
	case 0xFC: ++ cpu->y; set_nz(cpu, cpu->y); break;			// INC Y
	case 0x9C: -- cpu->a; set_nz(cpu, cpu->a); break;			// DEC A
	case 0x1D: -- cpu->x; set_nz(cpu, cpu->x); break;			// DEC X
	case 0xDC: -- cpu->y; set_nz(cpu, cpu->y); break;			// DEC Y
	case 0x1C: cpu->a = asl_op(cpu, cpu->a); break;				// ASL A
	case 0x5C: cpu->a = lsr_op(cpu, cpu->a); break;				// LSR A
	case 0x3C: cpu->a = rol_op(cpu, cpu->a); break;				// ROL A
	case 0x7C: cpu->a = ror_op(cpu, cpu->a); break;				// ROR A
	case 0x9F:													// XCN A
		cpu->a = (UINT8)((cpu->a << 4) | (cpu->a >> 4));
		set_nz(cpu, cpu->a);
		break;

	// ---- multiply and divide (YA is the pair A=low, Y=high) ----
	case 0xCF:	// MUL YA (N,Z from Y only)
	{
		const UINT16 product = (UINT16)((unsigned int)cpu->y * (unsigned int)cpu->a);
		set_ya(cpu, product);
		set_nz(cpu, cpu->y);
		break;
	}
	case 0x9E:	// DIV YA,X
	{
		// The documented 9-iteration restoring division: the 17-bit accumulator ends as
		// YYYYYYYY V AAAAAAAA, so Y and A are the quotient/remainder and bit 8 is the
		// overflow flag. N,Z come from A; H is the nibble comparison X&$F <= Y&$F on the
		// entry values. The result past a quotient of 511 is hardware garbage the
		// algorithm still reproduces.
		const unsigned int entry_x = cpu->x, entry_y = cpu->y;
		UINT32 yva = (UINT32)((entry_y << 8) | cpu->a);
		const UINT32 x9 = (UINT32)entry_x << 9;
		UINT8 psw;
		int i;
		for (i = 0; i < 9; ++i)
		{
			yva = ((yva << 1) | ((yva >> 16) & 1u)) & 0x1FFFFu;	// rotate left within 17 bits
			if (yva >= x9) yva ^= 1u;
			if (yva & 1u) yva = (yva - x9) & 0x1FFFFu;
		}
		cpu->a = (UINT8)(yva & 0xFFu);
		cpu->y = (UINT8)((yva >> 9) & 0xFFu);
		psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_N | SPC700_FLAG_V | SPC700_FLAG_H | SPC700_FLAG_Z));
		if (cpu->a & 0x80u) psw |= SPC700_FLAG_N;
		if (cpu->a == 0) psw |= SPC700_FLAG_Z;
		if (yva & 0x100u) psw |= SPC700_FLAG_V;
		if ((entry_x & 0x0Fu) <= (entry_y & 0x0Fu)) psw |= SPC700_FLAG_H;
		cpu->psw = psw;
		break;
	}

	// ---- decimal adjust (N,Z,C; H is read but never changed) ----
	case 0xDF:	// DAA A
	{
		UINT16 a = cpu->a;
		if ((cpu->psw & SPC700_FLAG_C) || a > 0x99u)
		{
			a += 0x60u;
			cpu->psw = (UINT8)(cpu->psw | SPC700_FLAG_C);
		}
		if ((cpu->psw & SPC700_FLAG_H) || (cpu->a & 0x0Fu) > 0x09u) a += 0x06u;
		cpu->a = (UINT8)a;
		set_nz(cpu, cpu->a);
		break;
	}
	case 0xBE:	// DAS A
	{
		UINT16 a = cpu->a;
		if (! (cpu->psw & SPC700_FLAG_C) || a > 0x99u)
		{
			a -= 0x60u;
			cpu->psw = (UINT8)(cpu->psw & ~SPC700_FLAG_C);
		}
		if (! (cpu->psw & SPC700_FLAG_H) || (cpu->a & 0x0Fu) > 0x09u) a -= 0x06u;
		cpu->a = (UINT8)a;
		set_nz(cpu, cpu->a);
		break;
	}

	// ---- the status flags ----
	// CLRV clears the half-carry with the overflow, and I is an enable rather than a
	// disable: EI sets it, DI clears it. The audio unit has no interrupt source, so
	// nothing is ever delivered through it.
	case 0x60: cpu->psw = (UINT8)(cpu->psw & ~SPC700_FLAG_C); break;
	case 0x80: cpu->psw = (UINT8)(cpu->psw | SPC700_FLAG_C); break;
	case 0xED: cpu->psw = (UINT8)(cpu->psw ^ SPC700_FLAG_C); break;
	case 0x20: cpu->psw = (UINT8)(cpu->psw & ~SPC700_FLAG_P); break;
	case 0x40: cpu->psw = (UINT8)(cpu->psw | SPC700_FLAG_P); break;
	case 0xE0:
		cpu->psw = (UINT8)(cpu->psw & ~(SPC700_FLAG_V | SPC700_FLAG_H));
		break;
	case 0xA0: cpu->psw = (UINT8)(cpu->psw | SPC700_FLAG_I); break;
	case 0xC0: cpu->psw = (UINT8)(cpu->psw & ~SPC700_FLAG_I); break;

	case 0x00: break;	// NOP, which reads the byte after the opcode and nothing else

	default: break;
	}
}

// Applies an instruction that settles on the byte it read without writing anything:
// a comparison, or a carry-bit operation that spends a last cycle inside the chip.
static void apply_internal_result(SNES_SPC700* cpu, UINT8 opcode, UINT8 destination, UINT8 source)
{
	switch(opcode)
	{
	case 0x69: case 0x78: case 0x79:	// CMP dp,dp / dp,#imm / (X),(Y)
		cmp_op(cpu, destination, source);
		break;

	// ---- the carry flag against one bit of a byte ----
	case 0x0A:							// OR1 C,m.b
		set_carry(cpu, (cpu->psw & SPC700_FLAG_C) || operand_bit(cpu, destination));
		break;
	case 0x2A:							// OR1 C,/m.b
		set_carry(cpu, (cpu->psw & SPC700_FLAG_C) || ! operand_bit(cpu, destination));
		break;
	case 0x8A:							// EOR1 C,m.b
		set_carry(cpu, ((cpu->psw & SPC700_FLAG_C) != 0) != operand_bit(cpu, destination));
		break;

	default:
		// As for apply_memory_read: every form that settles inside the chip is listed above.
		break;
	}
}

// Applies a word instruction that has both of its bytes.
static void apply_word_read(SNES_SPC700* cpu, UINT8 opcode, UINT16 word)
{
	switch(opcode)
	{
	case 0xBA:							// MOVW YA,dp
		set_ya(cpu, word);
		set_nz16(cpu, word);
		break;
	case 0x7A: set_ya(cpu, addw_op(cpu, get_ya(cpu), word)); break;	// ADDW YA,dp
	case 0x9A: set_ya(cpu, subw_op(cpu, get_ya(cpu), word)); break;	// SUBW YA,dp
	case 0x5A: cmpw_op(cpu, get_ya(cpu), word); break;				// CMPW YA,dp (the difference is discarded)

	default:
		// As for apply_memory_read: every word form that reads a whole word is listed above.
		break;
	}
}

// The value a read-modify-write instruction writes back: computed from the byte at
// the destination and, for the two-operand forms, the source byte beside it. The
// flags it sets are the instruction's, so this runs on the cycle that writes.
static UINT8 memory_modify_value(SNES_SPC700* cpu, UINT8 opcode, UINT8 destination, UINT8 source)
{
	UINT8 v;

	switch(opcode)
	{
	// ---- one byte, changed in place ----
	case 0xAB: case 0xBB: case 0xAC:	// INC dp / dp+X / !abs
		v = (UINT8)(destination + 1);
		set_nz(cpu, v);
		return v;
	case 0x8B: case 0x9B: case 0x8C:	// DEC dp / dp+X / !abs
		v = (UINT8)(destination - 1);
		set_nz(cpu, v);
		return v;
	case 0x0B: case 0x1B: case 0x0C: return asl_op(cpu, destination);	// ASL dp / dp+X / !abs
	case 0x4B: case 0x5B: case 0x4C: return lsr_op(cpu, destination);	// LSR dp / dp+X / !abs
	case 0x2B: case 0x3B: case 0x2C: return rol_op(cpu, destination);	// ROL dp / dp+X / !abs
	case 0x6B: case 0x7B: case 0x6C: return ror_op(cpu, destination);	// ROR dp / dp+X / !abs

	// ---- two bytes, the result landing on the destination ----
	case 0x99: case 0x89: case 0x98:	// ADC (X),(Y) / dp,dp / dp,#imm
		return adc_op(cpu, destination, source);
	case 0xB9: case 0xA9: case 0xB8:	// SBC ...
		return sbc_op(cpu, destination, source);
	case 0x39: case 0x29: case 0x38:	// AND ...
		v = (UINT8)(destination & source);
		set_nz(cpu, v);
		return v;
	case 0x19: case 0x09: case 0x18:	// OR ...
		v = (UINT8)(destination | source);
		set_nz(cpu, v);
		return v;
	case 0x59: case 0x49: case 0x58:	// EOR ...
		v = (UINT8)(destination ^ source);
		set_nz(cpu, v);
		return v;

	// ---- one bit of a direct-page byte, set or cleared (no flags) ----
	// The bit index is in the opcode's top three bits, and the low nibble tells the two
	// instructions apart: SET1 is $x2 with an even high nibble, CLR1 with an odd one.
	case 0x02: case 0x22: case 0x42: case 0x62:	// SET1 dp.0..7
	case 0x82: case 0xA2: case 0xC2: case 0xE2:
		return (UINT8)(destination | (1u << (opcode >> 5)));
	case 0x12: case 0x32: case 0x52: case 0x72:	// CLR1 dp.0..7
	case 0x92: case 0xB2: case 0xD2: case 0xF2:
		return (UINT8)(destination & ~(1u << (opcode >> 5)));

	// ---- the byte tested against A, then its bits set or cleared by A ----
	// The flags are those of A - memory; A itself is untouched, and the bits it holds
	// are the ones the write turns on or off.
	case 0x0E: case 0x4E:				// TSET1 / TCLR1 !abs
		set_nz(cpu, (UINT8)(cpu->a - destination));
		return opcode == 0x0E ? (UINT8)(destination | cpu->a)
								: (UINT8)(destination & ~cpu->a);

	// ---- one bit of a byte, flipped or written from the carry flag (no flags) ----
	case 0xEA:							// NOT1 m.b
		return (UINT8)(destination ^ operand_bit_mask(cpu));
	case 0xCA:							// MOV1 m.b,C
		return (cpu->psw & SPC700_FLAG_C)
				? (UINT8)(destination | operand_bit_mask(cpu))
				: (UINT8)(destination & ~operand_bit_mask(cpu));

	default:
		// As for apply_memory_read: every modify form is listed above.
		return destination;
	}
}


// ---- the cycle engine -----------------------------------------------------

// An address settles over a mode's setup cycles: the operand bytes come off the
// program stream, an indexed mode spends a cycle on the addition, and an indirect one
// reads its pointer out of the direct page. Only an implied instruction finishes here
// (it has no address to settle). Executes one of the cycles a mode spends settling its
// address. Answers whether the instruction is finished.
static int execute_setup_cycle(SNES_SPC700* cpu, SNES_APU* apu, const MEM_FORM* form)
{
	switch(form->mode)
	{
	case AM_IMPLIED:
	{
		// The byte after the opcode is read and thrown away. Whatever the instruction
		// computes happens inside the chip, over as many cycles as it takes, and lands
		// on the last of them.
		const UINT8 last = (UINT8)(1 + form->internals);
		if (cpu->tcu == 1) discard_next_byte(cpu, apu);
		if (cpu->tcu != last) return 0;
		apply_implied(cpu, cpu->ir);
		return 1;
	}

	case AM_DP:
		cpu->ea = dp_addr(cpu, fetch(cpu, apu));
		return 0;

	case AM_DPX:
	case AM_DPY:
		if (cpu->tcu == 1)
		{
			cpu->tmp = fetch(cpu, apu);
			return 0;
		}
		// The indexing cycle: the addition happens inside the chip, so the cycle
		// reaches memory not at all.
		cpu->ea = dp_addr(cpu, (UINT8)(cpu->tmp + (form->mode == AM_DPX ? cpu->x : cpu->y)));
		return 0;

	case AM_ABS:
	case AM_ABSX:
	case AM_ABSY:
	case AM_ABSBIT:
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->ptr = (UINT16)(cpu->ptr | (fetch(cpu, apu) << 8));
			// A plain absolute operand is the address itself. A bit operand carries the
			// address in its low 13 bits and the bit index in the three above them, so the
			// whole operand stays in the pointer scratch and only the address reaches memory.
			if (form->mode == AM_ABS) cpu->ea = cpu->ptr;
			if (form->mode == AM_ABSBIT)
			{
				cpu->ea = (UINT16)(cpu->ptr & 0x1FFFu);
			}
			return 0;
		}
		cpu->ea = (UINT16)(cpu->ptr + (form->mode == AM_ABSX ? cpu->x : cpu->y));
		return 0;

	case AM_INDIRECT:
	case AM_INDIRECT_INC:
		// X is the whole address here, so the byte after the opcode is read and thrown
		// away rather than being an operand.
		discard_next_byte(cpu, apu);
		cpu->ea = dp_addr(cpu, cpu->x);
		return 0;

	case AM_INDEXED_INDIRECT:
	case AM_INDIRECT_INDEXED:
	{
		UINT8 pointer_cycle;
		if (cpu->tcu == 1)
		{
			cpu->tmp = fetch(cpu, apu);
			return 0;
		}
		pointer_cycle = waits_before_pointer(form) ? 3 : 2;
		if (cpu->tcu == pointer_cycle)
		{
			cpu->ptr = snes_apu_bus_read(apu, pointer_addr(cpu, form, 0));
			return 0;
		}
		if (cpu->tcu == pointer_cycle + 1)
		{
			cpu->ptr = (UINT16)(cpu->ptr | (snes_apu_bus_read(apu, pointer_addr(cpu, form, 1)) << 8));
			cpu->ea = form->mode == AM_INDEXED_INDIRECT
						? cpu->ptr
						: (UINT16)(cpu->ptr + cpu->y);
			return 0;
		}
		// The internal cycle, which sits before the pointer or after it by the mode.
		return 0;
	}

	case AM_INDIRECT_TO_INDIRECT:
		// X names the target and Y the source. X addresses the target on its own, so the
		// byte after the opcode is read and thrown away as it is for every (X) form.
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			cpu->ea = dp_addr(cpu, cpu->x);
			return 0;
		}
		cpu->ptr = snes_apu_bus_read(apu, dp_addr(cpu, cpu->y));
		return 0;

	case AM_DP_TO_DP:
		// The source offset, then the byte it names, then the destination offset. The
		// source byte takes the pointer scratch once the offset that found it is spent.
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->ptr = snes_apu_bus_read(apu, dp_addr(cpu, (UINT8)cpu->ptr));
			return 0;
		}
		cpu->ea = dp_addr(cpu, fetch(cpu, apu));
		return 0;

	case AM_IMMEDIATE_TO_DP:
		// The source byte is in the instruction, so only the destination offset follows.
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		cpu->ea = dp_addr(cpu, fetch(cpu, apu));
		return 0;

	case AM_IMMEDIATE:
		break;	// its operand fetch is the access itself, so it has no setup cycle
	}
	return 0;
}

// Once the address is settled the instruction reaches memory: one cycle to read the
// byte, or two to reach the destination and settle there. Register work lands on the
// instruction's last cycle, so a snapshot taken mid-instruction never holds a register
// the hardware has not written yet. Executes one cycle of a memory instruction.
static int execute_memory_cycle(SNES_SPC700* cpu, SNES_APU* apu, const MEM_FORM* form)
{
	const UINT8 setup = (UINT8)(setup_cycles(form->mode) + form->internals);
	UINT8 step;
	UINT8 destination;
	UINT8 source;

	if (cpu->tcu <= setup) return execute_setup_cycle(cpu, apu, form);

	step = (UINT8)(cpu->tcu - setup);
	if (form->mode == AM_IMMEDIATE)
	{
		apply_memory_read(cpu, cpu->ir, fetch(cpu, apu));
		return 1;
	}

	if (form->access == MA_READ)
	{
		if (step == 1)
		{
			const UINT8 value = snes_apu_bus_read(apu, cpu->ea);
			if (form->mode != AM_INDIRECT_INC)
			{
				apply_memory_read(cpu, cpu->ir, value);
				return 1;
			}
			cpu->tmp = value;
			return 0;
		}
		// (X)+ spends a last cycle stepping X, and the load settles with it.
		apply_memory_read(cpu, cpu->ir, (UINT8)cpu->tmp);
		++ cpu->x;
		return 1;
	}

	// The cycles between the settled address and the write. Most forms read the byte
	// there once: a modify needs it, a plain store discards it but still makes the
	// access. The test-and-set pair and MOV1 m.b,C read it and then spend a cycle inside
	// the chip, MOV (X)+,A spends its one such cycle inside the chip, and the two-operand
	// moves have no such cycle at all.
	if (step <= destination_cycles(form->destination))
	{
		const int reaches_memory =
			form->destination == DC_READ ||
			form->destination == DC_READ_THEN_READ ||
			(form->destination == DC_READ_THEN_WAIT && step == 1);
		// The test-and-set pair reaches its operand on both of these cycles and keeps
		// the first byte: the second access drives the bus and its value is discarded.
		const int keeps_value =
			! (form->destination == DC_READ_THEN_READ && step == 2);
		if (reaches_memory)
		{
			const UINT8 value = snes_apu_bus_read(apu, cpu->ea);
			if (keeps_value) cpu->tmp = value;
		}
		return 0;
	}

	destination = (UINT8)cpu->tmp;
	source = (UINT8)cpu->ptr;
	switch(form->access)
	{
	case MA_WRITE:
		// A two-operand move carries the byte it writes; every other store writes a
		// register.
		snes_apu_bus_write(apu, cpu->ea,
							two_operand(form->mode) ? source : memory_store_value(cpu, cpu->ir));
		if (form->mode == AM_INDIRECT_INC) ++ cpu->x;
		break;
	case MA_MODIFY:
		snes_apu_bus_write(apu, cpu->ea, memory_modify_value(cpu, cpu->ir, destination, source));
		break;
	case MA_INTERNAL:
		// The result settles inside the chip, so this cycle reaches memory not at all.
		apply_internal_result(cpu, cpu->ir, destination, source);
		break;
	case MA_READ:
		break;	// a read settles on the cycle above, never here
	}
	return 1;
}

// A direct-page word instruction, one cycle at a time. One offset settles both
// addresses at once (the low byte's, and the high byte's one past it, wrapped inside
// the page) and from there the shapes differ only in what happens between the two
// bytes.
static int execute_word_cycle(SNES_SPC700* cpu, SNES_APU* apu, UINT8 form)
{
	UINT8 step;

	if (cpu->tcu == 1)
	{
		const UINT8 offset = fetch(cpu, apu);
		cpu->ea = dp_addr(cpu, offset);
		cpu->ptr = dp_addr(cpu, (UINT8)(offset + 1));
		return 0;
	}

	step = (UINT8)(cpu->tcu - 1);
	switch(form)
	{
	case WF_READ:
	case WF_COMPARE:
		// Both read the low byte and then the high one; the arithmetic and move forms
		// spend a cycle inside the chip between them, and the comparison does not.
		if (step == 1)
		{
			cpu->tmp = snes_apu_bus_read(apu, cpu->ea);
			return 0;
		}
		if (form == WF_READ && step == 2) return 0;
		apply_word_read(cpu, cpu->ir, (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, cpu->ptr) << 8)));
		return 1;

	case WF_MODIFY:
	{
		// Each half is written back before the next is read, so the two addresses are
		// reached in the order the chip reaches them. The high byte moves only when the
		// low one wrapped past its own end, and the flags are those of the whole word.
		const int increment = cpu->ir == 0x3A;	// INCW dp; DECW dp is 0x1A
		UINT8 low;
		UINT8 read;
		int carries;
		UINT8 high;
		if (step == 1)
		{
			cpu->tmp = snes_apu_bus_read(apu, cpu->ea);
			return 0;
		}
		if (step == 2)
		{
			low = (UINT8)(increment ? cpu->tmp + 1 : cpu->tmp - 1);
			snes_apu_bus_write(apu, cpu->ea, low);
			cpu->tmp = low;
			return 0;
		}
		if (step == 3)
		{
			cpu->tmp = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, cpu->ptr) << 8));
			return 0;
		}
		low = (UINT8)cpu->tmp;
		read = (UINT8)(cpu->tmp >> 8);
		carries = increment ? low == 0x00 : low == 0xFF;
		high = (UINT8)(carries ? (increment ? read + 1 : read - 1) : read);
		snes_apu_bus_write(apu, cpu->ptr, high);
		set_nz16(cpu, (UINT16)(low | (high << 8)));
		return 1;
	}

	case WF_STORE:
		// The low byte is read and discarded before either half is written, so a store
		// through this instruction clears a register that clears when read, but only the
		// one at the low address. Neither write sets a flag.
		if (step == 1)
		{
			(void)snes_apu_bus_read(apu, cpu->ea);
			return 0;
		}
		if (step == 2)
		{
			snes_apu_bus_write(apu, cpu->ea, cpu->a);
			return 0;
		}
		snes_apu_bus_write(apu, cpu->ptr, cpu->y);
		return 1;

	case WF_NONE:
		break;	// execute_cycle reaches this path only for the shapes above
	}
	return 1;
}

// A control-flow instruction, one cycle at a time. Two laws run through the family.
// A branch's condition is settled on the cycle that reads what it tests, so the
// cycles it prices afterwards are already known to be spent or not. And the program
// counter, like every other register, moves on the instruction's last cycle, which
// is why a stack address is measured from where the stack pointer began, and why a
// return address goes onto the stack before the destination reaches the core.
static int execute_control_cycle(SNES_SPC700* cpu, SNES_APU* apu, const CTRL_FORM* form)
{
	switch(form->kind)
	{
	// ---- the relative branches. The displacement arrives on the second cycle and
	//      the condition settles with it; a taken branch spends two more cycles
	//      inside the chip before the program counter moves ----
	case CK_BRANCH:
		if (cpu->tcu == 1)
		{
			cpu->tmp = fetch(cpu, apu);
			cpu->taken = (UINT8)branch_taken(cpu, form);
			return ! cpu->taken;
		}
		if (cpu->tcu == 2) return 0;
		take_branch(cpu);
		return 1;

	// ---- BBS, BBC and CBNE dp: the direct-page byte is read, a cycle passes inside
	//      the chip, and only then does the displacement follow at PC+2 ----
	case CK_BIT_BRANCH:
		if (cpu->tcu == 1)
		{
			cpu->ea = dp_addr(cpu, fetch(cpu, apu));
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->taken = (UINT8)byte_branch_taken(cpu, cpu->ir, snes_apu_bus_read(apu, cpu->ea));
			return 0;
		}
		if (cpu->tcu == 3) return 0;
		if (cpu->tcu == 4)
		{
			cpu->tmp = fetch(cpu, apu);
			return ! cpu->taken;
		}
		if (cpu->tcu == 5) return 0;
		take_branch(cpu);
		return 1;

	// ---- CBNE dp+X: the indexing cycle comes before the byte, as it does in every
	//      indexed mode, which puts the whole instruction one cycle behind CBNE dp ----
	case CK_COMPARE_INDEXED:
		if (cpu->tcu == 1)
		{
			cpu->tmp = fetch(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->ea = dp_addr(cpu, (UINT8)(cpu->tmp + cpu->x));
			return 0;
		}
		if (cpu->tcu == 3)
		{
			cpu->taken = (UINT8)byte_branch_taken(cpu, cpu->ir, snes_apu_bus_read(apu, cpu->ea));
			return 0;
		}
		if (cpu->tcu == 4) return 0;
		if (cpu->tcu == 5)
		{
			cpu->tmp = fetch(cpu, apu);
			return ! cpu->taken;
		}
		if (cpu->tcu == 6) return 0;
		take_branch(cpu);
		return 1;

	// ---- DBNZ dp: the byte goes back decremented before the displacement is read,
	//      so the store lands whether or not the branch is taken ----
	case CK_DECREMENT_DP:
		if (cpu->tcu == 1)
		{
			cpu->ea = dp_addr(cpu, fetch(cpu, apu));
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->tmp = snes_apu_bus_read(apu, cpu->ea);
			return 0;
		}
		if (cpu->tcu == 3)
		{
			const UINT8 value = (UINT8)(cpu->tmp - 1);
			snes_apu_bus_write(apu, cpu->ea, value);
			cpu->taken = value != 0;
			return 0;
		}
		if (cpu->tcu == 4)
		{
			cpu->tmp = fetch(cpu, apu);
			return ! cpu->taken;
		}
		if (cpu->tcu == 5) return 0;
		take_branch(cpu);
		return 1;

	// ---- DBNZ Y: the displacement is read, a cycle passes on the decrement, and the
	//      displacement's own address is read a second time. Y settles at the end ----
	case CK_DECREMENT_Y:
		if (cpu->tcu == 1)
		{
			cpu->ea = cpu->pc;
			cpu->tmp = fetch(cpu, apu);
			cpu->taken = (UINT8)(cpu->y - 1) != 0;
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		if (cpu->tcu == 3)
		{
			(void)snes_apu_bus_read(apu, cpu->ea);
			if (cpu->taken) return 0;
			-- cpu->y;
			return 1;
		}
		if (cpu->tcu == 4) return 0;
		-- cpu->y;
		take_branch(cpu);
		return 1;

	// ---- JMP !abs: the operand is the destination, and no cycle is spent inside the
	//      chip at all: the shortest instruction that moves the program counter ----
	case CK_JUMP:
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		cpu->pc = (UINT16)(cpu->ptr | (fetch(cpu, apu) << 8));
		return 1;

	// ---- JMP [!abs+X]: the operand plus X addresses a pointer, and that pointer's
	//      two bytes are one LINEAR byte apart. It is the one address in the core
	//      that does not wrap inside a page ----
	case CK_JUMP_INDEXED:
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->ptr = (UINT16)(cpu->ptr | (fetch(cpu, apu) << 8));
			return 0;
		}
		if (cpu->tcu == 3)
		{
			cpu->ea = (UINT16)(cpu->ptr + cpu->x);
			return 0;
		}
		if (cpu->tcu == 4)
		{
			cpu->tmp = snes_apu_bus_read(apu, cpu->ea);
			return 0;
		}
		cpu->pc = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, (UINT16)(cpu->ea + 1)) << 8));
		return 1;

	// ---- CALL !abs: the destination arrives first, then the return address goes on
	//      the stack high byte first, and two cycles pass inside the chip ----
	case CK_CALL:
		if (cpu->tcu == 1)
		{
			cpu->ptr = fetch(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			cpu->ptr = (UINT16)(cpu->ptr | (fetch(cpu, apu) << 8));
			return 0;
		}
		if (cpu->tcu == 3) return 0;
		if (cpu->tcu == 4)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, 0), (UINT8)(cpu->pc >> 8));
			return 0;
		}
		if (cpu->tcu == 5)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, -1), (UINT8)cpu->pc);
			return 0;
		}
		if (cpu->tcu == 6) return 0;
		cpu->sp = (UINT8)(cpu->sp - 2);
		cpu->pc = cpu->ptr;
		return 1;

	// ---- PCALL: the destination is one byte into page $FF, so the pushes begin a
	//      cycle earlier than CALL's and only one trails them ----
	case CK_CALL_PAGE:
		if (cpu->tcu == 1)
		{
			cpu->ptr = (UINT16)(0xFF00u | fetch(cpu, apu));
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		if (cpu->tcu == 3)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, 0), (UINT8)(cpu->pc >> 8));
			return 0;
		}
		if (cpu->tcu == 4)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, -1), (UINT8)cpu->pc);
			return 0;
		}
		cpu->sp = (UINT8)(cpu->sp - 2);
		cpu->pc = cpu->ptr;
		return 1;

	// ---- TCALL: the byte after the opcode is read and discarded, a cycle passes,
	//      the return address goes on the stack, and only after another cycle is the
	//      destination read out of the table ----
	case CK_CALL_VECTOR:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		if (cpu->tcu == 3)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, 0), (UINT8)(cpu->pc >> 8));
			return 0;
		}
		if (cpu->tcu == 4)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, -1), (UINT8)cpu->pc);
			return 0;
		}
		if (cpu->tcu == 5) return 0;
		if (cpu->tcu == 6)
		{
			cpu->tmp = snes_apu_bus_read(apu, call_vector(cpu));
			return 0;
		}
		cpu->pc = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, (UINT16)(call_vector(cpu) + 1)) << 8));
		cpu->sp = (UINT8)(cpu->sp - 2);
		return 1;

	// ---- BRK: the same table entry TCALL 0 reaches, with the status byte pushed
	//      under the return address, and the three pushes begin immediately, where
	//      TCALL spends a cycle inside the chip first ----
	case CK_BREAK:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, 0), (UINT8)(cpu->pc >> 8));
			return 0;
		}
		if (cpu->tcu == 3)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, -1), (UINT8)cpu->pc);
			return 0;
		}
		if (cpu->tcu == 4)
		{
			// The status byte as it stands now: the break and interrupt flags move at
			// the end of the instruction, after the byte is already on the stack.
			snes_apu_bus_write(apu, stack_addr(cpu, -2), cpu->psw);
			return 0;
		}
		if (cpu->tcu == 5) return 0;
		if (cpu->tcu == 6)
		{
			cpu->tmp = snes_apu_bus_read(apu, call_vector(cpu));
			return 0;
		}
		cpu->pc = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, (UINT16)(call_vector(cpu) + 1)) << 8));
		cpu->sp = (UINT8)(cpu->sp - 3);
		cpu->psw = (UINT8)((cpu->psw | SPC700_FLAG_B) & ~SPC700_FLAG_I);
		return 1;

	// ---- RET: a cycle inside the chip, then the return address off the stack, low
	//      byte first, the order it was pushed in ----
	case CK_RETURN:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		if (cpu->tcu == 3)
		{
			cpu->tmp = snes_apu_bus_read(apu, stack_addr(cpu, 1));
			return 0;
		}
		cpu->pc = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, stack_addr(cpu, 2)) << 8));
		cpu->sp = (UINT8)(cpu->sp + 2);
		return 1;

	// ---- RETI: the same, with the status byte coming back first ----
	case CK_RETURN_INTERRUPT:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		if (cpu->tcu == 3)
		{
			cpu->ptr = snes_apu_bus_read(apu, stack_addr(cpu, 1));
			return 0;
		}
		if (cpu->tcu == 4)
		{
			cpu->tmp = snes_apu_bus_read(apu, stack_addr(cpu, 2));
			return 0;
		}
		cpu->pc = (UINT16)(cpu->tmp | (snes_apu_bus_read(apu, stack_addr(cpu, 3)) << 8));
		cpu->psw = (UINT8)cpu->ptr;
		cpu->sp = (UINT8)(cpu->sp + 3);
		return 1;

	// ---- the stack transfers, which are mirror images: a push writes and then
	//      spends a cycle inside the chip, a pop spends the cycle and then reads ----
	case CK_PUSH:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2)
		{
			snes_apu_bus_write(apu, stack_addr(cpu, 0), push_value(cpu, cpu->ir));
			return 0;
		}
		-- cpu->sp;
		return 1;

	case CK_POP:
		if (cpu->tcu == 1)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu == 2) return 0;
		apply_pop(cpu, cpu->ir, snes_apu_bus_read(apu, stack_addr(cpu, 1)));
		++ cpu->sp;
		return 1;

	// ---- SLEEP and STOP: the byte after the opcode is read three times over, each
	//      read followed by a cycle inside the chip, and the core halts as the last
	//      of them ends: on an instruction boundary, with the program counter left
	//      on the byte it kept reading ----
	case CK_HALT:
		if (cpu->tcu == 1 || cpu->tcu == 3 || cpu->tcu == 5)
		{
			discard_next_byte(cpu, apu);
			return 0;
		}
		if (cpu->tcu != 6) return 0;
		cpu->run = cpu->ir == 0xEF ? SPC700_RUN_SLEEPING : SPC700_RUN_STOPPED;
		return 1;
	}
	return 1;
}

// Executes one cycle of the instruction already in the instruction register, and
// answers whether that cycle was its last. The cycle index says which cycle is due;
// every branch a cycle can take is decided by state the earlier cycles latched, so
// a handler never has to look ahead.
static int execute_cycle(SNES_SPC700* cpu, SNES_APU* apu)
{
	MEM_FORM form;
	CTRL_FORM control;
	UINT8 word = word_form(cpu->ir);

	if (word != WF_NONE)
		return execute_word_cycle(cpu, apu, word);
	form.mode = AM_IMPLIED;
	form.access = MA_READ;
	form.destination = DC_READ;
	form.internals = 0;
	if (memory_form(cpu->ir, &form))
		return execute_memory_cycle(cpu, apu, &form);
	control.kind = CK_BRANCH;
	control.flag = 0;
	control.when_set = 0;
	(void)control_form(cpu->ir, &control);
	return execute_control_cycle(cpu, apu, &control);
}

void snes_spc700_step_cycle(SNES_SPC700* cpu, SNES_APU* apu)
{
	// A sleeping or stopped core reaches memory not at all; the cycle still passes, and
	// the machine around it keeps time.
	if (cpu->run != SPC700_RUN_RUNNING) return;

	if (snes_spc700_at_boundary(cpu))
	{
		cpu->ir = fetch(cpu, apu);
		cpu->tcu = 1;
		return;
	}

	cpu->tcu = execute_cycle(cpu, apu) ? (UINT8)0 : (UINT8)(cpu->tcu + 1);
}
