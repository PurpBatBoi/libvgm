/***************************************************************************
    SPDX-License-Identifier: ISC
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

    MOS 6502 (NES 2A03) instruction-set core

    C port of ares' ares/component/processor/mos6502 (mos6502.hpp, memory.cpp,
    addresses.cpp, algorithms.cpp, instruction.cpp, instructions.cpp).
    https://github.com/ares-emulator/ares

    Port notes:
    - The C++ class is the NSF_CPU6502 struct, its virtual read / write are the two
      function pointers of the struct (plus a context pointer). The member function
      pointers of `instructionLoad(addr mode, algorithm alg)` are plain function
      pointers; the 256 row `switch` of the C++ is kept, so every row passes constants
      and the compiler specialises each one (a run-time opcode table was 4 to 8 %
      slower than the C++).
    - The bus use of every instruction (address, order, dummy reads and writes) is the
      C++'s. The C++ `idle()` is a read of PC whose result is dropped, and so it is here.
    - Deliberate changes from ares:
      - No decimal mode. ares' fc/cpu/cpu.cpp sets `MOS6502::BCD = 0` (the 2A03 has none),
        so ADC and SBC only have the binary path here. The D flag exists and is pushed
        and pulled as in the C++, it just has no effect.
    - Not ported: the interrupt hooks (`lastCycle`, `cancelNmi`, `delayIrq`, `irqPending`,
      `nmi`) and `interrupt()` / `reset()` (the NSF player has no interrupts and sets the
      registers itself; the NMI-vector override and the interrupt-delay of a taken
      branch go with them, they change nothing without a caller), `readDebugger`, the
      disassembler and the serializer.
    - `PC--` of the JAM opcodes and the other C++ integer behaviour (8 and 16 bit wrap
      at every store, `JMP ($xxFF)` reading the high byte from the same page, the 9-bit
      compare of CMP / CPX / CPY / AXS) are kept as they are.

***************************************************************************/

#include "../stdtype.h"
#include "../common_def.h"
#include "nsf_cpu6502.h"


typedef UINT16 (*ADDR_MODE)(NSF_CPU6502* cpu);
typedef void (*ALGORITHM)(NSF_CPU6502* cpu);


//----------------------------------------------------------------------------
// memory.cpp
//----------------------------------------------------------------------------

INLINE UINT8 rd(NSF_CPU6502* cpu, UINT16 address)
{
	return cpu->read(cpu->ctx, address);
}

INLINE void wr(NSF_CPU6502* cpu, UINT16 address, UINT8 data)
{
	cpu->write(cpu->ctx, address, data);
}

INLINE void idle(NSF_CPU6502* cpu)
{
	rd(cpu, cpu->pc);
}

INLINE void idle_zero_page(NSF_CPU6502* cpu, UINT8 address)
{
	rd(cpu, address);
}

INLINE void idle_page_crossed(NSF_CPU6502* cpu, UINT16 x, UINT16 y)
{
	if ((x >> 8) == (y >> 8))
		return;
	rd(cpu, (UINT16)((x & 0xFF00) | (y & 0x00FF)));
}

INLINE void idle_page_always(NSF_CPU6502* cpu, UINT16 x, UINT16 y)
{
	rd(cpu, (UINT16)((x & 0xFF00) | (y & 0x00FF)));
}

INLINE UINT8 opcode(NSF_CPU6502* cpu)
{
	return rd(cpu, cpu->pc ++);
}

INLINE UINT8 operand(NSF_CPU6502* cpu)
{
	return rd(cpu, cpu->pc ++);
}

INLINE UINT8 load(NSF_CPU6502* cpu, UINT8 address)
{
	return rd(cpu, address);
}

INLINE void push(NSF_CPU6502* cpu, UINT8 data)
{
	wr(cpu, (UINT16)(0x0100 | cpu->s), data);
	cpu->s --;
}

INLINE UINT8 pull(NSF_CPU6502* cpu)
{
	cpu->s ++;
	return rd(cpu, (UINT16)(0x0100 | cpu->s));
}


//----------------------------------------------------------------------------
// the P register and the flags every instruction sets
//----------------------------------------------------------------------------

UINT8 nsf_cpu6502_get_p(const NSF_CPU6502* cpu)
{
	return (UINT8)(cpu->c << 0 | cpu->z << 1 | cpu->i << 2 | cpu->d << 3 | cpu->v << 6 | cpu->n << 7);
}

void nsf_cpu6502_set_p(NSF_CPU6502* cpu, UINT8 data)
{
	cpu->c = (data >> 0) & 1;
	cpu->z = (data >> 1) & 1;
	cpu->i = (data >> 2) & 1;
	cpu->d = (data >> 3) & 1;
	cpu->v = (data >> 6) & 1;
	cpu->n = (data >> 7) & 1;
}

// Z = value == 0, N = bit 7 of value
INLINE void set_nz(NSF_CPU6502* cpu, UINT8 value)
{
	cpu->z = (value == 0);
	cpu->n = (UINT8)(value >> 7);
}

INLINE UINT16 set_pc_low(UINT16 pc, UINT8 data)
{
	return (UINT16)((pc & 0xFF00) | data);
}

INLINE UINT16 set_pc_high(UINT16 pc, UINT8 data)
{
	return (UINT16)((pc & 0x00FF) | (data << 8));
}


//----------------------------------------------------------------------------
// addresses.cpp
//----------------------------------------------------------------------------

static UINT16 addr_Absolute(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	absolute |= (UINT16)(operand(cpu) << 8);
	return absolute;
}

static UINT16 addr_AbsoluteJMP(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	// JMP Indirect last cycle
	absolute |= (UINT16)(operand(cpu) << 8);
	return absolute;
}

static UINT16 addr_AbsoluteXRead(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	UINT16 address;

	absolute |= (UINT16)(operand(cpu) << 8);
	address = (UINT16)(absolute + cpu->x);
	idle_page_crossed(cpu, absolute, address);
	return address;
}

static UINT16 addr_AbsoluteXWrite(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	UINT16 address;

	absolute |= (UINT16)(operand(cpu) << 8);
	address = (UINT16)(absolute + cpu->x);
	idle_page_always(cpu, absolute, address);
	return address;
}

static UINT16 addr_AbsoluteYRead(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	UINT16 address;

	absolute |= (UINT16)(operand(cpu) << 8);
	address = (UINT16)(absolute + cpu->y);
	idle_page_crossed(cpu, absolute, address);
	return address;
}

static UINT16 addr_AbsoluteYWrite(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	UINT16 address;

	absolute |= (UINT16)(operand(cpu) << 8);
	address = (UINT16)(absolute + cpu->y);
	idle_page_always(cpu, absolute, address);
	return address;
}

static UINT16 addr_Immediate(NSF_CPU6502* cpu)
{
	return cpu->pc ++;
}

static UINT16 addr_Implied(NSF_CPU6502* cpu)
{
	return cpu->pc;
}

static UINT16 addr_Indirect(NSF_CPU6502* cpu)
{
	UINT16 absolute = operand(cpu);
	UINT16 indirect;

	absolute |= (UINT16)(operand(cpu) << 8);
	indirect = rd(cpu, absolute);
	// only the low byte of the pointer is incremented: no carry into the page
	absolute = (UINT16)((absolute & 0xFF00) | ((absolute + 1) & 0x00FF));
	// JMP Indirect last cycle
	indirect |= (UINT16)(rd(cpu, absolute) << 8);
	return indirect;
}

static UINT16 addr_IndirectX(NSF_CPU6502* cpu)
{
	UINT8 zero_page = operand(cpu);
	UINT16 absolute;

	idle_zero_page(cpu, zero_page);
	absolute = load(cpu, (UINT8)(zero_page + cpu->x));
	absolute |= (UINT16)(load(cpu, (UINT8)(zero_page + cpu->x + 1)) << 8);
	return absolute;
}

static UINT16 addr_IndirectYRead(NSF_CPU6502* cpu)
{
	UINT8 zero_page = operand(cpu);
	UINT16 absolute;
	UINT16 address;

	absolute = load(cpu, (UINT8)(zero_page + 0));
	absolute |= (UINT16)(load(cpu, (UINT8)(zero_page + 1)) << 8);
	address = (UINT16)(absolute + cpu->y);
	idle_page_crossed(cpu, absolute, address);
	return address;
}

static UINT16 addr_IndirectYWrite(NSF_CPU6502* cpu)
{
	UINT8 zero_page = operand(cpu);
	UINT16 absolute;
	UINT16 address;

	absolute = load(cpu, (UINT8)(zero_page + 0));
	absolute |= (UINT16)(load(cpu, (UINT8)(zero_page + 1)) << 8);
	address = (UINT16)(absolute + cpu->y);
	idle_page_always(cpu, absolute, address);
	return address;
}

static UINT16 addr_Relative(NSF_CPU6502* cpu)
{
	return cpu->pc ++;
}

static UINT16 addr_ZeroPage(NSF_CPU6502* cpu)
{
	return operand(cpu);
}

static UINT16 addr_ZeroPageX(NSF_CPU6502* cpu)
{
	UINT8 zero_page = operand(cpu);

	idle_zero_page(cpu, zero_page);
	zero_page += cpu->x;
	return zero_page;
}

static UINT16 addr_ZeroPageY(NSF_CPU6502* cpu)
{
	UINT8 zero_page = operand(cpu);

	idle_zero_page(cpu, zero_page);
	zero_page += cpu->y;
	return zero_page;
}


//----------------------------------------------------------------------------
// algorithms.cpp
//----------------------------------------------------------------------------

// ADC, and SBC with the operand inverted. The C++ has the decimal branch here too, ares
// turns it off for the NES (BCD = 0), so only the binary one is ported.
static void add_with_carry(NSF_CPU6502* cpu, UINT8 i)
{
	UINT8 a = cpu->a;
	UINT16 o = (UINT16)(a + i + cpu->c);

	cpu->c = (UINT8)((o >> 8) & 1);
	cpu->z = ((o & 0xFF) == 0);
	cpu->n = (UINT8)((o >> 7) & 1);
	cpu->v = ((~(a ^ i) & (a ^ o) & 0x80) != 0);
	cpu->a = (UINT8)o;
}

// A, X or Y minus the operand: C is "no borrow", Z and N from the 8-bit result
static void compare(NSF_CPU6502* cpu, UINT8 reg)
{
	UINT16 o = (UINT16)((reg - cpu->mdr) & 0x1FF);

	cpu->c = (UINT8)(!((o >> 8) & 1));
	cpu->z = ((o & 0xFF) == 0);
	cpu->n = (UINT8)((o >> 7) & 1);
}

// official algorithms

static void alg_ADC(NSF_CPU6502* cpu)
{
	add_with_carry(cpu, cpu->mdr);
}

static void alg_AND(NSF_CPU6502* cpu)
{
	cpu->a &= cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_ASLA(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->a >> 7);
	cpu->a <<= 1;
	set_nz(cpu, cpu->a);
}

static void alg_ASLM(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->mdr >> 7);
	cpu->mdr <<= 1;
	set_nz(cpu, cpu->mdr);
}

static void algorithm_branch(NSF_CPU6502* cpu, int take)
{
	if (! take)
	{
		cpu->mdr = rd(cpu, cpu->mar);	// dummy relative read
	}
	else
	{
		UINT16 new_pc;

		cpu->mdr = rd(cpu, cpu->mar);
		new_pc = (UINT16)(cpu->pc + (INT8)cpu->mdr);

		// idle page crossed without carry
		if ((new_pc >> 8) != (cpu->pc >> 8))
			rd(cpu, (UINT16)((cpu->pc & 0xFF00) | (new_pc & 0x00FF)));

		cpu->pc = new_pc;
		// idle page crossed with carry
		idle(cpu);
	}
}

static void alg_BCC(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->c == 0);
}

static void alg_BCS(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->c == 1);
}

static void alg_BEQ(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->z == 1);
}

static void alg_BIT(NSF_CPU6502* cpu)
{
	cpu->z = ((cpu->a & cpu->mdr) == 0);
	cpu->v = (UINT8)((cpu->mdr >> 6) & 1);
	cpu->n = (UINT8)((cpu->mdr >> 7) & 1);
}

static void alg_BMI(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->n == 1);
}

static void alg_BNE(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->z == 0);
}

static void alg_BPL(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->n == 0);
}

static void alg_BRK(NSF_CPU6502* cpu)
{
	UINT16 vector = 0xFFFE;

	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	cpu->pc ++;
	push(cpu, (UINT8)(cpu->pc >> 8));
	push(cpu, (UINT8)(cpu->pc & 0xFF));

	push(cpu, (UINT8)(nsf_cpu6502_get_p(cpu) | 0x30));
	cpu->i = 1;

	cpu->pc = set_pc_low(cpu->pc, rd(cpu, vector ++));
	cpu->pc = set_pc_high(cpu->pc, rd(cpu, vector ++));
}

static void alg_BVC(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->v == 0);
}

static void alg_BVS(NSF_CPU6502* cpu)
{
	algorithm_branch(cpu, cpu->v == 1);
}

static void alg_CLC(NSF_CPU6502* cpu)
{
	cpu->c = 0;
}

static void alg_CLD(NSF_CPU6502* cpu)
{
	cpu->d = 0;
}

static void alg_CLI(NSF_CPU6502* cpu)
{
	cpu->i = 0;
}

static void alg_CLV(NSF_CPU6502* cpu)
{
	cpu->v = 0;
}

static void alg_CMP(NSF_CPU6502* cpu)
{
	compare(cpu, cpu->a);
}

static void alg_CPX(NSF_CPU6502* cpu)
{
	compare(cpu, cpu->x);
}

static void alg_CPY(NSF_CPU6502* cpu)
{
	compare(cpu, cpu->y);
}

static void alg_DEC(NSF_CPU6502* cpu)
{
	cpu->mdr --;
	set_nz(cpu, cpu->mdr);
}

static void alg_DEX(NSF_CPU6502* cpu)
{
	cpu->x --;
	set_nz(cpu, cpu->x);
}

static void alg_DEY(NSF_CPU6502* cpu)
{
	cpu->y --;
	set_nz(cpu, cpu->y);
}

static void alg_EOR(NSF_CPU6502* cpu)
{
	cpu->a ^= cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_INC(NSF_CPU6502* cpu)
{
	cpu->mdr ++;
	set_nz(cpu, cpu->mdr);
}

static void alg_INX(NSF_CPU6502* cpu)
{
	cpu->x ++;
	set_nz(cpu, cpu->x);
}

static void alg_INY(NSF_CPU6502* cpu)
{
	cpu->y ++;
	set_nz(cpu, cpu->y);
}

static void alg_JMP(NSF_CPU6502* cpu)
{
	cpu->pc = cpu->mar;
}

static void alg_JSR(NSF_CPU6502* cpu)
{
	UINT16 target = operand(cpu);

	cpu->mdr = rd(cpu, (UINT16)(0x0100 | cpu->s));	// dummy stack read
	push(cpu, (UINT8)(cpu->pc >> 8));
	push(cpu, (UINT8)(cpu->pc & 0xFF));
	target |= (UINT16)(operand(cpu) << 8);
	cpu->pc = target;
}

static void alg_LDA(NSF_CPU6502* cpu)
{
	cpu->a = cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_LDX(NSF_CPU6502* cpu)
{
	cpu->x = cpu->mdr;
	set_nz(cpu, cpu->x);
}

static void alg_LDY(NSF_CPU6502* cpu)
{
	cpu->y = cpu->mdr;
	set_nz(cpu, cpu->y);
}

static void alg_LSRA(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->a & 1);
	cpu->a >>= 1;
	set_nz(cpu, cpu->a);
}

static void alg_LSRM(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->mdr & 1);
	cpu->mdr >>= 1;
	set_nz(cpu, cpu->mdr);
}

static void alg_NOP(NSF_CPU6502* cpu)
{
}

static void alg_ORA(NSF_CPU6502* cpu)
{
	cpu->a |= cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_PHA(NSF_CPU6502* cpu)
{
	rd(cpu, cpu->mar);	// dummy implied read
	push(cpu, cpu->a);
}

static void alg_PHP(NSF_CPU6502* cpu)
{
	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	push(cpu, (UINT8)(nsf_cpu6502_get_p(cpu) | 0x30));
}

static void alg_PLA(NSF_CPU6502* cpu)
{
	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	cpu->mdr = rd(cpu, (UINT16)(0x0100 | cpu->s));	// dummy stack read
	cpu->a = pull(cpu);
	set_nz(cpu, cpu->a);
}

static void alg_PLP(NSF_CPU6502* cpu)
{
	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	cpu->mdr = rd(cpu, (UINT16)(0x0100 | cpu->s));	// dummy stack read
	nsf_cpu6502_set_p(cpu, pull(cpu));
}

static void alg_ROLA(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->a >> 7);
	cpu->a = (UINT8)(cpu->a << 1 | c);
	set_nz(cpu, cpu->a);
}

static void alg_ROLM(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->mdr >> 7);
	cpu->mdr = (UINT8)(cpu->mdr << 1 | c);
	set_nz(cpu, cpu->mdr);
}

static void alg_RORA(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->a & 1);
	cpu->a = (UINT8)(cpu->a >> 1 | c << 7);
	set_nz(cpu, cpu->a);
}

static void alg_RORM(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->mdr & 1);
	cpu->mdr = (UINT8)(cpu->mdr >> 1 | c << 7);
	set_nz(cpu, cpu->mdr);
}

static void alg_RTI(NSF_CPU6502* cpu)
{
	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	cpu->mdr = rd(cpu, (UINT16)(0x0100 | cpu->s));	// dummy stack read
	nsf_cpu6502_set_p(cpu, pull(cpu));
	cpu->pc = set_pc_low(cpu->pc, pull(cpu));
	cpu->pc = set_pc_high(cpu->pc, pull(cpu));
}

static void alg_RTS(NSF_CPU6502* cpu)
{
	cpu->mdr = rd(cpu, cpu->mar);	// dummy implied read
	cpu->mdr = rd(cpu, (UINT16)(0x0100 | cpu->s));	// dummy stack read
	cpu->pc = set_pc_low(cpu->pc, pull(cpu));
	cpu->pc = set_pc_high(cpu->pc, pull(cpu));
	idle(cpu);
	cpu->pc ++;
}

static void alg_SBC(NSF_CPU6502* cpu)
{
	add_with_carry(cpu, (UINT8)~cpu->mdr);
}

static void alg_SEC(NSF_CPU6502* cpu)
{
	cpu->c = 1;
}

static void alg_SED(NSF_CPU6502* cpu)
{
	cpu->d = 1;
}

static void alg_SEI(NSF_CPU6502* cpu)
{
	cpu->i = 1;
}

static void alg_STA(NSF_CPU6502* cpu)
{
	cpu->mdr = cpu->a;
}

static void alg_STX(NSF_CPU6502* cpu)
{
	cpu->mdr = cpu->x;
}

static void alg_STY(NSF_CPU6502* cpu)
{
	cpu->mdr = cpu->y;
}

static void alg_TAX(NSF_CPU6502* cpu)
{
	cpu->x = cpu->a;
	set_nz(cpu, cpu->x);
}

static void alg_TAY(NSF_CPU6502* cpu)
{
	cpu->y = cpu->a;
	set_nz(cpu, cpu->y);
}

static void alg_TSX(NSF_CPU6502* cpu)
{
	cpu->x = cpu->s;
	set_nz(cpu, cpu->x);
}

static void alg_TXA(NSF_CPU6502* cpu)
{
	cpu->a = cpu->x;
	set_nz(cpu, cpu->a);
}

static void alg_TXS(NSF_CPU6502* cpu)
{
	cpu->s = cpu->x;
}

static void alg_TYA(NSF_CPU6502* cpu)
{
	cpu->a = cpu->y;
	set_nz(cpu, cpu->a);
}

// unofficial algorithms

static void alg_AAC(NSF_CPU6502* cpu)
{
	cpu->a &= cpu->mdr;
	cpu->c = (UINT8)(cpu->a >> 7);
	set_nz(cpu, cpu->a);
}

static void alg_AAX(NSF_CPU6502* cpu)
{
	cpu->mdr = (UINT8)(cpu->a & cpu->x);
}

// Highly unstable, do not use.
// A base value in A is determined based on the
// contets of A and a constant, which may be
// typically $00, $ff, $ee, etc. The value of
// this constant depends on temerature, the chip
// series, and maybe other factors, as well.
// In order to eliminate these uncertaincies
// from the equation, use either 0 as the operand
// or a value of $FF in the accumulator.
static void alg_ANE(NSF_CPU6502* cpu)
{
	cpu->a = (UINT8)((cpu->a | 0xFF) & cpu->x & cpu->mdr);
	set_nz(cpu, cpu->a);
}

// This operation involves the adder:
// V-flag is set according to (A AND oper) + oper
// The carry is not set, but bit 7 (sign)
// is exchanged with the carry
static void alg_ARR(NSF_CPU6502* cpu)
{
	cpu->a = (UINT8)(((cpu->a & cpu->mdr) >> 1) | (cpu->c << 7));
	cpu->c = (UINT8)((cpu->a >> 6) & 1);
	set_nz(cpu, cpu->a);
	cpu->v = (UINT8)(cpu->c ^ ((cpu->a >> 5) & 1));
}

static void alg_ASR(NSF_CPU6502* cpu)
{
	cpu->a &= cpu->mdr;
	cpu->c = (UINT8)(cpu->a & 1);
	cpu->a >>= 1;
	set_nz(cpu, cpu->a);
}

// Highly unstable, involves a 'magic' constant, see ANE
static void alg_ATX(NSF_CPU6502* cpu)
{
	cpu->a = (UINT8)((cpu->a | 0xFF) & cpu->mdr);
	cpu->x = cpu->a;
	set_nz(cpu, cpu->a);
}

static void alg_AXS(NSF_CPU6502* cpu)
{
	UINT16 o;

	cpu->x &= cpu->a;
	o = (UINT16)((cpu->x - cpu->mdr) & 0x1FF);
	cpu->c = (UINT8)(!((o >> 8) & 1));
	cpu->x = (UINT8)o;
	set_nz(cpu, cpu->x);
}

static void alg_DCP(NSF_CPU6502* cpu)
{
	cpu->mdr --;
	alg_CMP(cpu);
}

static void alg_ISC(NSF_CPU6502* cpu)
{
	cpu->mdr ++;
	alg_SBC(cpu);
}

static void alg_LAS(NSF_CPU6502* cpu)
{
	cpu->a = (UINT8)(cpu->mdr & cpu->s);
	cpu->x = cpu->a;
	cpu->s = cpu->a;
	set_nz(cpu, cpu->a);
}

static void alg_LAX(NSF_CPU6502* cpu)
{
	cpu->a = cpu->mdr;
	cpu->x = cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_RLA(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->mdr >> 7);
	cpu->mdr = (UINT8)(cpu->mdr << 1 | c);
	cpu->a &= cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_RRA(NSF_CPU6502* cpu)
{
	UINT8 c = cpu->c;

	cpu->c = (UINT8)(cpu->mdr & 1);
	cpu->mdr = (UINT8)(cpu->mdr >> 1 | c << 7);
	alg_ADC(cpu);
}

static void alg_SLO(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->mdr >> 7);
	cpu->mdr <<= 1;
	cpu->a |= cpu->mdr;
	set_nz(cpu, cpu->a);
}

static void alg_SRE(NSF_CPU6502* cpu)
{
	cpu->c = (UINT8)(cpu->mdr & 1);
	cpu->mdr >>= 1;
	cpu->a ^= cpu->mdr;
	set_nz(cpu, cpu->a);
}

// unstable: sometimes 'AND (H+1)' is dropped,
// page boundary crossings may not work (with
// the high-byte of the value used as the
// high-byte of the address)
static void alg_SXA(NSF_CPU6502* cpu)
{
	cpu->mar = (UINT16)(((cpu->x & ((cpu->mar >> 8) + 1)) << 8) | (cpu->mar & 0xFF));
	cpu->mdr = (UINT8)(cpu->mar >> 8);
}

// unstable: sometimes 'AND (H+1)' is dropped,
// page boundary crossings may not work (with
// the high-byte of the value used as the
// high-byte of the address)
static void alg_SYA(NSF_CPU6502* cpu)
{
	cpu->mar = (UINT16)(((cpu->y & ((cpu->mar >> 8) + 1)) << 8) | (cpu->mar & 0xFF));
	cpu->mdr = (UINT8)(cpu->mar >> 8);
}

// unstable: sometimes 'AND (H+1)' is dropped,
// page boundary crossings may not work (with
// the high-byte of the value used as the
// high-byte of the address)
static void alg_TAS(NSF_CPU6502* cpu)
{
	cpu->s = (UINT8)(cpu->a & cpu->x);
	// (mar - y) can be negative (mar wrapped past $FFFF): the shift is an arithmetic one, as in the C++
	cpu->mdr = (UINT8)(cpu->a & cpu->x & (((cpu->mar - cpu->y) >> 8) + 1));
}

// These instructions freeze the CPU.
// The processor will be trapped infinitely
// in T1 phase with $FF on the data bus.
// Reset required.
static void alg_JAM(NSF_CPU6502* cpu)
{
	cpu->pc --;
}


//----------------------------------------------------------------------------
// instructions.cpp
//----------------------------------------------------------------------------

INLINE void instruction_none(NSF_CPU6502* cpu, ADDR_MODE mode, ALGORITHM alg)
{
	cpu->mar = mode(cpu);
	alg(cpu);
}

INLINE void instruction_load(NSF_CPU6502* cpu, ADDR_MODE mode, ALGORITHM alg)
{
	cpu->mar = mode(cpu);
	cpu->mdr = rd(cpu, cpu->mar);
	alg(cpu);
}

INLINE void instruction_store(NSF_CPU6502* cpu, ADDR_MODE mode, ALGORITHM alg)
{
	cpu->mar = mode(cpu);
	alg(cpu);
	wr(cpu, cpu->mar, cpu->mdr);
}

INLINE void instruction_modify(NSF_CPU6502* cpu, ADDR_MODE mode, ALGORITHM alg)
{
	cpu->mar = mode(cpu);
	cpu->mdr = rd(cpu, cpu->mar);
	wr(cpu, cpu->mar, cpu->mdr);	// dummy write
	alg(cpu);
	wr(cpu, cpu->mar, cpu->mdr);
}


//----------------------------------------------------------------------------
// instruction.cpp
//----------------------------------------------------------------------------

// one row of the C++ `op(id, kind, addr(mode), fp(algorithm))` switch. The mode and the algorithm are
// constants in every row, so the compiler specialises each `instruction_...` call, as it does for the C++.
#define OP(id, kind, mode, alg)	case id: instruction_##kind(cpu, addr_##mode, alg_##alg); break;

void nsf_cpu6502_instruction(NSF_CPU6502* cpu)
{
	switch(opcode(cpu))
	{
	// 0x00
	OP(0x00, none,   Implied,        BRK)
	OP(0x01, load,   IndirectX,      ORA)
	OP(0x02, none,   Implied,        JAM)
	OP(0x03, modify, IndirectX,      SLO)
	OP(0x04, load,   ZeroPage,       NOP)
	OP(0x05, load,   ZeroPage,       ORA)
	OP(0x06, modify, ZeroPage,       ASLM)
	OP(0x07, modify, ZeroPage,       SLO)
	OP(0x08, none,   Implied,        PHP)
	OP(0x09, load,   Immediate,      ORA)
	OP(0x0A, load,   Implied,        ASLA)
	OP(0x0B, load,   Immediate,      AAC)
	OP(0x0C, load,   Absolute,       NOP)
	OP(0x0D, load,   Absolute,       ORA)
	OP(0x0E, modify, Absolute,       ASLM)
	OP(0x0F, modify, Absolute,       SLO)
	// 0x10
	OP(0x10, none,   Relative,       BPL)
	OP(0x11, load,   IndirectYRead,  ORA)
	OP(0x12, none,   Implied,        JAM)
	OP(0x13, modify, IndirectYWrite, SLO)
	OP(0x14, load,   ZeroPageX,      NOP)
	OP(0x15, load,   ZeroPageX,      ORA)
	OP(0x16, modify, ZeroPageX,      ASLM)
	OP(0x17, modify, ZeroPageX,      SLO)
	OP(0x18, load,   Implied,        CLC)
	OP(0x19, load,   AbsoluteYRead,  ORA)
	OP(0x1A, load,   Implied,        NOP)
	OP(0x1B, modify, AbsoluteYWrite, SLO)
	OP(0x1C, load,   AbsoluteXRead,  NOP)
	OP(0x1D, load,   AbsoluteXRead,  ORA)
	OP(0x1E, modify, AbsoluteXWrite, ASLM)
	OP(0x1F, modify, AbsoluteXWrite, SLO)
	// 0x20
	OP(0x20, none,   Implied,        JSR)
	OP(0x21, load,   IndirectX,      AND)
	OP(0x22, none,   Implied,        JAM)
	OP(0x23, modify, IndirectX,      RLA)
	OP(0x24, load,   ZeroPage,       BIT)
	OP(0x25, load,   ZeroPage,       AND)
	OP(0x26, modify, ZeroPage,       ROLM)
	OP(0x27, modify, ZeroPage,       RLA)
	OP(0x28, none,   Implied,        PLP)
	OP(0x29, load,   Immediate,      AND)
	OP(0x2A, load,   Implied,        ROLA)
	OP(0x2B, load,   Immediate,      AAC)
	OP(0x2C, load,   Absolute,       BIT)
	OP(0x2D, load,   Absolute,       AND)
	OP(0x2E, modify, Absolute,       ROLM)
	OP(0x2F, modify, Absolute,       RLA)
	// 0x30
	OP(0x30, none,   Relative,       BMI)
	OP(0x31, load,   IndirectYRead,  AND)
	OP(0x32, none,   Implied,        JAM)
	OP(0x33, modify, IndirectYWrite, RLA)
	OP(0x34, load,   ZeroPageX,      NOP)
	OP(0x35, load,   ZeroPageX,      AND)
	OP(0x36, modify, ZeroPageX,      ROLM)
	OP(0x37, modify, ZeroPageX,      RLA)
	OP(0x38, load,   Implied,        SEC)
	OP(0x39, load,   AbsoluteYRead,  AND)
	OP(0x3A, load,   Implied,        NOP)
	OP(0x3B, modify, AbsoluteYWrite, RLA)
	OP(0x3C, load,   AbsoluteXRead,  NOP)
	OP(0x3D, load,   AbsoluteXRead,  AND)
	OP(0x3E, modify, AbsoluteXWrite, ROLM)
	OP(0x3F, modify, AbsoluteXWrite, RLA)
	// 0x40
	OP(0x40, none,   Implied,        RTI)
	OP(0x41, load,   IndirectX,      EOR)
	OP(0x42, none,   Implied,        JAM)
	OP(0x43, modify, IndirectX,      SRE)
	OP(0x44, load,   ZeroPage,       NOP)
	OP(0x45, load,   ZeroPage,       EOR)
	OP(0x46, modify, ZeroPage,       LSRM)
	OP(0x47, modify, ZeroPage,       SRE)
	OP(0x48, none,   Implied,        PHA)
	OP(0x49, load,   Immediate,      EOR)
	OP(0x4A, load,   Implied,        LSRA)
	OP(0x4B, load,   Immediate,      ASR)
	OP(0x4C, none,   AbsoluteJMP,    JMP)
	OP(0x4D, load,   Absolute,       EOR)
	OP(0x4E, modify, Absolute,       LSRM)
	OP(0x4F, modify, Absolute,       SRE)
	// 0x50
	OP(0x50, none,   Relative,       BVC)
	OP(0x51, load,   IndirectYRead,  EOR)
	OP(0x52, none,   Implied,        JAM)
	OP(0x53, modify, IndirectYWrite, SRE)
	OP(0x54, load,   ZeroPageX,      NOP)
	OP(0x55, load,   ZeroPageX,      EOR)
	OP(0x56, modify, ZeroPageX,      LSRM)
	OP(0x57, modify, ZeroPageX,      SRE)
	OP(0x58, load,   Implied,        CLI)
	OP(0x59, load,   AbsoluteYRead,  EOR)
	OP(0x5A, load,   Implied,        NOP)
	OP(0x5B, modify, AbsoluteYWrite, SRE)
	OP(0x5C, load,   AbsoluteXRead,  NOP)
	OP(0x5D, load,   AbsoluteXRead,  EOR)
	OP(0x5E, modify, AbsoluteXWrite, LSRM)
	OP(0x5F, modify, AbsoluteXWrite, SRE)
	// 0x60
	OP(0x60, none,   Implied,        RTS)
	OP(0x61, load,   IndirectX,      ADC)
	OP(0x62, none,   Implied,        JAM)
	OP(0x63, modify, IndirectX,      RRA)
	OP(0x64, load,   ZeroPage,       NOP)
	OP(0x65, load,   ZeroPage,       ADC)
	OP(0x66, modify, ZeroPage,       RORM)
	OP(0x67, modify, ZeroPage,       RRA)
	OP(0x68, none,   Implied,        PLA)
	OP(0x69, load,   Immediate,      ADC)
	OP(0x6A, load,   Implied,        RORA)
	OP(0x6B, load,   Immediate,      ARR)
	OP(0x6C, none,   Indirect,       JMP)
	OP(0x6D, load,   Absolute,       ADC)
	OP(0x6E, modify, Absolute,       RORM)
	OP(0x6F, modify, Absolute,       RRA)
	// 0x70
	OP(0x70, none,   Relative,       BVS)
	OP(0x71, load,   IndirectYRead,  ADC)
	OP(0x72, none,   Implied,        JAM)
	OP(0x73, modify, IndirectYWrite, RRA)
	OP(0x74, load,   ZeroPageX,      NOP)
	OP(0x75, load,   ZeroPageX,      ADC)
	OP(0x76, modify, ZeroPageX,      RORM)
	OP(0x77, modify, ZeroPageX,      RRA)
	OP(0x78, load,   Implied,        SEI)
	OP(0x79, load,   AbsoluteYRead,  ADC)
	OP(0x7A, load,   Implied,        NOP)
	OP(0x7B, modify, AbsoluteYWrite, RRA)
	OP(0x7C, load,   AbsoluteXRead,  NOP)
	OP(0x7D, load,   AbsoluteXRead,  ADC)
	OP(0x7E, modify, AbsoluteXWrite, RORM)
	OP(0x7F, modify, AbsoluteXWrite, RRA)
	// 0x80
	OP(0x80, load,   Immediate,      NOP)
	OP(0x81, store,  IndirectX,      STA)
	OP(0x82, load,   Immediate,      NOP)
	OP(0x83, store,  IndirectX,      AAX)
	OP(0x84, store,  ZeroPage,       STY)
	OP(0x85, store,  ZeroPage,       STA)
	OP(0x86, store,  ZeroPage,       STX)
	OP(0x87, store,  ZeroPage,       AAX)
	OP(0x88, load,   Implied,        DEY)
	OP(0x89, load,   Immediate,      NOP)
	OP(0x8A, load,   Implied,        TXA)
	OP(0x8B, load,   Immediate,      ANE)
	OP(0x8C, store,  Absolute,       STY)
	OP(0x8D, store,  Absolute,       STA)
	OP(0x8E, store,  Absolute,       STX)
	OP(0x8F, store,  Absolute,       AAX)
	// 0x90
	OP(0x90, none,   Relative,       BCC)
	OP(0x91, store,  IndirectYWrite, STA)
	OP(0x92, none,   Implied,        JAM)
	OP(0x93, store,  IndirectYWrite, AAX)
	OP(0x94, store,  ZeroPageX,      STY)
	OP(0x95, store,  ZeroPageX,      STA)
	OP(0x96, store,  ZeroPageY,      STX)
	OP(0x97, store,  ZeroPageY,      AAX)
	OP(0x98, load,   Implied,        TYA)
	OP(0x99, store,  AbsoluteYWrite, STA)
	OP(0x9A, load,   Implied,        TXS)
	OP(0x9B, store,  AbsoluteYWrite, TAS)
	OP(0x9C, store,  AbsoluteXWrite, SYA)
	OP(0x9D, store,  AbsoluteXWrite, STA)
	OP(0x9E, store,  AbsoluteYWrite, SXA)
	OP(0x9F, store,  AbsoluteYWrite, AAX)
	// 0xA0
	OP(0xA0, load,   Immediate,      LDY)
	OP(0xA1, load,   IndirectX,      LDA)
	OP(0xA2, load,   Immediate,      LDX)
	OP(0xA3, load,   IndirectX,      LAX)
	OP(0xA4, load,   ZeroPage,       LDY)
	OP(0xA5, load,   ZeroPage,       LDA)
	OP(0xA6, load,   ZeroPage,       LDX)
	OP(0xA7, load,   ZeroPage,       LAX)
	OP(0xA8, load,   Implied,        TAY)
	OP(0xA9, load,   Immediate,      LDA)
	OP(0xAA, load,   Implied,        TAX)
	OP(0xAB, load,   Immediate,      ATX)
	OP(0xAC, load,   Absolute,       LDY)
	OP(0xAD, load,   Absolute,       LDA)
	OP(0xAE, load,   Absolute,       LDX)
	OP(0xAF, load,   Absolute,       LAX)
	// 0xB0
	OP(0xB0, none,   Relative,       BCS)
	OP(0xB1, load,   IndirectYRead,  LDA)
	OP(0xB2, none,   Implied,        JAM)
	OP(0xB3, load,   IndirectYRead,  LAX)
	OP(0xB4, load,   ZeroPageX,      LDY)
	OP(0xB5, load,   ZeroPageX,      LDA)
	OP(0xB6, load,   ZeroPageY,      LDX)
	OP(0xB7, load,   ZeroPageY,      LAX)
	OP(0xB8, load,   Implied,        CLV)
	OP(0xB9, load,   AbsoluteYRead,  LDA)
	OP(0xBA, load,   Implied,        TSX)
	OP(0xBB, load,   AbsoluteYRead,  LAS)
	OP(0xBC, load,   AbsoluteXRead,  LDY)
	OP(0xBD, load,   AbsoluteXRead,  LDA)
	OP(0xBE, load,   AbsoluteYRead,  LDX)
	OP(0xBF, load,   AbsoluteYRead,  LAX)
	// 0xC0
	OP(0xC0, load,   Immediate,      CPY)
	OP(0xC1, load,   IndirectX,      CMP)
	OP(0xC2, load,   Immediate,      NOP)
	OP(0xC3, modify, IndirectX,      DCP)
	OP(0xC4, load,   ZeroPage,       CPY)
	OP(0xC5, load,   ZeroPage,       CMP)
	OP(0xC6, modify, ZeroPage,       DEC)
	OP(0xC7, modify, ZeroPage,       DCP)
	OP(0xC8, load,   Implied,        INY)
	OP(0xC9, load,   Immediate,      CMP)
	OP(0xCA, load,   Implied,        DEX)
	OP(0xCB, load,   Immediate,      AXS)
	OP(0xCC, load,   Absolute,       CPY)
	OP(0xCD, load,   Absolute,       CMP)
	OP(0xCE, modify, Absolute,       DEC)
	OP(0xCF, modify, Absolute,       DCP)
	// 0xD0
	OP(0xD0, none,   Relative,       BNE)
	OP(0xD1, load,   IndirectYRead,  CMP)
	OP(0xD2, none,   Implied,        JAM)
	OP(0xD3, modify, IndirectYWrite, DCP)
	OP(0xD4, load,   ZeroPageX,      NOP)
	OP(0xD5, load,   ZeroPageX,      CMP)
	OP(0xD6, modify, ZeroPageX,      DEC)
	OP(0xD7, modify, ZeroPageX,      DCP)
	OP(0xD8, load,   Implied,        CLD)
	OP(0xD9, load,   AbsoluteYRead,  CMP)
	OP(0xDA, load,   Implied,        NOP)
	OP(0xDB, modify, AbsoluteYWrite, DCP)
	OP(0xDC, load,   AbsoluteXRead,  NOP)
	OP(0xDD, load,   AbsoluteXRead,  CMP)
	OP(0xDE, modify, AbsoluteXWrite, DEC)
	OP(0xDF, modify, AbsoluteXWrite, DCP)
	// 0xE0
	OP(0xE0, load,   Immediate,      CPX)
	OP(0xE1, load,   IndirectX,      SBC)
	OP(0xE2, load,   Immediate,      NOP)
	OP(0xE3, modify, IndirectX,      ISC)
	OP(0xE4, load,   ZeroPage,       CPX)
	OP(0xE5, load,   ZeroPage,       SBC)
	OP(0xE6, modify, ZeroPage,       INC)
	OP(0xE7, modify, ZeroPage,       ISC)
	OP(0xE8, load,   Implied,        INX)
	OP(0xE9, load,   Immediate,      SBC)
	OP(0xEA, load,   Implied,        NOP)
	OP(0xEB, load,   Immediate,      SBC)
	OP(0xEC, load,   Absolute,       CPX)
	OP(0xED, load,   Absolute,       SBC)
	OP(0xEE, modify, Absolute,       INC)
	OP(0xEF, modify, Absolute,       ISC)
	// 0xF0
	OP(0xF0, none,   Relative,       BEQ)
	OP(0xF1, load,   IndirectYRead,  SBC)
	OP(0xF2, none,   Implied,        JAM)
	OP(0xF3, modify, IndirectYWrite, ISC)
	OP(0xF4, load,   ZeroPageX,      NOP)
	OP(0xF5, load,   ZeroPageX,      SBC)
	OP(0xF6, modify, ZeroPageX,      INC)
	OP(0xF7, modify, ZeroPageX,      ISC)
	OP(0xF8, load,   Implied,        SED)
	OP(0xF9, load,   AbsoluteYRead,  SBC)
	OP(0xFA, load,   Implied,        NOP)
	OP(0xFB, modify, AbsoluteYWrite, ISC)
	OP(0xFC, load,   AbsoluteXRead,  NOP)
	OP(0xFD, load,   AbsoluteXRead,  SBC)
	OP(0xFE, modify, AbsoluteXWrite, INC)
	OP(0xFF, modify, AbsoluteXWrite, ISC)
	}
}

#undef OP

//----------------------------------------------------------------------------
// mos6502.cpp
//----------------------------------------------------------------------------

void nsf_cpu6502_power(NSF_CPU6502* cpu)
{
	cpu->a = 0x00;
	cpu->x = 0x00;
	cpu->y = 0x00;
	cpu->s = 0x00;
	nsf_cpu6502_set_p(cpu, 0x04);
	cpu->mdr = 0x00;
}
