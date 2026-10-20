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

// SNES audio unit (SPC700 + S-DSP) and its libvgm device
//
// C port of Snaggletooth's APU machine and .spc loader (https://github.com/etroimcasso/
// Snaggletooth, pinned at commit efd5049):
//   include/snaggletooth/apu/apu.h
//   src/apu.cpp
//   tools/spc/spc_loader.h and tools/spc/spc_loader.cpp (parseSpc)
// The SPC700 is snes_spc700.c and the S-DSP is snes_dsp.c.
//
// The machine runs one cycle at a time. Within a cycle the clocked events go first
// (the master counter advances, the stage-1 timer ticks land, the DSP takes its slot)
// and the CPU's single bus access lands last. So a read of a timer output on the cycle
// that timer ticks returns the incremented count, and a DSP write lands on its true
// cycle relative to the 32-cycle sample boundary. One counter drives all of it: T0 and
// T1 tick every 128 cycles and T2 every 16, on the counter's documented slots; the DSP
// takes a sample every 32. Each timer's stage-2 counter increments on its stage-1 ticks
// while enabled and, on reaching its target, advances a 4-bit stage-3 counter that an
// overlay read of TnOUT returns and clears.
//
// The device (sndDev_SNES_APU, DEVID_SNES_APU) plays one .spc image:
// - The whole image goes in through the memory block write (RWF_MEMORY, DEVRW_BLOCK) at
//   offset 0. The first 0x10200 bytes are kept, and Reset restores the machine from them
//   (the loaded state, not the power-on one).
// - The native rate is 32000 Hz, the DSP's output rate; the player resamples. Update()
//   runs machine cycles until exactly `samples` frames are out. The 16-bit frames go to
//   the outputs unscaled.
// - RWF_CHN_MUTE sets the DSP's per-voice mute mask (8 channels).
// - Before an image is loaded the device is the power-on machine: muted and silent.
//
// Port notes:
// - ApuState and the live CPU are one struct (SNES_APU): the C++ keeps a snapshot of the
//   CPU in the state that is written back at the end of run()/step(), and reads the live
//   core everywhere else, so the one live copy behaves the same.
// - The bus (Apu::Bus, busRead, busWrite, readRegister, registerValue, writeRegister) is
//   snes_apu_bus_read() / snes_apu_bus_write() and the static register functions. The CPU
//   calls them directly (see snes_spc700.h).
// - parseSpc() is snes_apu_load_spc(), which fills the machine in place. The C++ builds an
//   ApuState{} with its member defaults and overlays the file's fields on it; here that
//   is the power-on seed of snes_dsp_init() plus zero, then the same overlay.
// - The frame queue (frames_ and takeFrames(span)) is gone: snes_apu_render() runs the
//   cycles until the requested number of frames has been delivered and writes each frame
//   into the caller's buffer the moment it is delivered, so nothing is allocated or kept.
// - Not ported (no supported caller uses them):
//   - The observer (ApuObserver), the access watcher and the instruction watcher with
//     their armed tables and stand-ins, callInContext() / callOnStack() and runCall(),
//     and insideCycle_: debug and host-call surface.
//   - The boot-ROM window at $FFC0 (mapIplRom, iplImage_, CONTROL bit 7 on reads): the
//     .spc path never maps an image, and without one the C++ bus reads plain RAM there.
//   - Apu::restore() / reload() / reset() as calls, state() / cpuState() / setCpuState(),
//     setPc(), peek() / peekRegister() / poke() / addressable(), the host port faces
//     (writePort / readPort), readRam / writeRam / loadRam, readOverlayRegister /
//     writeOverlayRegister, readDspRegister / writeDspRegister, the owning-vs-storage
//     constructors and the move constructor, step() (the machine is driven by cycles
//     only), and ApuState's byte form.
//   - The CPU's run state is always Running after a load, as in the C++ (the format has no
//     halt field).
// - Deliberate changes from the C++:
//   - The libvgm device and the mute mask (see above and snes_dsp.c); the C++ has no
//     device. The machine itself is unchanged.
//   - snes_apu_load_spc() keeps the DSP mute mask, which is not part of the C++ state.
//   - snes_apu_load_spc() writes the dumped KON byte back through the DSP's CPU write, so the
//     voices it names key on at the first poll. The C++ leaves the key-on latch clear and those
//     voices stay silent until the driver keys them again (the opening notes of most dumps).
//     blargg's snes_spc (and so libgme) arms its latch from KON at load the same way.
// - Inherited from the C++ (bit-identical, so not changed here):
//   - Snaggletooth marks the S-DSP and its .spc renderer "in progress": three blargg DSP
//     sub-tests fail upstream (envelope, key-on pitch, BRR while playing). A fix against the C++
//     is a deliberate change and needs a sign-off.
//   - An .spc image does not carry the DSP's per-voice streaming state, the envelopes, the noise
//     generator or the echo ring, nor the timer and comm-port state: they start from the power-on
//     shape, so a dump taken in the middle of a track restarts its voices. Dumps of single sound
//     effects can render silent (SMW "25 Fade Out!" and "26 Secret Exit!": silent in the C++ too).

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../../common_def.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "snes_spc700.h"
#include "snes_dsp.h"
#include "snes_apu.h"


// Field offsets in the .spc image (the v0.30/v0.31 layout: the two revisions agree
// on every machine-state field).
#define SPC_MAGIC			"SNES-SPC700 Sound File Data v"	// through 'v'
#define SPC_MARKER_OFFSET	0x0021	// two 26 bytes
#define SPC_PC_OFFSET		0x0025
#define SPC_A_OFFSET		0x0027
#define SPC_X_OFFSET		0x0028
#define SPC_Y_OFFSET		0x0029
#define SPC_PSW_OFFSET		0x002A
#define SPC_SP_OFFSET		0x002B
#define SPC_RAM_OFFSET		0x0100
#define SPC_DSP_OFFSET		0x10100
#define SPC_EXTRA_RAM_OFFSET	0x101C0
#define SPC_EXTRA_RAM_SIZE	64

// CONTROL ($F1) bit 7 maps the boot-ROM image over the $FFC0 window for SPC700
// reads: not ported, see the port notes. The bit still decides whether a .spc image's
// extra-RAM field holds $FFC0-$FFFF.
#define CONTROL_IPL_ROM		0x80

// FLG ($6C) resets to $E0: soft reset set, amplifier muted, echo writes disabled,
// noise stopped. A driver clears it once it has set up the DSP.
#define DSP_FLG				0x6C

// KON ($4C): the voices the driver last keyed on.
#define DSP_KON				0x4C

// The native sample rate of the DSP, which is the device's.
#define SNES_APU_RATE		32000


// ---- the machine ------------------------------------------------------------

// The seeded post-IPL power-on state: what the machine looks like once the boot
// ROM has cleared zero page, posted the ready bytes, and handed control over.
void snes_apu_init(SNES_APU* apu)
{
	unsigned int i;

	memset(apu, 0x00, sizeof(SNES_APU));
	snes_dsp_init(&apu->dsp);
	apu->aux_ports[0] = 0xFF;
	apu->aux_ports[1] = 0xFF;
	apu->cpu.sp = 0xEF;	// the stack lives in page $01; the first push lands at $01EF
	apu->test = 0x0A;
	apu->control = 0xB0;	// bit 7 maps the $FFC0 window when an image is mapped
	apu->output_ports[0] = 0xAA;	// the ready bytes a host polls for
	apu->output_ports[1] = 0xBB;
	apu->dsp.regs[DSP_FLG] = 0xE0;	// FLG's documented reset value
	for (i = 0; i < 3; i ++)
	{
		apu->timers[i].target = 0x00;
		apu->timers[i].stage2 = 0x00;
		apu->timers[i].stage3 = 0x0F;	// TnOUT reads $F on power on
	}
	// The DSP's sample slot rides the master counter: after a cycle that leaves the
	// counter at D, the DSP has run slot (D-1) and its cursor sits at D mod 32. A
	// seeded state establishes that lockstep so the machine drives the schedule
	// without passing the phase every cycle.
	apu->dsp.slot_cursor = (UINT8)(apu->divider & 31u);
}

// Maps an SPC700 Sound File image onto the machine (parseSpc).
UINT8 snes_apu_load_spc(SNES_APU* apu, const UINT8* data, UINT32 len)
{
	UINT8 mute_mask;
	unsigned int i;

	if (len < SNES_APU_SPC_MIN_SIZE)
		return SNES_APU_LOAD_TOO_SHORT;

	// The magic through the 'v'; the two version digits vary between revisions.
	if (memcmp(data, SPC_MAGIC, sizeof(SPC_MAGIC) - 1) != 0)
		return SNES_APU_LOAD_BAD_MAGIC;
	if (data[SPC_MARKER_OFFSET] != 26 || data[SPC_MARKER_OFFSET + 1] != 26)
		return SNES_APU_LOAD_BAD_MARKER;

	// The member-initializers of ApuState / DspState / TimerState ARE the power-on
	// shape for everything the file does not carry; the loader overlays only what it
	// does.
	mute_mask = apu->dsp.mute_mask;
	memset(apu, 0x00, sizeof(SNES_APU));
	snes_dsp_init(&apu->dsp);
	apu->dsp.mute_mask = mute_mask;
	apu->aux_ports[0] = 0xFF;
	apu->aux_ports[1] = 0xFF;

	// CPU registers. PC is little-endian; run state stays Running (the format has no
	// halt field).
	apu->cpu.pc = (UINT16)(data[SPC_PC_OFFSET] | (data[SPC_PC_OFFSET + 1] << 8));
	apu->cpu.a = data[SPC_A_OFFSET];
	apu->cpu.x = data[SPC_X_OFFSET];
	apu->cpu.y = data[SPC_Y_OFFSET];
	apu->cpu.psw = data[SPC_PSW_OFFSET];
	apu->cpu.sp = data[SPC_SP_OFFSET];

	// RAM verbatim, including the $F0-$FF bytes: those are the underlying RAM, which
	// is what the host sees and what the recovered registers below are read from.
	memcpy(apu->ram, data + SPC_RAM_OFFSET, sizeof(apu->ram));

	// DSP registers verbatim.
	memcpy(apu->dsp.regs, data + SPC_DSP_OFFSET, sizeof(apu->dsp.regs));

	// The extra-RAM field overwrites $FFC0-$FFFF only when the dump was taken with
	// CONTROL bit 7 set (the IPL window mapped read-only, so the RAM image could not
	// hold those bytes). When clear, the RAM image already holds them and the field
	// is ignored. The machine treats $FFC0-$FFFF as plain RAM either way.
	if (apu->ram[0xF1] & CONTROL_IPL_ROM)
		memcpy(apu->ram + 0xFFC0, data + SPC_EXTRA_RAM_OFFSET, SPC_EXTRA_RAM_SIZE);

	// Registers recovered from the RAM image, by direct field assignment (not a bus
	// write, so no CONTROL port-clear side effect fires at load).
	apu->control = apu->ram[0xF1];
	apu->dsp_addr = apu->ram[0xF2];
	for (i = 0; i < 3; i ++)
	{
		apu->timers[i].target = apu->ram[0xFA + i];
		apu->timers[i].stage3 = (UINT8)(apu->ram[0xFD + i] & 0x0F);
	}
	for (i = 0; i < 4; i ++)
	{
		// Both faces seed from the dumped bytes. The output latch is the last value the
		// CPU wrote (the overlay mirrors a port write into RAM), and the input latch
		// takes the same byte, so a driver polling a handshake value sees no spontaneous
		// change at sample zero.
		apu->input_ports[i] = apu->ram[0xF4 + i];
		apu->output_ports[i] = apu->ram[0xF4 + i];
	}
	// The AUXIO bytes read back what the CPU last wrote, which is what the dump's
	// RAM image holds at $F8/$F9 (a port write mirrors into the RAM beneath).
	apu->aux_ports[0] = apu->ram[0xF8];
	apu->aux_ports[1] = apu->ram[0xF9];

	// The DSP's sample slot rides the master counter (see snes_apu_init).
	apu->dsp.slot_cursor = (UINT8)(apu->divider & 31u);

	// The dumped KON byte keys its voices on again, as a CPU write would, so the notes
	// sounding at dump time are heard from the first sample (blargg's SPC_DSP::load does
	// the same with new_kon). The C++ leaves the key-on latch clear.
	snes_dsp_cpu_write_register(&apu->dsp, DSP_KON, apu->dsp.regs[DSP_KON]);
	return SNES_APU_LOAD_OK;
}

// One stage-1 tick for timer `index`: its stage-2 counter increments while the
// timer is enabled and, on reaching the target, advances stage 3 and zeroes.
static void tick_timer(SNES_APU* apu, unsigned int index)
{
	const UINT8 enable = (UINT8)(1u << index);
	SNES_APU_TIMER* t;
	if (! (apu->control & enable)) return;	// stage 2 only counts while enabled
	t = &apu->timers[index];
	t->stage2 = (UINT8)(t->stage2 + 1);	// 0-255 wraparound
	// Post-increment comparator: a target of 0 matches only after 256 counts.
	if (t->stage2 == t->target)
	{
		t->stage3 = (UINT8)((t->stage3 + 1) & 0x0Fu);	// 4-bit output
		t->stage2 = 0;
	}
}

// One machine cycle. The master counter advances, the timer ticks and the DSP
// sample boundary that land on the new count are taken, and then the CPU makes
// its one bus access: the order the chips share their multiplexed bus in, and
// the reason a read sees the tick that shares its cycle. Returns whether the DSP
// delivered a frame on this cycle.
INLINE int apu_machine_cycle(SNES_APU* apu)
{
	UINT16 phase;
	int delivered;

	// The counter wraps at 65536, a multiple of 128, 32 and 16, so every phase
	// below survives the wrap unbroken.
	++ apu->divider;
	phase = apu->divider;

	// The clocked events, on their documented slots of the DSP's 32-cycle sample
	// frame: T0 and T1 tick on the first slot of every fourth frame, T2 on the
	// first slot of every half-frame, and the sample lands on the frame boundary.
	if (phase % 128u == 1u)
	{
		tick_timer(apu, 0);
		tick_timer(apu, 1);
	}
	if (phase % 16u == 1u) tick_timer(apu, 2);

	// The DSP runs one of a sample's 32 slots this cycle; the wrap slot delivers the
	// finished stereo frame. The RAM is writable so the echo unit can write its
	// feedback straight into APU RAM (its own bus access, no overlay), the delay line
	// the echo depends on.
	delivered = snes_dsp_step_cycle(&apu->dsp, apu->ram);

	// The CPU's access closes the cycle. A halted core reaches nothing here, and
	// the cycle still passes for everything above.
	snes_spc700_step_cycle(&apu->cpu, apu);
	return delivered;
}

int snes_apu_machine_cycle(SNES_APU* apu)
{
	return apu_machine_cycle(apu);
}

void snes_apu_render(SNES_APU* apu, UINT32 frames, INT32* out_left, INT32* out_right)
{
	UINT32 n = 0;

	while (n < frames)
	{
		if (apu_machine_cycle(apu))
		{
			out_left[n] = apu->dsp.slot_frame_left;
			out_right[n] = apu->dsp.slot_frame_right;
			n ++;
		}
	}
}


// ---- the bus and the register overlay ---------------------------------------

// The byte the sound CPU's read of a register at $F0-$FF returns, with nothing
// moved: a timer's output is answered and not cleared, the DSP register under
// DSPADDR is answered and the address stands.
static UINT8 register_value(const SNES_APU* apu, UINT8 reg)
{
	switch(reg)
	{
	case 0xF2: return apu->dsp_addr;						// DSPADDR reads back the latched address
	case 0xF3: return apu->dsp.regs[apu->dsp_addr & 0x7Fu];	// DSPDATA masks the address with $7F
	case 0xF4: case 0xF5: case 0xF6: case 0xF7:				// input ports: what the host last wrote
		return apu->input_ports[reg - 0xF4u];
	case 0xF8: case 0xF9:									// AUXIO: the port's own byte, not the RAM beneath
		return apu->aux_ports[reg - 0xF8u];
	case 0xFD: case 0xFE: case 0xFF:						// TnOUT: the 4-bit stage-3 counter
		return (UINT8)(apu->timers[reg - 0xFDu].stage3 & 0x0Fu);
	// TEST, CONTROL and TnTARGET are write-only and read back 0.
	default:
		return 0;
	}
}

// The CPU's read of register `reg` ($F0-$FF): register_value's byte, then the
// clear a read of $FD-$FF makes.
static UINT8 read_register(SNES_APU* apu, UINT8 reg)
{
	const UINT8 v = register_value(apu, reg);
	if (reg >= 0xFDu) apu->timers[reg - 0xFDu].stage3 = 0;	// reading TnOUT clears the counter
	return v;
}

static void write_register(SNES_APU* apu, UINT8 reg, UINT8 value)
{
	// Every overlay write also lands in the underlying RAM byte (the register is an
	// overlay on RAM), then applies the register's own effect. TEST while the P
	// flag is set is the one write that has no effect at all.
	switch(reg)
	{
	case 0xF0:	// TEST
		if (apu->cpu.psw & SPC700_FLAG_P) return;
		apu->ram[reg] = value;
		apu->test = value;
		return;
	case 0xF1:	// CONTROL
	{
		const UINT8 previous = apu->control;
		unsigned int i;
		apu->ram[reg] = value;
		apu->control = value;
		// Bit 4 clears input ports 0/1, bit 5 clears input ports 2/3: a one-time
		// zeroing on every write with the bit set, never a 0->1 transition.
		if (value & 0x10u) { apu->input_ports[0] = 0; apu->input_ports[1] = 0; }
		if (value & 0x20u) { apu->input_ports[2] = 0; apu->input_ports[3] = 0; }
		// A timer enable going 0->1 resets that timer's stage-2 and stage-3
		// counters; the shared stage-1 divider is left running.
		for (i = 0; i < 3; i ++)
		{
			const UINT8 enable = (UINT8)(1u << i);
			if ((value & enable) && ! (previous & enable))
			{
				apu->timers[i].stage2 = 0;
				apu->timers[i].stage3 = 0;
			}
		}
		return;
	}
	case 0xF2:	// DSPADDR
		apu->ram[reg] = value;
		apu->dsp_addr = value;
		return;
	case 0xF3:	// DSPDATA: writes beyond address $7F are ignored
		apu->ram[reg] = value;
		if (apu->dsp_addr <= 0x7Fu) snes_dsp_cpu_write_register(&apu->dsp, apu->dsp_addr, value);
		return;
	case 0xF4: case 0xF5: case 0xF6: case 0xF7:	// output ports: SPC700 -> host
		apu->ram[reg] = value;
		apu->output_ports[reg - 0xF4u] = value;
		return;
	case 0xFA: case 0xFB: case 0xFC:	// TnTARGET
		apu->ram[reg] = value;
		apu->timers[reg - 0xFAu].target = value;
		return;
	case 0xF8: case 0xF9:	// AUXIO: the port keeps its own byte beside the RAM's
		apu->ram[reg] = value;
		apu->aux_ports[reg - 0xF8u] = value;
		return;
	// A write to a TnOUT register ($FD-$FF) lands in RAM but drives nothing, so
	// it needs only the RAM write.
	default:
		apu->ram[reg] = value;
		return;
	}
}

// The internal bus: $00F0-$00FF route to the register overlay, everything else
// is RAM.
UINT8 snes_apu_bus_read(SNES_APU* apu, UINT16 address)
{
	if (address >= 0x00F0u && address <= 0x00FFu)
		return read_register(apu, (UINT8)address);
	return apu->ram[address];
}

void snes_apu_bus_write(SNES_APU* apu, UINT16 address, UINT8 value)
{
	if (address >= 0x00F0u && address <= 0x00FFu)
	{
		write_register(apu, (UINT8)address, value);
		return;
	}
	apu->ram[address] = value;
}


// ---- the libvgm device ----------------------------------------------------

typedef struct _snes_apu_chip
{
	DEV_DATA _devData;

	SNES_APU apu;
	UINT8* image;	// the machine-complete part of the loaded .spc, NULL before a load
} SNES_APU_CHIP;

static void snes_apu_dev_reset(void* info);
static void snes_apu_dev_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_snes_apu(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_snes_apu(void* info);
static void snes_apu_write_block(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void snes_apu_set_mute_mask(void* info, UINT32 MuteMask);

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, snes_apu_write_block},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, snes_apu_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"SNES APU", "Snaggletooth", FCC_SNAG,

	device_start_snes_apu,
	device_stop_snes_apu,
	snes_apu_dev_reset,
	snes_apu_dev_update,

	NULL,	// SetOptionBits
	snes_apu_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "SNES APU";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 8;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_SNES_APU =
{
	DEVID_SNES_APU,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};

static void snes_apu_dev_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	SNES_APU_CHIP* chip = (SNES_APU_CHIP*)info;

	snes_apu_render(&chip->apu, samples, outputs[0], outputs[1]);
}

// Back to the loaded state (or the power-on one before a load); the mute mask stays.
static void snes_apu_dev_reset(void* info)
{
	SNES_APU_CHIP* chip = (SNES_APU_CHIP*)info;
	const UINT8 mute_mask = chip->apu.dsp.mute_mask;

	if (chip->image != NULL)
		snes_apu_load_spc(&chip->apu, chip->image, SNES_APU_SPC_MIN_SIZE);
	else
		snes_apu_init(&chip->apu);
	chip->apu.dsp.mute_mask = mute_mask;
}

static UINT8 device_start_snes_apu(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	SNES_APU_CHIP* chip;

	chip = (SNES_APU_CHIP*)calloc(1, sizeof(SNES_APU_CHIP));
	if (chip == NULL)
		return 0xFF;

	snes_apu_init(&chip->apu);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, SNES_APU_RATE, &devDef);
	return 0x00;
}

static void device_stop_snes_apu(void* info)
{
	SNES_APU_CHIP* chip = (SNES_APU_CHIP*)info;

	free(chip->image);
	free(chip);
}

// The whole .spc image, at offset 0. A rejected image leaves the device as it was.
static void snes_apu_write_block(void* info, UINT32 offset, UINT32 length, const UINT8* data)
{
	SNES_APU_CHIP* chip = (SNES_APU_CHIP*)info;

	if (offset != 0 || length < SNES_APU_SPC_MIN_SIZE)
		return;
	if (snes_apu_load_spc(&chip->apu, data, length) != SNES_APU_LOAD_OK)
		return;

	if (chip->image == NULL)
		chip->image = (UINT8*)malloc(SNES_APU_SPC_MIN_SIZE);
	if (chip->image != NULL)
		memcpy(chip->image, data, SNES_APU_SPC_MIN_SIZE);
}

static void snes_apu_set_mute_mask(void* info, UINT32 MuteMask)
{
	SNES_APU_CHIP* chip = (SNES_APU_CHIP*)info;

	chip->apu.dsp.mute_mask = (UINT8)(MuteMask & 0xFF);
}
