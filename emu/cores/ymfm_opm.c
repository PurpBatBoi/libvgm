/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles

    Yamaha OPM family

    C port of ymfm's OPM family (ymfm_opm.h, ymfm_opm.cpp).
    https://github.com/aaronsgiles/ymfm

    Chips: YM2151 (OPM).

    Port notes:
    - The registers (OPM_REGISTERS, ymfm's opm_registers) plug into the
      shared FM engine (ymfm_fm.c) through the opm_family table.
    - The noise generator of channel 7 uses the engine's FM_CH_NOISE
      channel kind, set when the channel is cached. The LFO AM value and
      the noise state are latched by the engine after each LFO clock.
    - The AM offset lfo_am << (sensitivity - 1), or 0 for sensitivity 0,
      is cached as the shift pair (lshift, rshift) = (sensitivity - 1, 0),
      or (0, 8): the AM value has 8 bits, so a right shift by 8 gives 0.
    - ymfm's m_noise_lfo is never used, so it is dropped.
    - One update sample is one FM sample (clock / 64).
    - The busy flag is not emulated. The CT1/CT2 output lines (register
      0x1B bits 6-7) are stored but drive nothing.
    - The YM2164 (OPP) variant is not included.
    - The key-on fix of the shared engine applies (see ymfm_fm.c).

***************************************************************************/

#include <stdlib.h>
#include <string.h>	// for memset

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"

#include "ymfm_fm.h"
#include "ymfm_fm_impl.h"
#include "ymfm_opm.h"


static void ym2151_update(void* info, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_ym2151(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_ym2151(void* info);
static void device_reset_ym2151(void* info);
static void ym2151_w(void* info, UINT8 offset, UINT8 data);
static UINT8 ym2151_r(void* info, UINT8 offset);
static void ym2151_set_mute_mask(void* info, UINT32 MuteMask);


//----------------------------------------------------------------------------
// device definitions
//----------------------------------------------------------------------------

static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, ym2151_w},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, ym2151_r},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, ym2151_set_mute_mask},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef =
{
	"YM2151", "ymfm", FCC_YMFM,

	device_start_ym2151,
	device_stop_ym2151,
	device_reset_ym2151,
	ym2151_update,

	NULL,	// SetOptionBits
	ym2151_set_mute_mask,
	NULL,	// SetPanning
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice

	devFunc,	// rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "YM2151";
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

const DEV_DECL sndDev_YM2151 =
{
	DEVID_YM2151,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};


//----------------------------------------------------------------------------
// chip state
//----------------------------------------------------------------------------

#define CHANNELS		8
#define REGISTERS		0x100
#define REG_MODE		0x14
#define DEFAULT_PRESCALE	2

#define STATUS_TIMERA	0x01
#define STATUS_TIMERB	0x02
#define STATUS_BUSY		0x80

// LFO waveforms are 256 entries long
#define LFO_WAVEFORM_LENGTH	256

//
// OPM register map:
//
//      System-wide registers:
//           01 xxxxxx-x Test register
//              ------x- LFO reset
//           08 -x------ Key on/off operator 4
//              --x----- Key on/off operator 3
//              ---x---- Key on/off operator 2
//              ----x--- Key on/off operator 1
//              -----xxx Channel select
//           0F x------- Noise enable
//              ---xxxxx Noise frequency
//           10 xxxxxxxx Timer A value (upper 8 bits)
//           11 ------xx Timer A value (lower 2 bits)
//           12 xxxxxxxx Timer B value
//           14 x------- CSM mode
//              --x----- Reset timer B
//              ---x---- Reset timer A
//              ----x--- Enable timer B
//              -----x-- Enable timer A
//              ------x- Load timer B
//              -------x Load timer A
//           18 xxxxxxxx LFO frequency
//           19 0xxxxxxx AM LFO depth
//              1xxxxxxx PM LFO depth
//           1B xx------ CT (2 output data lines)
//              ------xx LFO waveform
//
//     Per-channel registers (channel in address bits 0-2)
//        20-27 x------- Pan right
//              -x------ Pan left
//              --xxx--- Feedback level for operator 1 (0-7)
//              -----xxx Operator connection algorithm (0-7)
//        28-2F -xxxxxxx Key code
//        30-37 xxxxxx-- Key fraction
//        38-3F -xxx---- LFO PM sensitivity
//              ------xx LFO AM shift
//
//     Per-operator registers (channel in address bits 0-2, operator in bits 3-4)
//        40-5F -xxx---- Detune value (0-7)
//              ----xxxx Multiple value (0-15)
//        60-7F -xxxxxxx Total level (0-127)
//        80-9F xx------ Key scale rate (0-3)
//              ---xxxxx Attack rate (0-31)
//        A0-BF x------- LFO AM enable
//              ---xxxxx Decay rate (0-31)
//        C0-DF xx------ Detune 2 value (0-3)
//              ---xxxxx Sustain rate (0-31)
//        E0-FF xxxx---- Sustain level (0-15)
//              ----xxxx Release rate (0-15)
//
//     Internal (fake) registers:
//           1A -xxxxxxx PM depth
//

// the OPM registers (ymfm's opm_registers); the FM engine gets a pointer
// to this as its owner
typedef struct _opm_registers
{
	UINT32 lfo_counter;		// LFO counter
	UINT32 noise_lfsr;		// noise LFSR state
	UINT8 noise_counter;	// noise counter
	UINT8 noise_state;		// latched noise state
	UINT8 regdata[REGISTERS];	// register data
	INT16 lfo_waveform[4][LFO_WAVEFORM_LENGTH];	// LFO waveforms; AM in low 8, PM in upper 8
	UINT16 waveform[FM_WAVEFORM_LENGTH];	// sine waveform
} OPM_REGISTERS;

typedef struct _ym2151_state
{
	DEV_DATA _devData;

	FM_ENGINE fm;
	OPM_REGISTERS regs;

	UINT8 address;			// address register
} YM2151_STATE;


//----------------------------------------------------------------------------
// tables
//----------------------------------------------------------------------------

// The phase step is essentially the fnum in OPN-speak. To compute this table,
// we used the standard formula for computing the frequency of a note, and
// then converted that frequency to fnum using the formula documented in the
// YM2608 manual.
//
// However, the YM2608 manual describes everything in terms of a nominal 8MHz
// clock, which produces an FM clock of:
//
//    8000000 / 24(operators) / 6(prescale) = 55555Hz FM clock
//
// Whereas the descriptions for the YM2151 use a nominal 3.579545MHz clock:
//
//    3579545 / 32(operators) / 2(prescale) = 55930Hz FM clock
//
// To correct for this, the YM2608 formula was adjusted to use a clock of
// 8053920Hz, giving this equation for the fnum:
//
//    fnum = (double(144) * freq * (1 << 20)) / double(8053920) / 4;
//
// Unfortunately, the computed table differs in a few spots from the data
// verified from an actual chip. The table below comes from David Viens'
// analysis, used with his permission.
static const UINT32 s_phase_step[12*64] =
{
	41568,41600,41632,41664,41696,41728,41760,41792,41856,41888,41920,41952,42016,42048,42080,42112,
	42176,42208,42240,42272,42304,42336,42368,42400,42464,42496,42528,42560,42624,42656,42688,42720,
	42784,42816,42848,42880,42912,42944,42976,43008,43072,43104,43136,43168,43232,43264,43296,43328,
	43392,43424,43456,43488,43552,43584,43616,43648,43712,43744,43776,43808,43872,43904,43936,43968,
	44032,44064,44096,44128,44192,44224,44256,44288,44352,44384,44416,44448,44512,44544,44576,44608,
	44672,44704,44736,44768,44832,44864,44896,44928,44992,45024,45056,45088,45152,45184,45216,45248,
	45312,45344,45376,45408,45472,45504,45536,45568,45632,45664,45728,45760,45792,45824,45888,45920,
	45984,46016,46048,46080,46144,46176,46208,46240,46304,46336,46368,46400,46464,46496,46528,46560,
	46656,46688,46720,46752,46816,46848,46880,46912,46976,47008,47072,47104,47136,47168,47232,47264,
	47328,47360,47392,47424,47488,47520,47552,47584,47648,47680,47744,47776,47808,47840,47904,47936,
	48032,48064,48096,48128,48192,48224,48288,48320,48384,48416,48448,48480,48544,48576,48640,48672,
	48736,48768,48800,48832,48896,48928,48992,49024,49088,49120,49152,49184,49248,49280,49344,49376,
	49440,49472,49504,49536,49600,49632,49696,49728,49792,49824,49856,49888,49952,49984,50048,50080,
	50144,50176,50208,50240,50304,50336,50400,50432,50496,50528,50560,50592,50656,50688,50752,50784,
	50880,50912,50944,50976,51040,51072,51136,51168,51232,51264,51328,51360,51424,51456,51488,51520,
	51616,51648,51680,51712,51776,51808,51872,51904,51968,52000,52064,52096,52160,52192,52224,52256,
	52384,52416,52448,52480,52544,52576,52640,52672,52736,52768,52832,52864,52928,52960,52992,53024,
	53120,53152,53216,53248,53312,53344,53408,53440,53504,53536,53600,53632,53696,53728,53792,53824,
	53920,53952,54016,54048,54112,54144,54208,54240,54304,54336,54400,54432,54496,54528,54592,54624,
	54688,54720,54784,54816,54880,54912,54976,55008,55072,55104,55168,55200,55264,55296,55360,55392,
	55488,55520,55584,55616,55680,55712,55776,55808,55872,55936,55968,56032,56064,56128,56160,56224,
	56288,56320,56384,56416,56480,56512,56576,56608,56672,56736,56768,56832,56864,56928,56960,57024,
	57120,57152,57216,57248,57312,57376,57408,57472,57536,57568,57632,57664,57728,57792,57824,57888,
	57952,57984,58048,58080,58144,58208,58240,58304,58368,58400,58464,58496,58560,58624,58656,58720,
	58784,58816,58880,58912,58976,59040,59072,59136,59200,59232,59296,59328,59392,59456,59488,59552,
	59648,59680,59744,59776,59840,59904,59936,60000,60064,60128,60160,60224,60288,60320,60384,60416,
	60512,60544,60608,60640,60704,60768,60800,60864,60928,60992,61024,61088,61152,61184,61248,61280,
	61376,61408,61472,61536,61600,61632,61696,61760,61824,61856,61920,61984,62048,62080,62144,62208,
	62272,62304,62368,62432,62496,62528,62592,62656,62720,62752,62816,62880,62944,62976,63040,63104,
	63200,63232,63296,63360,63424,63456,63520,63584,63648,63680,63744,63808,63872,63904,63968,64032,
	64096,64128,64192,64256,64320,64352,64416,64480,64544,64608,64672,64704,64768,64832,64896,64928,
	65024,65056,65120,65184,65248,65312,65376,65408,65504,65536,65600,65664,65728,65792,65856,65888,
	65984,66016,66080,66144,66208,66272,66336,66368,66464,66496,66560,66624,66688,66752,66816,66848,
	66944,66976,67040,67104,67168,67232,67296,67328,67424,67456,67520,67584,67648,67712,67776,67808,
	67904,67936,68000,68064,68128,68192,68256,68288,68384,68448,68512,68544,68640,68672,68736,68800,
	68896,68928,68992,69056,69120,69184,69248,69280,69376,69440,69504,69536,69632,69664,69728,69792,
	69920,69952,70016,70080,70144,70208,70272,70304,70400,70464,70528,70560,70656,70688,70752,70816,
	70912,70976,71040,71104,71136,71232,71264,71360,71424,71488,71552,71616,71648,71744,71776,71872,
	71968,72032,72096,72160,72192,72288,72320,72416,72480,72544,72608,72672,72704,72800,72832,72928,
	72992,73056,73120,73184,73216,73312,73344,73440,73504,73568,73632,73696,73728,73824,73856,73952,
	74080,74144,74208,74272,74304,74400,74432,74528,74592,74656,74720,74784,74816,74912,74944,75040,
	75136,75200,75264,75328,75360,75456,75488,75584,75648,75712,75776,75840,75872,75968,76000,76096,
	76224,76288,76352,76416,76448,76544,76576,76672,76736,76800,76864,76928,77024,77120,77152,77248,
	77344,77408,77472,77536,77568,77664,77696,77792,77856,77920,77984,78048,78144,78240,78272,78368,
	78464,78528,78592,78656,78688,78784,78816,78912,78976,79040,79104,79168,79264,79360,79392,79488,
	79616,79680,79744,79808,79840,79936,79968,80064,80128,80192,80256,80320,80416,80512,80544,80640,
	80768,80832,80896,80960,80992,81088,81120,81216,81280,81344,81408,81472,81568,81664,81696,81792,
	81952,82016,82080,82144,82176,82272,82304,82400,82464,82528,82592,82656,82752,82848,82880,82976,
};

// convert an OPM concatenated block (3 bits), keycode (4 bits) and key
// fraction (6 bits) to a 0.10 phase step, after applying the given delta
static UINT32 opm_key_code_to_phase_step(UINT32 block_freq, INT32 delta)
{
	UINT32 block;
	UINT32 adjusted_code;
	INT32 eff_freq;

	// extract the block (octave) first
	block = bitfield(block_freq, 10, 3);

	// the keycode (bits 6-9) is "gappy", mapping 12 values over 16 in each
	// octave; to correct for this, we multiply the 4-bit value by 3/4 (or
	// rather subtract 1/4); note that a (invalid) value of 15 will bleed into
	// the next octave -- this is confirmed
	adjusted_code = bitfield(block_freq, 6, 4) - bitfield(block_freq, 8, 2);

	// now re-insert the 6-bit fraction
	eff_freq = (INT32)((adjusted_code << 6) | bitfield(block_freq, 0, 6));

	// now that the gaps are removed, add the delta
	eff_freq += delta;

	// handle over/underflow by adjusting the block:
	if ((UINT32)eff_freq >= 768)
	{
		// minimum delta is -512 (PM), so we can only underflow by 1 octave
		if (eff_freq < 0)
		{
			eff_freq += 768;
			if (block-- == 0)
				return s_phase_step[0] >> 7;
		}

		// maximum delta is +512+608 (PM+detune), so we can overflow by up to 2 octaves
		else
		{
			eff_freq -= 768;
			if (eff_freq >= 768)
			{
				block ++;
				eff_freq -= 768;
			}
			if (block++ >= 7)
				return s_phase_step[767];
		}
	}

	// look up the phase shift for the key code, then shift by octave
	return s_phase_step[eff_freq] >> (block ^ 7);
}


//----------------------------------------------------------------------------
// registers
//----------------------------------------------------------------------------

#define REG(offset)		(regs->regdata[offset])

// system-wide registers
#define lfo_reset()			bitfield(REG(0x01), 1, 1)
#define noise_frequency()	(bitfield(REG(0x0f), 0, 5) ^ 0x1f)
#define noise_enable()		bitfield(REG(0x0f), 7, 1)
#define timer_a_value()		((bitfield(REG(0x10), 0, 8) << 2) | bitfield(REG(0x11), 0, 2))
#define timer_b_value()		bitfield(REG(0x12), 0, 8)
#define csm()				bitfield(REG(0x14), 7, 1)
#define reset_timer_b()		bitfield(REG(0x14), 5, 1)
#define reset_timer_a()		bitfield(REG(0x14), 4, 1)
#define enable_timer_b()	bitfield(REG(0x14), 3, 1)
#define enable_timer_a()	bitfield(REG(0x14), 2, 1)
#define load_timer_b()		bitfield(REG(0x14), 1, 1)
#define load_timer_a()		bitfield(REG(0x14), 0, 1)
#define lfo_rate()			bitfield(REG(0x18), 0, 8)
#define lfo_am_depth()		bitfield(REG(0x19), 0, 7)
#define lfo_pm_depth()		bitfield(REG(0x1a), 0, 7)
#define lfo_waveform()		bitfield(REG(0x1b), 0, 2)

// per-channel registers
#define ch_output_0(choffs)		bitfield(REG(0x20 + (choffs)), 6, 1)
#define ch_output_1(choffs)		bitfield(REG(0x20 + (choffs)), 7, 1)
#define ch_feedback(choffs)		bitfield(REG(0x20 + (choffs)), 3, 3)
#define ch_algorithm(choffs)	bitfield(REG(0x20 + (choffs)), 0, 3)
#define ch_block_freq(choffs)	((bitfield(REG(0x28 + (choffs)), 0, 7) << 6) | bitfield(REG(0x30 + (choffs)), 2, 6))
#define ch_lfo_pm_sens(choffs)	bitfield(REG(0x38 + (choffs)), 4, 3)
#define ch_lfo_am_sens(choffs)	bitfield(REG(0x38 + (choffs)), 0, 2)

// per-operator registers
#define op_detune(opoffs)		bitfield(REG(0x40 + (opoffs)), 4, 3)
#define op_multiple(opoffs)		bitfield(REG(0x40 + (opoffs)), 0, 4)
#define op_total_level(opoffs)	bitfield(REG(0x60 + (opoffs)), 0, 7)
#define op_ksr(opoffs)			bitfield(REG(0x80 + (opoffs)), 6, 2)
#define op_attack_rate(opoffs)	bitfield(REG(0x80 + (opoffs)), 0, 5)
#define op_lfo_am_enable(opoffs)	bitfield(REG(0xa0 + (opoffs)), 7, 1)
#define op_decay_rate(opoffs)	bitfield(REG(0xa0 + (opoffs)), 0, 5)
#define op_detune2(opoffs)		bitfield(REG(0xc0 + (opoffs)), 6, 2)
#define op_sustain_rate(opoffs)	bitfield(REG(0xc0 + (opoffs)), 0, 5)
#define op_sustain_level(opoffs)	bitfield(REG(0xe0 + (opoffs)), 4, 4)
#define op_release_rate(opoffs)	bitfield(REG(0xe0 + (opoffs)), 0, 4)

// on OPM the channel offset is the channel number
static UINT32 opm_channel_offset(UINT32 chnum)
{
	return chnum;
}

// on OPM the operator offset is the operator number
static UINT32 opm_operator_offset(UINT32 opnum)
{
	return opnum;
}

// return an array of operator indices for each channel; for OPM this is fixed
static void opm_operator_map(void* owner, UINT32* map)
{
	UINT32 chnum;

	// Note that the channel index order is 0,2,1,3, so we bitswap the index.
	//
	// This is because the order in the map is:
	//    carrier 1, carrier 2, modulator 1, modulator 2
	//
	// But when wiring up the connections, the more natural order is:
	//    carrier 1, modulator 1, carrier 2, modulator 2
	for (chnum = 0; chnum < CHANNELS; chnum ++)
		map[chnum] = fm_operator_list(chnum + 0, chnum + 16, chnum + 8, chnum + 24);
	return;
}

static void opm_reset(void* owner)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;

	memset(regs->regdata, 0x00, REGISTERS);

	// enable output on both channels by default
	REG(0x20) = REG(0x21) = REG(0x22) = REG(0x23) = 0xc0;
	REG(0x24) = REG(0x25) = REG(0x26) = REG(0x27) = 0xc0;
	return;
}

// handle writes to the register array; returns 1 for key on/off writes
static UINT8 opm_write(void* owner, UINT16 index, UINT8 data, UINT32* channel, UINT32* opmask)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;

	// LFO AM/PM depth are written to the same register (0x19);
	// redirect the PM depth to an unused neighbor (0x1a)
	if (index == 0x19)
		REG(index + bitfield(data, 7, 1)) = data;
	else if (index != 0x1a)
		REG(index) = data;

	// handle writes to the key on index
	if (index == 0x08)
	{
		*channel = bitfield(data, 0, 3);
		*opmask = bitfield(data, 3, 4);
		return 1;
	}
	return 0;
}

static void opm_read_mode(void* owner, FM_MODE* mode)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;

	mode->timer_a_value = timer_a_value();
	mode->timer_b_value = timer_b_value();
	mode->status_mask = 0;
	mode->irq_reset = 0;
	mode->csm = (UINT8)csm();
	mode->reset_timer_a = (UINT8)reset_timer_a();
	mode->reset_timer_b = (UINT8)reset_timer_b();
	mode->enable_timer_a = (UINT8)enable_timer_a();
	mode->enable_timer_b = (UINT8)enable_timer_b();
	mode->load_timer_a = (UINT8)load_timer_a();
	mode->load_timer_b = (UINT8)load_timer_b();
	return;
}

// clock the noise and LFO, handling clock division, depth, and waveform
// computations; returns the LFO PM value
static INT32 opm_clock_noise_and_lfo(void* owner, UINT8* lfo_am, UINT8* noise_state)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;
	UINT32 freq;
	UINT32 rate;
	UINT32 lfo;
	UINT32 lfo_noise;
	INT32 ampm;
	int rep;

	// base noise frequency is measured at 2x 1/2 FM frequency; this
	// means each tick counts as two steps against the noise counter
	freq = noise_frequency();
	for (rep = 0; rep < 2; rep ++)
	{
		// evidence seems to suggest the LFSR is clocked continually and just
		// sampled at the noise frequency for output purposes; note that the
		// low 8 bits are the most recent 8 bits of history while bits 8-24
		// contain the 17 bit LFSR state
		regs->noise_lfsr <<= 1;
		regs->noise_lfsr |= bitfield(regs->noise_lfsr, 17, 1) ^ bitfield(regs->noise_lfsr, 14, 1) ^ 1;

		// compare against the frequency and latch when we exceed it
		if (regs->noise_counter++ >= freq)
		{
			regs->noise_counter = 0;
			regs->noise_state = (UINT8)bitfield(regs->noise_lfsr, 17, 1);
		}
	}
	*noise_state = regs->noise_state;

	// treat the rate as a 4.4 floating-point step value with implied
	// leading 1; this matches exactly the frequencies in the application
	// manual, though it might not be implemented exactly this way on chip
	rate = lfo_rate();
	regs->lfo_counter += (0x10 | bitfield(rate, 0, 4)) << bitfield(rate, 4, 4);

	// bit 1 of the test register is officially undocumented but has been
	// discovered to hold the LFO in reset while active
	if (lfo_reset())
		regs->lfo_counter = 0;

	// now pull out the non-fractional LFO value
	lfo = bitfield(regs->lfo_counter, 22, 8);

	// fill in the noise entry 1 ahead of our current position; this
	// ensures the current value remains stable for a full LFO clock
	// and effectively latches the running value when the LFO advances
	lfo_noise = bitfield(regs->noise_lfsr, 17, 8);
	regs->lfo_waveform[3][(lfo + 1) & 0xff] = (INT16)(lfo_noise | (lfo_noise << 8));

	// fetch the AM/PM values based on the waveform; AM is unsigned and
	// encoded in the low 8 bits, while PM signed and encoded in the upper
	// 8 bits
	ampm = regs->lfo_waveform[lfo_waveform()][lfo];

	// apply depth to the AM value and store for later
	*lfo_am = (UINT8)(((ampm & 0xff) * lfo_am_depth()) >> 7);

	// apply depth to the PM value and return it
	return ((ampm >> 8) * (INT32)lfo_pm_depth()) >> 7;
}

static void opm_cache_channel_data(void* owner, UINT32 choffs, FM_CHANNEL_CACHE* cache)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;
	UINT32 am_sensitivity;

	cache->feedback = (UINT8)ch_feedback(choffs);
	cache->algorithm = (UINT8)ch_algorithm(choffs);
	cache->outputs = (UINT8)(ch_output_0(choffs) | (ch_output_1(choffs) << 1));

	// OPM maps AM quite differently from OPN
	// shift value for AM sensitivity is [*, 0, 1, 2],
	// mapping to values of [0, 23.9, 47.8, and 95.6dB]

	// QUESTION: see OPN note below for the dB range mapping; it applies
	// here as well

	// raw LFO AM value on OPM is 0-FF, which is already a factor of 2
	// larger than the OPN below, putting our staring point at 2x theirs;
	// this works out since our minimum is 2x their maximum
	am_sensitivity = ch_lfo_am_sens(choffs);
	cache->am_lshift = (UINT8)((am_sensitivity == 0) ? 0 : am_sensitivity - 1);
	cache->am_rshift = (UINT8)((am_sensitivity == 0) ? 8 : 0);

	// the 4th operator of channel 7 is the noise generator when enabled
	cache->kind = (noise_enable() && choffs == 7) ? FM_CH_NOISE : FM_CH_NORMAL;
	return;
}

// compute the phase step, given a PM value
static UINT32 opm_compute_phase_step(void* owner, UINT32 choffs, UINT32 opoffs, const FM_OPDATA_CACHE* cache, INT32 lfo_raw_pm)
{
	// start with coarse detune delta; table uses cents value from
	// manual, converted into 1/64ths
	static const INT16 s_detune2_delta[4] = { 0, (600*64+50)/100, (781*64+50)/100, (950*64+50)/100 };
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;
	INT32 delta;
	UINT32 pm_sensitivity;
	UINT32 phase_step;

	// OPM logic is rather unique here, due to extra detune
	// and the use of key codes (not to be confused with keycode)
	delta = s_detune2_delta[op_detune2(opoffs)];

	// add in the PM delta
	pm_sensitivity = ch_lfo_pm_sens(choffs);
	if (pm_sensitivity != 0)
	{
		// raw PM value is -127..128 which is +/- 200 cents
		// manual gives these magnitudes in cents:
		//    0, +/-5, +/-10, +/-20, +/-50, +/-100, +/-400, +/-700
		// this roughly corresponds to shifting the 200-cent value:
		//    0  >> 5,  >> 4,  >> 3,  >> 2,  >> 1,   << 1,   << 2
		if (pm_sensitivity < 6)
			delta += lfo_raw_pm >> (6 - pm_sensitivity);
		else
			delta += (INT32)((UINT32)lfo_raw_pm << (pm_sensitivity - 5));
	}

	// apply delta and convert to a frequency number
	phase_step = opm_key_code_to_phase_step(cache->block_freq, delta);

	// apply detune based on the keycode
	phase_step += cache->detune;

	// apply frequency multiplier (which is cached as an x.1 value)
	return (phase_step * cache->multiple) >> 1;
}

// fill the operator cache with prefetched data
static void opm_cache_operator_data(void* owner, UINT32 choffs, UINT32 opoffs, FM_OPDATA_CACHE* cache)
{
	OPM_REGISTERS* regs = (OPM_REGISTERS*)owner;
	UINT32 block_freq;
	UINT32 keycode;
	UINT32 ksrval;

	// set up the easy stuff
	cache->waveform = regs->waveform;
	cache->lfo_am_enable = (UINT8)op_lfo_am_enable(opoffs);
	cache->ssg_eg = 0;

	// get frequency from the channel
	block_freq = cache->block_freq = ch_block_freq(choffs);

	// compute the keycode: block_freq is:
	//
	//     BBBCCCCFFFFFF
	//     ^^^^^
	//
	// the 5-bit keycode is just the top 5 bits (block + top 2 bits
	// of the key code)
	keycode = bitfield(block_freq, 8, 5);

	// detune adjustment
	cache->detune = fm_detune_adjustment(op_detune(opoffs), keycode);

	// multiple value, as an x.1 value (0 means 0.5)
	cache->multiple = op_multiple(opoffs) * 2;
	if (cache->multiple == 0)
		cache->multiple = 1;

	// phase step, or PHASE_STEP_DYNAMIC if PM is active; this depends on
	// block_freq, detune, and multiple, so compute it after we've done those
	if (lfo_pm_depth() == 0 || ch_lfo_pm_sens(choffs) == 0)
		cache->phase_step = opm_compute_phase_step(owner, choffs, opoffs, cache, 0);
	else
		cache->phase_step = PHASE_STEP_DYNAMIC;

	// total level, scaled by 8
	cache->total_level = op_total_level(opoffs) << 3;

	// 4-bit sustain level, but 15 means 31 so effectively 5 bits
	cache->eg_sustain = op_sustain_level(opoffs);
	cache->eg_sustain |= (cache->eg_sustain + 1) & 0x10;
	cache->eg_sustain <<= 5;

	// determine KSR adjustment for envelope rates
	ksrval = keycode >> (op_ksr(opoffs) ^ 3);
	cache->eg_rate[EG_ATTACK] = (UINT8)fm_effective_rate(op_attack_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_DECAY] = (UINT8)fm_effective_rate(op_decay_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_SUSTAIN] = (UINT8)fm_effective_rate(op_sustain_rate(opoffs) * 2, ksrval);
	cache->eg_rate[EG_RELEASE] = (UINT8)fm_effective_rate(op_release_rate(opoffs) * 4 + 2, ksrval);
	return;
}

#undef REG

// YM2151: 8 channels, 2 outputs
static const FM_FAMILY opm_family =
{
	CHANNELS,			// channels
	2,					// outputs
	CHANNELS * 4,		// operators
	DEFAULT_PRESCALE,	// default_prescale
	3,					// eg_clock_divider
	REG_MODE,			// reg_mode
	(1 << CHANNELS) - 1,	// csm_trigger_mask
	STATUS_TIMERA,		// status_timera
	STATUS_TIMERB,		// status_timerb
	STATUS_BUSY,		// status_busy
	0,					// status_irq
	0,					// dynamic_ops
	0,					// modulator_delay
	0,					// eg_has_ssg

	opm_channel_offset,
	opm_operator_offset,
	opm_operator_map,
	opm_reset,
	opm_write,
	opm_read_mode,
	opm_clock_noise_and_lfo,
	opm_cache_channel_data,
	opm_cache_operator_data,
	opm_compute_phase_step,
};

// set up the registers and the FM engine of one chip
static void opm_engine_init(FM_ENGINE* fm, OPM_REGISTERS* regs)
{
	UINT32 index;
	UINT8 am;
	UINT8 pm;

	regs->lfo_counter = 0;
	regs->noise_lfsr = 1;
	regs->noise_counter = 0;
	regs->noise_state = 0;

	// create the waveforms
	for (index = 0; index < FM_WAVEFORM_LENGTH; index ++)
		regs->waveform[index] = (UINT16)(fm_abs_sin_attenuation(index) | (bitfield(index, 9, 1) << 15));

	// create the LFO waveforms; AM in the low 8 bits, PM in the upper 8
	// waveforms are adjusted to match the pictures in the application manual
	for (index = 0; index < LFO_WAVEFORM_LENGTH; index ++)
	{
		// waveform 0 is a sawtooth
		am = (UINT8)(index ^ 0xff);
		pm = (UINT8)index;
		regs->lfo_waveform[0][index] = (INT16)(am | (pm << 8));

		// waveform 1 is a square wave
		am = bitfield(index, 7, 1) ? 0 : 0xff;
		pm = am ^ 0x80;
		regs->lfo_waveform[1][index] = (INT16)(am | (pm << 8));

		// waveform 2 is a triangle wave
		am = (UINT8)(bitfield(index, 7, 1) ? (index << 1) : ((index ^ 0xff) << 1));
		pm = (UINT8)(bitfield(index, 6, 1) ? am : ~am);
		regs->lfo_waveform[2][index] = (INT16)(am | (pm << 8));

		// waveform 3 is noise; it is filled in dynamically
		regs->lfo_waveform[3][index] = 0;
	}

	fm_engine_init(fm, &opm_family, regs);
	return;
}


//----------------------------------------------------------------------------
// YM2151
//----------------------------------------------------------------------------

// generate one sample of sound
static void ym2151_generate(YM2151_STATE* chip, DEV_SMPL* outL, DEV_SMPL* outR)
{
	INT32 output[2];

	// clock the system
	fm_engine_clock(&chip->fm, &opm_family, chip->fm.all_channels);

	// update the FM content; OPM is full 14-bit with no intermediate clipping
	output[0] = output[1] = 0;
	fm_engine_output(&chip->fm, &opm_family, output, 0, 32767, chip->fm.all_channels);

	// YM2151 uses an external DAC (YM3012) with mantissa/exponent format
	// convert to 10.3 floating point value and back to simulate truncation
	*outL = fm_roundtrip_fp(output[0]);
	*outR = fm_roundtrip_fp(output[1]);
	return;
}


//----------------------------------------------------------------------------
// libvgm device interface
//----------------------------------------------------------------------------

static UINT8 device_start_ym2151(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	YM2151_STATE* chip;

	chip = (YM2151_STATE*)calloc(1, sizeof(YM2151_STATE));
	if (chip == NULL)
		return 0xFF;

	opm_engine_init(&chip->fm, &chip->regs);
	fm_engine_reset(&chip->fm);

	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, cfg->clock / (DEFAULT_PRESCALE * CHANNELS * 4), &devDef);
	return 0x00;
}

static void device_stop_ym2151(void* info)
{
	free(info);
	return;
}

static void device_reset_ym2151(void* info)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	// ymfm's reset leaves the address register alone; a device reset clears it too
	chip->address = 0;
	fm_engine_reset(&chip->fm);
	return;
}

static void ym2151_update(void* info, UINT32 samples, DEV_SMPL** outputs)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;
	UINT32 curSmpl;

	for (curSmpl = 0; curSmpl < samples; curSmpl ++)
		ym2151_generate(chip, &outputs[0][curSmpl], &outputs[1][curSmpl]);
	return;
}

static void ym2151_w(void* info, UINT8 offset, UINT8 data)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	if (! (offset & 1))
		chip->address = data;	// address port
	else
		fm_engine_write(&chip->fm, chip->address, data);	// data port
	return;
}

static UINT8 ym2151_r(void* info, UINT8 offset)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	// status port, YM2203 compatible; the data port is unused
	if (! (offset & 1))
		return 0xFF;
	return fm_engine_status(&chip->fm);
}

static void ym2151_set_mute_mask(void* info, UINT32 MuteMask)
{
	YM2151_STATE* chip = (YM2151_STATE*)info;

	chip->fm.mute_mask = MuteMask;
	return;
}
