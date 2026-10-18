/***************************************************************************
	SPDX-License-Identifier: BSD-3-Clause
    Copyright (c) 2021, Aaron Giles
***************************************************************************/

#ifndef __YMFM_ADPCM_H__
#define __YMFM_ADPCM_H__

#include "../../stdtype.h"

#define ADPCM_A_CHANNELS	6
#define ADPCM_A_REGISTERS	0x30

#define ADPCM_B_REGISTERS	0x11

// ADPCM-B status bits
#define ADPCM_B_STATUS_EOS		0x01
#define ADPCM_B_STATUS_BRDY		0x02
#define ADPCM_B_STATUS_PLAYING	0x04

typedef struct _adpcm_a_channel
{
	UINT32 playing;			// currently playing?
	UINT32 curnibble;		// index of the current nibble
	UINT32 curbyte;			// current byte of data
	UINT32 curaddress;		// current address
	INT32 accumulator;		// accumulator
	INT32 step_index;		// index in the stepping table
} ADPCM_A_CHANNEL;

// the 6 ADPCM-A channels (YM2608 rhythm, YM2610)
typedef struct _adpcm_a_engine
{
	UINT8 regdata[ADPCM_A_REGISTERS];	// register data
	ADPCM_A_CHANNEL channel[ADPCM_A_CHANNELS];
	UINT32 shift;			// address bits shift-left
	const UINT8* mem;		// sample memory, owned by the chip; reads outside it return 0
	UINT32 mem_size;
} ADPCM_A_ENGINE;

// the ADPCM-B channel (Y8950, YM2608, YM2610)
typedef struct _adpcm_b_engine
{
	UINT8 regdata[ADPCM_B_REGISTERS];	// register data
	UINT32 status;			// ADPCM_B_STATUS_xxx
	UINT32 curnibble;		// index of the current nibble
	UINT32 curbyte;			// current byte of data
	UINT32 dummy_read;		// dummy read tracker
	UINT32 position;		// current fractional position
	UINT32 curaddress;		// current address
	INT32 accumulator;		// accumulator
	INT32 prev_accum;		// previous accumulator (for linear interp)
	INT32 adpcm_step;		// next forecast
	UINT32 shift;			// fixed address shift, 0 = from the registers
	UINT8* mem;				// sample memory, owned by the chip; accesses outside it read 0 / are dropped
	UINT32 mem_size;
} ADPCM_B_ENGINE;

// set up the engine with its address shift; the chip sets mem/mem_size
void adpcm_a_init(ADPCM_A_ENGINE* a, UINT32 shift);
void adpcm_a_reset(ADPCM_A_ENGINE* a);
// set the start/end registers of a channel
void adpcm_a_set_start_end(ADPCM_A_ENGINE* a, UINT8 chnum, UINT16 start, UINT16 end);
// clock the given channels; returns a bitmask of the samples that ended
UINT32 adpcm_a_clock(ADPCM_A_ENGINE* a, UINT32 chanmask);
// add the output of the given channels to output[0] (left) and output[1] (right)
void adpcm_a_output(const ADPCM_A_ENGINE* a, INT32* output, UINT32 chanmask);
void adpcm_a_write(ADPCM_A_ENGINE* a, UINT32 regnum, UINT8 data);

// set up the engine with its address shift (0 = from the registers); the chip sets mem/mem_size
void adpcm_b_init(ADPCM_B_ENGINE* b, UINT32 shift);
void adpcm_b_reset(ADPCM_B_ENGINE* b);
void adpcm_b_clock(ADPCM_B_ENGINE* b);
// add the output to output[0] (mono, outputs = 1) or with panning to output[0] / output[1] (outputs = 2)
void adpcm_b_output(const ADPCM_B_ENGINE* b, INT32* output, UINT32 outputs, UINT32 rshift);
UINT8 adpcm_b_read(ADPCM_B_ENGINE* b, UINT32 regnum);
void adpcm_b_write(ADPCM_B_ENGINE* b, UINT32 regnum, UINT8 value);

#endif	// __YMFM_ADPCM_H__
