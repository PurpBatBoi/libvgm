// license:BSD-3-Clause
// copyright-holders:Bryan McPhail
/***************************************************************************

    Konami 005289 - SCC sound as used in Bubblesystem

    This file is pieced together by Bryan McPhail from a combination of
    Namco Sound, Amuse by Cab, Nemesis schematics and whoever first
    figured out SCC!

    The 005289 is a 2 channel sound generator. Each channel gets its
    waveform from a prom (4 bits wide).

    (From Nemesis schematics)

    Address lines A0-A4 of the prom run to the 005289, giving 32 bytes
    per waveform.  Address lines A5-A7 of the prom run to PA5-PA7 of
    the AY8910 control port A, giving 8 different waveforms. PA0-PA3
    of the AY8910 control volume.

    The second channel is the same as above except port B is used.

    The 005289 has 12 address inputs and 4 control inputs: LD1, LD2, TG1, TG2.
    It has no data bus, so data values written don't matter.
    When LD1 or LD2 is asserted, the 12 bit value on the address bus is
    latched. Each of the two channels has its own latch.
    When TG1 or TG2 is asserted, the frequency of the respective channel is
    set to the previously latched value.

    The 005289 itself is nothing but an address generator. Digital to analog
    conversion, volume control and mixing of the channels is all done
    externally via resistor networks and 4066 switches and is only implemented
    here for convenience.

***************************************************************************/

/**********************************************************************************************
    Konami 005289 SCC sound as used in Bubblesystem (libvgm implemetation)
	This is essentially a mix between MAME code and improvements by Mao and cam900.
	References:
	https://gitlab.com/cam900/vgsound_emu/-/tree/main/vgsound_emu/src/k005289?ref_type=heads
	https://git.redump.net/mame/tree/src/devices/sound/k005289.cpp
	https://git.redump.net/mame/tree/src/devices/sound/k005289.h
	
***********************************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stddef.h>	// for NULL

#include "../../stdtype.h"
#include "../../libs/blep_filter/blep_filter.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "k005289.h"

#define PROM_SIZE 0x200 // 512 bytes (256 per channel)
#define BLEP_BUFFER_SIZE 4000

typedef struct _k005289_state {
    DEV_DATA _devData;
    
    struct {
        UINT16 pitch;
        UINT16 freq;
        UINT8 volume;
        UINT8 waveform;
        INT16 counter;
        UINT8 addr;
    } voice[2];
    
    UINT8 prom[PROM_SIZE]; // Internal PROM storage
    UINT32 clock;
    UINT32 rate;
    UINT8 mute_mask;
    INT16 voice_level[2];
    blip_t* blep;
} k005289_state;

static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_k005289(void* chip);
static void device_reset_k005289(void* chip);
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask);
static void k005289_write(void* chip, UINT8 address, UINT16 data);
static void k005289_write_prom(void *info, UINT32 offset, UINT32 length, const UINT8* data);

// Add PROM write handler
static DEVDEF_RWFUNC devFunc[] = {
    {RWF_REGISTER | RWF_WRITE, DEVRW_A8D16, 0, k005289_write},
    {RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, k005289_set_mute_mask},
    {RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, k005289_write_prom},
    {0x00, 0x00, 0, NULL}
};

static DEV_DEF devDef = {
    "K005289", "MAME", FCC_MAME,
    device_start_k005289,
    device_stop_k005289,
    device_reset_k005289,
    k005289_update,

    NULL,   // SetOptionBits
    k005289_set_mute_mask,
    NULL,   // SetPanning
    NULL,   // SetSampleRateChangeCallback
    NULL,   // SetLoggingCallback
    NULL,   // LinkDevice

    devFunc
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "K005289";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return 2;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_K005289 =
{
	DEVID_K005289,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
		&devDef,
		NULL
	}
};

// Device start with internal PROM
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf) {
    k005289_state* info = calloc(1, sizeof(k005289_state));
    if (!info) return 0xFF;
    
    info->clock = cfg->clock;
    info->rate = cfg->smplRate ? cfg->smplRate : info->clock;
    if (info->clock == 0 || info->rate == 0)
    {
        free(info);
        return 0xFF;
    }
    info->blep = blip_new(BLEP_BUFFER_SIZE);
    if (info->blep == NULL)
    {
        free(info);
        return 0xFF;
    }
    blip_set_rates(info->blep, info->clock, info->rate);
    
    memset(info->prom, 0xFF, PROM_SIZE);
    
    info->mute_mask = 0x00;
    
    info->_devData.chipInf = info;
    INIT_DEVINF(retDevInf, &info->_devData, info->rate, &devDef);
    return 0x00;
}

// Device stop
static void device_stop_k005289(void* chip) {
    k005289_state* info = (k005289_state*)chip;
    blip_delete(info->blep);
    free(info);
}

// Device reset
static void device_reset_k005289(void* chip) {
    k005289_state* info = (k005289_state*)chip;
    memset(info->voice, 0, sizeof(info->voice));
    info->voice_level[0] = info->voice_level[1] = 0;
    blip_clear(info->blep);
}

static INT16 k005289_voice_level(const k005289_state* info, int ch)
{
    UINT16 prom_addr;
    INT8 sample;

    if (info->mute_mask & (1 << ch))
        return 0;
    prom_addr = (ch * 0x100) | (info->voice[ch].waveform << 5) | info->voice[ch].addr;
    sample = (INT8)(info->prom[prom_addr] - 8);
    return (INT16)(sample * info->voice[ch].volume * 16);
}

static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs) {
    k005289_state* info = (k005289_state*)param;
    short pcm[BLEP_BUFFER_SIZE];
    UINT32 done = 0;

    while (done < samples)
    {
        UINT32 count = (samples - done < BLEP_BUFFER_SIZE) ? samples - done : BLEP_BUFFER_SIZE;
        UINT32 clocks = (UINT32)blip_clocks_needed(info->blep, (int)count);
        UINT32 position = 0;
        INT32 delta = 0;
        UINT32 i;
        int ch;

        /* Register and mute changes between updates take effect at the frame boundary. */
        for (ch = 0; ch < 2; ch++)
        {
            INT16 level = k005289_voice_level(info, ch);
            delta += level - info->voice_level[ch];
            info->voice_level[ch] = level;
        }
        if (delta != 0)
            blip_add_delta(info->blep, 0, delta);

        while (position < clocks)
        {
            UINT32 step = clocks - position;

            for (ch = 0; ch < 2; ch++)
            {
                UINT32 until_step;
                if (info->mute_mask & (1 << ch))
                    continue;
                until_step = (UINT32)info->voice[ch].counter + 1;
                if (until_step < step)
                    step = until_step;
            }
            position += step;
            delta = 0;
            for (ch = 0; ch < 2; ch++)
            {
                if (info->mute_mask & (1 << ch))
                    continue;
                info->voice[ch].counter = (INT16)(info->voice[ch].counter - step);
                if (info->voice[ch].counter < 0)
                {
                    INT16 old_level = info->voice_level[ch];
                    info->voice[ch].addr = (info->voice[ch].addr + 1) & 0x1F;
                    info->voice[ch].counter = info->voice[ch].freq;
                    info->voice_level[ch] = k005289_voice_level(info, ch);
                    delta += info->voice_level[ch] - old_level;
                }
            }
            if (delta != 0)
                blip_add_delta(info->blep, position, delta);
        }

        blip_end_frame(info->blep, clocks);
        blip_read_samples(info->blep, pcm, (int)count, 0);
        for (i = 0; i < count; i++)
        {
            outputs[0][done + i] = pcm[i];
            outputs[1][done + i] = pcm[i];
        }
        done += count;
    }
}

// Write handlers
static void k005289_write(void* chip, UINT8 address, UINT16 data) {
    k005289_state* info = (k005289_state*)chip;
    int ch = address & 1; // Channel select

        switch (address) {
        // Control A (Channel 1)
        case 0x00:
        // Control B (Channel 2)
        case 0x01:
            info->voice[ch].volume = data & 0x0F;
            info->voice[ch].waveform = (data >> 5) & 0x07;
            break;

        // LD1/LD2 - Latch pitch
        case 0x02:
        case 0x03:
            info->voice[ch].pitch = 0xFFF - (data & 0x0FFF);
            break;

        // TG1/TG2 - Trigger frequency update
        case 0x04:
        case 0x05:
            info->voice[ch].freq = info->voice[ch].pitch;
            break;
    }
}

// Mute mask
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask) {
    k005289_state* info = (k005289_state*)chip;
    info->mute_mask = mute_mask & 0x03;
}

static void k005289_write_prom(void *info, UINT32 offset, UINT32 length, const UINT8* data)
{
    k005289_state *chip = (k005289_state *)info;
    
    if (offset >= PROM_SIZE)
        return;
    if (offset + length > PROM_SIZE)
        length = PROM_SIZE - offset;
    
    memcpy(chip->prom + offset, data, length);
}
