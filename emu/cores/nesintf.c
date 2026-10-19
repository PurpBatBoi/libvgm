#include <stdlib.h>
#include <string.h>	// for memset
#include <stddef.h>	// for NULL

#include "../../stdtype.h"
#include "../../_stdbool.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../EmuHelper.h"

#include "nesintf.h"
#include "../panning.h"

#ifdef EC_NES_BEENES
#include "beenes_intf.h"
#include "purps_fds.h"
#endif


typedef struct nesapu_info NESAPU_INF;

static void nes_stream_update_beenes(void* chip, UINT32 samples, DEV_SMPL** outputs);

static UINT8 device_start_nes_beenes(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_nes_beenes(void* chip);
static void device_stop_nes_common(NESAPU_INF* info);
static void device_reset_nes_beenes(void* chip);

static void nes_w_beenes(void* chip, UINT8 offset, UINT8 data);
static UINT8 nes_r_beenes(void* chip, UINT8 offset);
static void nes_write_ram(void* chip, UINT32 offset, UINT32 length, const UINT8* data);

static void nes_set_chip_option_beenes(void* chip, UINT32 NesOptions);
static void nes_set_mute_mask_beenes(void* chip, UINT32 MuteMask);
static void nes_set_pan_beenes(void* chip, const INT16* PanVals);


#ifdef EC_NES_BEENES
static DEVDEF_RWFUNC devFunc_BeeNES[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, nes_w_beenes},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, nes_r_beenes},
	{RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, nes_write_ram},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, nes_set_mute_mask_beenes},
	{RWF_CHN_PAN | RWF_WRITE, DEVRW_ALL, 0, nes_set_pan_beenes},
	{0x00, 0x00, 0, NULL}
};
static DEV_DEF devDef_BeeNES =
{
	"NES APU", "BeeNES", FCC_BNES,
	
	device_start_nes_beenes,
	device_stop_nes_beenes,
	device_reset_nes_beenes,
	nes_stream_update_beenes,
	
	nes_set_chip_option_beenes,	// SetOptionBits
	nes_set_mute_mask_beenes,
	nes_set_pan_beenes,
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice
	
	devFunc_BeeNES,	// rwFuncs
};
#endif

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL && devCfg->flags)
		return "NES APU + FDS";
	return "NES APU";
}

#define DEV_CHN_COUNT_BASE	5
#define DEV_CHN_COUNT_FDS	(DEV_CHN_COUNT_BASE + 1)
static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	if (devCfg != NULL)
	{
		if (devCfg->flags)
			return DEV_CHN_COUNT_FDS;
		else
			return DEV_CHN_COUNT_BASE;
	}
	return DEV_CHN_COUNT_FDS;	// without configuration, return maximum channel number
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[DEV_CHN_COUNT_FDS] =
	{
		"Square 1", "Square 2", "Triangle", "Noise", "DPCM",
		"FDS",
	};
	return names;
}

static const DEVLINK_IDS* DeviceLinkIDs(const DEV_GEN_CFG* devCfg)
{
	return NULL;
}

const DEV_DECL sndDev_NES_APU =
{
	DEVID_NES_APU,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	DeviceLinkIDs,
	{	// cores
#ifdef EC_NES_BEENES
		&devDef_BeeNES,
#endif
		NULL
	}
};


struct nesapu_info
{
	DEV_DATA _devData;
	
	void* chip_apu;
	void* chip_dmc;
	void* chip_fds;
	UINT8* memory;
	UINT8 fds_disable;
};

// bit shift for transforming the "usual" panning values (factor 1<<16) into
// NSFPlay's scale, which uses a factor of 1<<7
#define PAN_NSFSHIFT	(PANNING_BITS - 7)
#define PAN_EMU2NSF(x)	(INT16)((x + (1 << PAN_NSFSHIFT >> 1)) >> PAN_NSFSHIFT)	// (x >> PAN_NSFSHIFT) with additional rounding




#ifdef EC_NES_BEENES
static void nes_stream_update_beenes(void* chip, UINT32 samples, DEV_SMPL** outputs)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_update(info->chip_apu, samples, outputs);
	if (info->chip_fds != NULL)
		purps_fds_update(info->chip_fds, samples, outputs);
	
	return;
}
#endif




#ifdef EC_NES_BEENES
static UINT8 device_start_nes_beenes(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
	NESAPU_INF* info;
	UINT32 rate;
	
	// The core synthesizes band-limited output at any rate, so it renders at the output rate
	// directly. Above clock/4 it would gain nothing, so libvgm resamples from there instead.
	rate = cfg->smplRate;
	if (rate == 0 || rate > cfg->clock / 4)
		rate = cfg->clock / 4;
	
	info = (NESAPU_INF*)calloc(1, sizeof(NESAPU_INF));
	if (info == NULL)
		return 0xFF;
	info->chip_apu = beenes_intf_create(cfg->clock, rate);
	if (info->chip_apu == NULL)
	{
		free(info);
		return 0xFF;
	}
	info->chip_dmc = NULL;
	
	// The BeeNES core has no FDS, so it comes with its own, written from the nesdev wiki.
	if (cfg->flags)	// enable FDS sound
		info->chip_fds = purps_fds_create(cfg->clock, rate);
	else
		info->chip_fds = NULL;
	
	info->memory = (UINT8*)malloc(0x8000);
	memset(info->memory, 0x00, 0x8000);
	beenes_intf_set_memory(info->chip_apu, info->memory);
	
	info->fds_disable = 0;
	
	info->_devData.chipInf = info;
	INIT_DEVINF(retDevInf, &info->_devData, rate, &devDef_BeeNES);
	return 0x00;
}
#endif



#ifdef EC_NES_BEENES
static void device_stop_nes_beenes(void* chip)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_destroy(info->chip_apu);
	if (info->chip_fds != NULL)
		purps_fds_destroy(info->chip_fds);
	info->chip_fds = NULL;	// already freed, not NSFPlay's
	device_stop_nes_common(info);
	
	return;
}
#endif

static void device_stop_nes_common(NESAPU_INF* info)
{
	if (info->memory != NULL)
		free(info->memory);
	
	free(info);
	return;
}



#ifdef EC_NES_BEENES
static void device_reset_nes_beenes(void* chip)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_reset(info->chip_apu);
	if (info->chip_fds != NULL)
		purps_fds_reset(info->chip_fds);
	
	return;
}
#endif




#ifdef EC_NES_BEENES
static void nes_w_beenes(void* chip, UINT8 offset, UINT8 data)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	if (offset < 0x20)	// NES APU
	{
		beenes_intf_write(info->chip_apu, offset, data);
	}
	else	// FDS
	{
		if (info->chip_fds != NULL && ! info->fds_disable)
			purps_fds_write(info->chip_fds, 0x4000 | offset, data);
	}
	return;
}

static UINT8 nes_r_beenes(void* chip, UINT8 offset)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	if (offset < 0x20)	// NES APU
	{
		return beenes_intf_read(info->chip_apu, offset);
	}
	else	// FDS
	{
		if (info->chip_fds != NULL && ! info->fds_disable)
			return purps_fds_read(info->chip_fds, 0x4000 | offset);
		return 0x00;
	}
}
#endif


static void nes_write_ram(void* chip, UINT32 offset, UINT32 length, const UINT8* data)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	UINT32 remBytes;
	
	if (offset >= 0x10000)
		return;
	
	// if (offset < 0x8000), then just copy the part for 0x8000+
	if (offset < 0x8000)
	{
		if (offset + length <= 0x8000)
			return;
		
		remBytes = 0x8000 - offset;
		offset = 0x8000;
		data += remBytes;
		length -= remBytes;
	}
	
	remBytes = 0x0000;
	if (offset + length > 0x10000)
	{
		remBytes = length;
		length = 0x10000 - offset;
		remBytes -= length;
	}
	memcpy(info->memory + (offset - 0x8000), data, length);
	// if we're crossing the boundary for 0x10000,
	// copy the bytes for 0x10000+ to 0x8000+
	if (remBytes)
	{
		if (remBytes > 0x8000)
			remBytes = 0x8000;
		memcpy(info->memory, data + length, remBytes);
	}
	
	return;
}




#ifdef EC_NES_BEENES
static void nes_set_chip_option_beenes(void* chip, UINT32 NesOptions)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_set_options(info->chip_apu, NesOptions);
	if (info->chip_fds != NULL)
		purps_fds_set_options(info->chip_fds, NesOptions);
	info->fds_disable = (NesOptions & OPT_NES_FDS_DISABLE) ? 1 : 0;
	
	return;
}
#endif




#ifdef EC_NES_BEENES
static void nes_set_mute_mask_beenes(void* chip, UINT32 MuteMask)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_set_mute_mask(info->chip_apu, MuteMask & 0x1F);
	if (info->chip_fds != NULL)
		purps_fds_set_mute(info->chip_fds, (MuteMask & 0x20) ? 1 : 0);
	
	return;
}

static void nes_set_pan_beenes(void* chip, const INT16* PanVals)
{
	NESAPU_INF* info = (NESAPU_INF*)chip;
	
	beenes_intf_set_panning(info->chip_apu, PanVals);
	if (info->chip_fds != NULL)
		purps_fds_set_panning(info->chip_fds, PanVals[5]);
	
	return;
}
#endif
