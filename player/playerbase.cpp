#include "playerbase.hpp"

#include <stdlib.h>
#include <string.h>	// for memset()

#include "../emu/SoundEmu.h"	// for SndEmu_GetDeviceFunc()
#include "../emu/EmuCores.h"

PlayerBase::PlayerBase() :
	_outSmplRate(0),
	_userDevList(NULL),
	_devStartOpts(0x00),
	_eventCbFunc(NULL),
	_eventCbParam(NULL),
	_fileReqCbFunc(NULL),
	_fileReqCbParam(NULL),
	_logCbFunc(NULL),
	_logCbParam(NULL)
{
}

PlayerBase::~PlayerBase()
{
}

bool PlayerBase::RequestFile(const char* fileName, std::vector<UINT8>& data)
{
	DATA_LOADER* dLoad;
	UINT32 size;
	const UINT8* ptr;
	
	data.clear();
	if (_fileReqCbFunc == NULL)
		return false;
	dLoad = _fileReqCbFunc(_fileReqCbParam, this, fileName);
	if (dLoad == NULL)
		return false;
	DataLoader_ReadAll(dLoad);
	size = DataLoader_GetSize(dLoad);
	ptr = DataLoader_GetData(dLoad);
	if (size > 0 && ptr != NULL)
		data.assign(ptr, ptr + size);
	DataLoader_Deinit(dLoad);
	return ! data.empty();
}

void PlayerBase::LoadYM2608RhythmROM(const DEV_INFO* devInf)
{
	DEVFUNC_WRITE_BLOCK romWrite = NULL;
	std::vector<UINT8> rom;
	
	// 0x524F = 'RO': the rhythm ROM; cores without it keep their own sounds
	if (SndEmu_GetDeviceFunc(devInf->devDef, RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0x524F, (void**)&romWrite))
		return;
	if (! RequestFile("ym2608_adpcm_rom.bin", rom))
		return;
	romWrite(devInf->dataPtr, 0x00, (UINT32)rom.size(), &rom[0]);
	return;
}

UINT32 PlayerBase::GetPlayerType(void) const
{
	return 0x00000000;
}

const char* PlayerBase::GetPlayerName(void) const
{
	return "";
}

/*static*/ UINT8 PlayerBase::PlayerCanLoadFile(DATA_LOADER *dataLoader)
{
	return 0xFF;
}

UINT8 PlayerBase::CanLoadFile(DATA_LOADER *dataLoader) const
{
	return this->PlayerCanLoadFile(dataLoader);
}

/*static*/ UINT8 PlayerBase::InitDeviceOptions(PLR_DEV_OPTS& devOpts)
{
	devOpts.emuCore[0] = 0x00;
	devOpts.emuCore[1] = 0x00;
	devOpts.srMode = DEVRI_SRMODE_NATIVE;
	devOpts.resmplMode = 0x00;
	devOpts.smplRate = 0;
	devOpts.coreOpts = 0x00;
	devOpts.muteOpts.disable = 0x00;
	devOpts.muteOpts.chnMute[0] = 0x00;
	devOpts.muteOpts.chnMute[1] = 0x00;
	memset(devOpts.panOpts.chnPan, 0x00, sizeof(devOpts.panOpts.chnPan));
	return 0x00;
}

UINT32 PlayerBase::GetSampleRate(void) const
{
	return _outSmplRate;
}

UINT8 PlayerBase::SetSampleRate(UINT32 sampleRate)
{
	_outSmplRate = sampleRate;
	return 0x00;
}

double PlayerBase::GetPlaybackSpeed(void) const
{
	return -1;	// not implemented
}

UINT8 PlayerBase::SetPlaybackSpeed(double speed)
{
	return 0xFF;	// not implemented
}

void PlayerBase::SetUserDevices(const DEV_DECL** userDevList, UINT8 devStartOpts)
{
	_userDevList = userDevList;
	_devStartOpts = devStartOpts;
	
	return;
}

void PlayerBase::SetEventCallback(PLAYER_EVENT_CB cbFunc, void* cbParam)
{
	_eventCbFunc = cbFunc;
	_eventCbParam = cbParam;
	
	return;
}

void PlayerBase::SetFileReqCallback(PLAYER_FILEREQ_CB cbFunc, void* cbParam)
{
	_fileReqCbFunc = cbFunc;
	_fileReqCbParam = cbParam;
	
	return;
}

void PlayerBase::SetLogCallback(PLAYER_LOG_CB cbFunc, void* cbParam)
{
	_logCbFunc = cbFunc;
	_logCbParam = cbParam;
	
	return;
}

double PlayerBase::Sample2Second(UINT32 samples) const
{
	if (samples == (UINT32)-1)
		return -1.0;
	return samples / (double)_outSmplRate;
}

UINT32 PlayerBase::GetTotalPlayTicks(UINT32 numLoops) const
{
	if (numLoops == 0 && GetLoopTicks() > 0)
		return (UINT32)-1;
	return GetTotalTicks() + GetLoopTicks() * (numLoops - 1);
}
