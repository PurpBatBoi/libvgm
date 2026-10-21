#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <algorithm>

#include "../common_def.h"
#include "spcplayer.hpp"
#include "../emu/EmuStructs.h"
#include "../emu/SoundEmu.h"
#include "../emu/Resampler.h"
#include "../emu/SoundDevs.h"
#include "../emu/EmuCores.h"
#include "../utils/StrUtils.h"
#include "helper.h"


// file layout
#define SPC_MAGIC			"SNES-SPC700 Sound File Data v"	// checked up to the 'v'
#define SPC_MAGIC_LEN		29
#define SPC_MARKER_OFS		0x21	// 26, 26
#define SPC_ID666_FLAG_OFS	0x23	// 26 = the header has an ID666 tag
#define SPC_MIN_SIZE		0x10200	// machine-complete image (header, RAM, DSP registers, extra RAM)
#define SPC_XID6_OFS		0x10200	// optional extended ID666 chunk

#define SPC_TICK_RATE		32000	// the S-DSP's output rate; ticks are device frames
#define XID6_TICK_DIV		2		// xid6 lengths are 1/64000 s

INLINE UINT32 ReadLE32(const UINT8* data)
{
	return	((UINT32)data[0x03] << 24) | ((UINT32)data[0x02] << 16) |
			((UINT32)data[0x01] <<  8) | ((UINT32)data[0x00] <<  0);
}

INLINE void SaveDeviceConfig(std::vector<UINT8>& dst, const void* srcData, size_t srcLen)
{
	const UINT8* srcPtr = (const UINT8*)srcData;
	dst.assign(srcPtr, srcPtr + srcLen);
	return;
}

// value of a numeric string field of the ID666 tag (digits, ended by NUL or the end of the field)
static UINT32 ReadNumericField(const UINT8* data, size_t maxlen)
{
	UINT32 value = 0;
	size_t curPos;

	for (curPos = 0; curPos < maxlen; curPos ++)
	{
		if (data[curPos] < '0' || data[curPos] > '9')
			break;
		value = value * 10 + (data[curPos] - '0');
	}
	return value;
}

SPCPlayer::SPCPlayer() :
	_dLoad(NULL),
	_fileLen(0),
	_fileData(NULL),
	_tickFreq(SPC_TICK_RATE),
	_playTicks(0),
	_fadeTicks(0),
	_totalTicks(0),
	_speedNum(1000),
	_tickBase(0),
	_smplBase(0),
	_tsMult(0),
	_tsDiv(SPC_TICK_RATE),
	_playSmpl(0),
	_playTick(0),
	_playState(0x00)
{
	UINT8 retVal;

	memset(&_fileHdr, 0x00, sizeof(SPC_HEADER));
	_playOpts.genOpts.pbSpeed = 0x10000;
	InitDeviceOptions(_devOpts);
	GenerateDeviceConfig();

	retVal = CPConv_Init(&_cpc1252, "CP1252", "UTF-8");
	if (retVal)
		_cpc1252 = NULL;
	_tagList.reserve(16);
	_tagList.push_back(NULL);
}

SPCPlayer::~SPCPlayer()
{
	_eventCbFunc = NULL;	// prevent any callbacks during destruction

	if (_playState & PLAYSTATE_PLAY)
		Stop();
	UnloadFile();

	if (_cpc1252 != NULL)
		CPConv_Deinit(_cpc1252);
}

UINT32 SPCPlayer::GetPlayerType(void) const
{
	return FCC_SPC;
}

const char* SPCPlayer::GetPlayerName(void) const
{
	return "SPC";
}

/*static*/ UINT8 SPCPlayer::PlayerCanLoadFile(DATA_LOADER *dataLoader)
{
	const UINT8* data;

	DataLoader_ReadUntil(dataLoader, SPC_MARKER_OFS + 2);
	if (DataLoader_GetSize(dataLoader) < SPC_MARKER_OFS + 2)
		return 0xF1;	// file too small
	data = DataLoader_GetData(dataLoader);
	if (memcmp(&data[0x00], SPC_MAGIC, SPC_MAGIC_LEN))
		return 0xF0;	// invalid file
	if (data[SPC_MARKER_OFS + 0] != 26 || data[SPC_MARKER_OFS + 1] != 26)
		return 0xF0;	// invalid file

	DataLoader_ReadUntil(dataLoader, SPC_MIN_SIZE);
	if (DataLoader_GetSize(dataLoader) < SPC_MIN_SIZE)
		return 0xF1;	// file too small for a complete machine image
	return 0x00;
}

UINT8 SPCPlayer::CanLoadFile(DATA_LOADER *dataLoader) const
{
	return this->PlayerCanLoadFile(dataLoader);
}

UINT8 SPCPlayer::LoadFile(DATA_LOADER *dataLoader)
{
	UINT8 retVal;

	_dLoad = NULL;
	retVal = PlayerCanLoadFile(dataLoader);
	if (retVal)
		return retVal;

	_dLoad = dataLoader;
	DataLoader_ReadAll(_dLoad);
	_fileData = DataLoader_GetData(_dLoad);
	_fileLen = DataLoader_GetSize(_dLoad);

	_fileHdr.hasID666 = (_fileData[SPC_ID666_FLAG_OFS] == 26);
	_fileHdr.hasXID6 = (_fileLen >= SPC_XID6_OFS + 8 && ! memcmp(&_fileData[SPC_XID6_OFS], "xid6", 4));
	_fileHdr.id666Binary = (_fileHdr.hasID666 && ! IsTextID666(_fileData));

	LoadTags();
	CalcSongLength();

	return 0x00;
}

UINT8 SPCPlayer::UnloadFile(void)
{
	if (_playState & PLAYSTATE_PLAY)
		return 0xFF;

	_playState = 0x00;
	_dLoad = NULL;
	_fileData = NULL;
	_fileLen = 0;
	memset(&_fileHdr, 0x00, sizeof(SPC_HEADER));
	_devices.clear();

	return 0x00;
}

const SPC_HEADER* SPCPlayer::GetFileHeader(void) const
{
	return &_fileHdr;
}

// The ID666 tag has two forms that share their offsets up to the song length: text and binary.
// The format has no indicator for it. The fields that differ are the dump date (11 characters
// "MM/DD/YYYY" or 4 bytes), the fade length (5 or 4 digits) and with that the artist (at 0xB1
// or 0xB0). The tag is taken as text when the dump date is made of digits and '/' (or empty) and
// the song length and the first four digits of the fade length are digits (or empty), as they
// are in every text tag. A binary tag fails that on its date bytes or, with an empty date, on
// the artist's first character in the fade field.
/*static*/ bool SPCPlayer::IsTextID666(const UINT8* hdr)
{
	size_t curPos;

	for (curPos = 0x9E; curPos < 0x9E + 11; curPos ++)
	{
		if (! ((hdr[curPos] >= '0' && hdr[curPos] <= '9') || hdr[curPos] == '/' || hdr[curPos] == '\0'))
			return false;
	}
	for (curPos = 0xA9; curPos < 0xA9 + 7; curPos ++)
	{
		if (! ((hdr[curPos] >= '0' && hdr[curPos] <= '9') || hdr[curPos] == '\0'))
			return false;
	}
	return true;
}

UINT8 SPCPlayer::LoadTags(void)
{
	std::map<std::string, std::string>::const_iterator mapIt;

	_tagData.clear();
	_tagList.clear();
	_id666Secs = 0;
	_id666FadeMs = 0;
	_xHasLen = 0;
	_xIntro = 0;
	_xLoop = 0;
	_xEnd = 0;
	_xLoopCount = 1;
	_xHasFade = 0;
	_xFade = 0;

	if (_fileHdr.hasID666)
		LoadID666Tags(_fileData);
	if (_fileHdr.hasXID6)
		LoadXID6Tags();
	SetTagString("SYSTEM", "Super Nintendo");

	for (mapIt = _tagData.begin(); mapIt != _tagData.end(); ++mapIt)
	{
		_tagList.push_back(mapIt->first.c_str());
		_tagList.push_back(mapIt->second.c_str());
	}
	_tagList.push_back(NULL);
	return 0x00;
}

void SPCPlayer::LoadID666Tags(const UINT8* hdr)
{
	const bool binary = (_fileHdr.id666Binary != 0);

	LoadTag("TITLE",        (const char*)&hdr[0x2E], 32);
	LoadTag("GAME",         (const char*)&hdr[0x4E], 32);
	LoadTag("ENCODED_BY",   (const char*)&hdr[0x6E], 16);
	LoadTag("COMMENT",      (const char*)&hdr[0x7E], 32);
	if (! binary)
	{
		// "MM/DD/YYYY", written as it is when it is not a date of this form
		const char* dateStr = (const char*)&hdr[0x9E];
		size_t dateLen = 0;

		while(dateLen < 11 && dateStr[dateLen] != '\0')
			dateLen ++;
		if (dateLen == 10 && dateStr[2] == '/' && dateStr[5] == '/')
		{
			char isoDate[11];

			sprintf(isoDate, "%.4s-%.2s-%.2s", &dateStr[6], &dateStr[0], &dateStr[3]);
			SetTagString("DATE", isoDate);
		}
		else
		{
			LoadTag("DATE", dateStr, 11);
		}
		LoadTag("ARTIST",   (const char*)&hdr[0xB1], 32);
		_id666FadeMs = ReadNumericField(&hdr[0xAC], 5);
	}
	else
	{
		// 4 bytes (YYYYMMDD as a number), 0 = no date
		UINT32 date = ReadLE32(&hdr[0x9E]);

		if (date >= 10000101 && date <= 99991231)
		{
			char isoDate[16];

			sprintf(isoDate, "%04u-%02u-%02u", (unsigned)(date / 10000), (unsigned)(date / 100 % 100), (unsigned)(date % 100));
			SetTagString("DATE", isoDate);
		}
		LoadTag("ARTIST",   (const char*)&hdr[0xB0], 32);
		_id666FadeMs = ReadNumericField(&hdr[0xAC], 4);
	}
	_id666Secs = ReadNumericField(&hdr[0xA9], 3);
	return;
}

// The extended ID666 chunk: "xid6", the size of what follows (not counting these 8 bytes), then
// sub-chunks of a 4-byte header (ID, type, 16 bit data) and, for a type other than 0, as many data
// bytes as the 16 bit value says, padded to a multiple of 4. A chunk that runs past the end of
// the chunk or of the file ends the list.
void SPCPlayer::LoadXID6Tags(void)
{
	UINT32 chunkSize = ReadLE32(&_fileData[SPC_XID6_OFS + 4]);
	UINT32 pos = SPC_XID6_OFS + 8;
	UINT32 endPos = _fileLen;

	if (chunkSize < endPos - pos)
		endPos = pos + chunkSize;

	while(pos + 4 <= endPos)
	{
		UINT8 id = _fileData[pos + 0];
		UINT8 type = _fileData[pos + 1];
		UINT16 value = _fileData[pos + 2] | (_fileData[pos + 3] << 8);
		const UINT8* data = NULL;
		UINT32 dataLen = 0;

		pos += 4;
		if (type != 0)
		{
			dataLen = value;
			if (dataLen > endPos - pos)
				break;	// the sub-chunk does not fit
			data = &_fileData[pos];
			pos += (dataLen + 3) & ~3U;
		}

		if (type == 1)	// string (the length includes the NUL)
		{
			switch(id)
			{
			case 0x01: LoadTag("TITLE", (const char*)data, dataLen); break;
			case 0x02: LoadTag("GAME", (const char*)data, dataLen); break;
			case 0x03: LoadTag("ARTIST", (const char*)data, dataLen); break;
			case 0x04: LoadTag("ENCODED_BY", (const char*)data, dataLen); break;
			case 0x07: LoadTag("COMMENT", (const char*)data, dataLen); break;
			case 0x13: LoadTag("PUBLISHER", (const char*)data, dataLen); break;
			}
		}
		else if (type == 4 && dataLen >= 4)	// 32 bit integer
		{
			UINT32 number = ReadLE32(data);

			switch(id)
			{
			case 0x05:	// dump date, YYYYMMDD
				if (number >= 10000101 && number <= 99991231)
				{
					char isoDate[16];

					sprintf(isoDate, "%04u-%02u-%02u", (unsigned)(number / 10000), (unsigned)(number / 100 % 100), (unsigned)(number % 100));
					SetTagString("DATE", isoDate);
				}
				break;
			case 0x30:	// introduction length
				_xIntro = (INT32)number;
				_xHasLen = 1;
				break;
			case 0x31:	// loop length
				_xLoop = (INT32)number;
				_xHasLen = 1;
				break;
			case 0x32:	// end length (can be negative)
				_xEnd = (INT32)number;
				_xHasLen = 1;
				break;
			case 0x33:	// fade length
				_xFade = number;
				_xHasFade = 1;
				break;
			}
		}
		else if (type == 0 && id == 0x35)	// number of times to loop (the data is in the header)
		{
			_xLoopCount = (UINT8)value;
		}
	}
	return;
}

void SPCPlayer::LoadTag(const char* tagName, const char* startPtr, size_t maxlen)
{
	const char* endPtr = (const char*)memchr(startPtr, '\0', maxlen);

	if (endPtr == NULL)
		endPtr = startPtr + maxlen;
	SetTagString(tagName, GetUTF8String(startPtr, endPtr));
	return;
}

// An empty value removes the tag (the xid6 chunk can clear what the ID666 tag set).
void SPCPlayer::SetTagString(const char* tagName, const std::string& value)
{
	if (value.empty())
		_tagData.erase(tagName);
	else
		_tagData[tagName] = value;
	return;
}

std::string SPCPlayer::GetUTF8String(const char* startPtr, const char* endPtr)
{
	if (startPtr == endPtr)
		return std::string();

	if (_cpc1252 != NULL)
	{
		size_t convSize = 0;
		char* convData = NULL;
		std::string result;
		UINT8 retVal;

		retVal = CPConv_StrConvert(_cpc1252, &convSize, &convData, endPtr - startPtr, startPtr);

		result.assign(convData, convData + convSize);
		free(convData);
		if (retVal < 0x80)
			return result;
	}
	// unable to convert - fallback using the original string
	return std::string(startPtr, endPtr);
}

// The song length is in the xid6 chunk (introduction + loop * loop count + end, then a fade) or in
// the ID666 tag (seconds, then a fade in milliseconds). A file with neither has no length: it
// plays until the host stops it.
void SPCPlayer::CalcSongLength(void)
{
	INT64 length = 0;

	_playTicks = 0;
	_fadeTicks = 0;
	_totalTicks = 0;

	if (_xHasLen)
	{
		// lengths are in 1/64000 s, the longest is 383999999
		length = (INT64)_xIntro + (INT64)_xLoop * _xLoopCount + (INT64)_xEnd;
		if (length > 383999999)
			length = 383999999;
		if (length > 0)
			_playTicks = (UINT32)(length / XID6_TICK_DIV);
	}
	if (_playTicks == 0 && _id666Secs > 0)
		_playTicks = _id666Secs * SPC_TICK_RATE;
	if (_playTicks == 0)
		return;	// no length: plays forever

	if (_xHasFade)
		_fadeTicks = _xFade / XID6_TICK_DIV;
	else
		_fadeTicks = _id666FadeMs * (SPC_TICK_RATE / 1000);
	_totalTicks = _playTicks + _fadeTicks;
	return;
}

const char* const* SPCPlayer::GetTags(void)
{
	return &_tagList[0];
}

UINT8 SPCPlayer::GetSongInfo(PLR_SONG_INFO& songInf)
{
	if (_dLoad == NULL)
		return 0xFF;

	songInf.format = FCC_SPC;
	songInf.fileVerMaj = 0;
	songInf.fileVerMin = 0;
	songInf.tickRateMul = 1;
	songInf.tickRateDiv = _tickFreq;
	songInf.songLen = GetTotalTicks();	// 0 = no length, plays forever
	songInf.loopTick = (UINT32)-1;
	songInf.volGain = 0x10000;
	songInf.deviceCnt = (UINT32)_devCfgs.size();

	return 0x00;
}

UINT8 SPCPlayer::GetSongDeviceInfo(std::vector<PLR_DEV_INFO>& devInfList) const
{
	if (_dLoad == NULL)
		return 0xFF;

	const DEV_GEN_CFG* devCfg = reinterpret_cast<const DEV_GEN_CFG*>(&_devCfgs[0].data[0]);
	PLR_DEV_INFO devInf;

	devInfList.clear();
	memset(&devInf, 0x00, sizeof(PLR_DEV_INFO));

	devInf.id = 0;
	devInf.parentIdx = (UINT32)-1;
	devInf.type = _devCfgs[0].type;
	devInf.instance = 0;
	devInf.devCfg = devCfg;
	if (! _devices.empty())
	{
		const VGM_BASEDEV& cDev = _devices[0].base;
		devInf.devDecl = cDev.defInf.devDecl;
		devInf.core = (cDev.defInf.devDef != NULL) ? cDev.defInf.devDef->coreID : 0x00;
		devInf.volume = (cDev.resmpl.volumeL + cDev.resmpl.volumeR) / 2;
		devInf.smplRate = cDev.defInf.sampleRate;
	}
	else
	{
		devInf.devDecl = SndEmu_GetDevDecl(devInf.type, _userDevList, _devStartOpts);
		devInf.core = 0x00;
		devInf.volume = _devCfgs[0].volume;
		devInf.smplRate = 0;
	}
	devInfList.push_back(devInf);
	if (! _devices.empty())
		return 0x01;	// returned "live" data
	else
		return 0x00;	// returned data based on the file
}

size_t SPCPlayer::DeviceID2OptionID(UINT32 id) const
{
	DEV_ID type;
	UINT8 instance;

	if (id & 0x80000000)
	{
		type = (id >> 0) & 0xFF;
		instance = (id >> 16) & 0xFF;
	}
	else if (id < _devCfgs.size())
	{
		type = _devCfgs[id].type;
		instance = 0;
	}
	else
	{
		return (size_t)-1;
	}

	if (instance == 0 && type == DEVID_SNES_APU)
		return 0;
	return (size_t)-1;
}

void SPCPlayer::RefreshMuting(SPC_CHIPDEV& chipDev, const PLR_MUTE_OPTS& muteOpts)
{
	DEV_INFO* devInf = &chipDev.base.defInf;
	if (devInf->dataPtr != NULL && devInf->devDef->SetMuteMask != NULL)
		devInf->devDef->SetMuteMask(devInf->dataPtr, muteOpts.chnMute[0]);

	return;
}

UINT8 SPCPlayer::SetDeviceOptions(UINT32 id, const PLR_DEV_OPTS& devOpts)
{
	if (DeviceID2OptionID(id) == (size_t)-1)
		return 0x80;	// bad device ID

	_devOpts = devOpts;
	if (! _devices.empty())
		RefreshMuting(_devices[0], _devOpts.muteOpts);
	return 0x00;
}

UINT8 SPCPlayer::GetDeviceOptions(UINT32 id, PLR_DEV_OPTS& devOpts) const
{
	if (DeviceID2OptionID(id) == (size_t)-1)
		return 0x80;	// bad device ID

	devOpts = _devOpts;
	return 0x00;
}

UINT8 SPCPlayer::SetDeviceMuting(UINT32 id, const PLR_MUTE_OPTS& muteOpts)
{
	if (DeviceID2OptionID(id) == (size_t)-1)
		return 0x80;	// bad device ID

	_devOpts.muteOpts = muteOpts;
	if (! _devices.empty())
		RefreshMuting(_devices[0], _devOpts.muteOpts);
	return 0x00;
}

UINT8 SPCPlayer::GetDeviceMuting(UINT32 id, PLR_MUTE_OPTS& muteOpts) const
{
	if (DeviceID2OptionID(id) == (size_t)-1)
		return 0x80;	// bad device ID

	muteOpts = _devOpts.muteOpts;
	return 0x00;
}

UINT8 SPCPlayer::SetPlayerOptions(const SPC_PLAY_OPTIONS& playOpts)
{
	_playOpts = playOpts;
	return 0x00;
}

UINT8 SPCPlayer::GetPlayerOptions(SPC_PLAY_OPTIONS& playOpts) const
{
	playOpts = _playOpts;
	return 0x00;
}

UINT8 SPCPlayer::SetSampleRate(UINT32 sampleRate)
{
	if (_playState & PLAYSTATE_PLAY)
		return 0x01;	// can't set during playback

	_outSmplRate = sampleRate;
	RefreshTSRates();
	return 0x00;
}

// The audio unit runs at the speed of the console; the playback speed is the speed of its three timers (the tempo
// of the song, the pitch is in the DSP). What the DSP does by itself (the envelopes, the echo) and the speed of
// the CPU keep their speed, as the chips of a VGM do.
double SPCPlayer::GetPlaybackSpeed(void) const
{
	return _speedNum / 1000.0;
}

UINT8 SPCPlayer::SetPlaybackSpeed(double speed)
{
	UINT32 speedNum;

	if (! (speed >= 0.01 && speed <= 1000.0))
		return 0xFF;
	speedNum = (UINT32)(speed * 1000.0 + 0.5);
	if (speedNum == _speedNum)
		return 0x00;

	// the position goes on from where it is
	_tickBase = _playTick;
	_smplBase = _playSmpl;
	_speedNum = speedNum;
	RefreshTSRates();
	if (! _devices.empty())
		RefreshTempo(_devices[0]);
	return 0x00;
}

// The speed to the timers of the audio unit (16.16 fixed point).
void SPCPlayer::RefreshTempo(SPC_CHIPDEV& chipDev)
{
	if (chipDev.base.defInf.dataPtr != NULL && chipDev.tempoWrite != NULL)
		chipDev.tempoWrite(chipDev.base.defInf.dataPtr, (INT32)(((UINT64)_speedNum << 16) / 1000));
	return;
}

void SPCPlayer::RefreshTSRates(void)
{
	_tsMult = (UINT64)_outSmplRate * 1000;
	_tsDiv = (UINT64)_tickFreq * _speedNum;
	return;
}

UINT32 SPCPlayer::Tick2Sample(UINT32 ticks) const
{
	if (ticks == (UINT32)-1)
		return -1;
	if (ticks <= _tickBase)
		return _smplBase;
	return (UINT32)(_smplBase + (ticks - _tickBase) * _tsMult / _tsDiv);
}

UINT32 SPCPlayer::Sample2Tick(UINT32 samples) const
{
	if (samples == (UINT32)-1)
		return -1;
	if (_tsMult == 0)
		return 0;
	if (samples <= _smplBase)
		return _tickBase;
	return (UINT32)(_tickBase + (samples - _smplBase) * _tsDiv / _tsMult);
}

double SPCPlayer::Tick2Second(UINT32 ticks) const
{
	if (ticks == (UINT32)-1)
		return -1.0;
	return ticks / (double)_tickFreq;
}

UINT8 SPCPlayer::GetState(void) const
{
	return _playState;
}

UINT32 SPCPlayer::GetCurPos(UINT8 unit) const
{
	switch(unit)
	{
	case PLAYPOS_TICK:
		return _playTick;
	case PLAYPOS_SAMPLE:
		return _playSmpl;
	case PLAYPOS_FILEOFS:
	case PLAYPOS_COMMAND:
	default:
		return (UINT32)-1;
	}
}

UINT32 SPCPlayer::GetCurLoop(void) const
{
	return 0;
}

UINT32 SPCPlayer::GetTotalTicks(void) const
{
	return _totalTicks;
}

UINT32 SPCPlayer::GetLoopTicks(void) const
{
	return 0;
}

void SPCPlayer::GenerateDeviceConfig(void)
{
	DEV_GEN_CFG devCfg;

	_devCfgs.clear();
	_devCfgs.resize(1);
	memset(&devCfg, 0x00, sizeof(DEV_GEN_CFG));
	devCfg.clock = 0;	// the audio unit runs at its own rate: 32000 Hz output
	_devCfgs[0].type = DEVID_SNES_APU;
	_devCfgs[0].volume = 0x100;
	SaveDeviceConfig(_devCfgs[0].data, &devCfg, sizeof(DEV_GEN_CFG));

	return;
}

UINT8 SPCPlayer::Start(void)
{
	SPC_CHIPDEV* cDev;
	DEV_GEN_CFG* devCfg = reinterpret_cast<DEV_GEN_CFG*>(&_devCfgs[0].data[0]);
	VGM_BASEDEV* clDev;
	UINT8 retVal;

	if (_dLoad == NULL)
		return 0xFF;

	_devices.clear();
	_devices.resize(1);
	cDev = &_devices[0];
	cDev->base.defInf.dataPtr = NULL;
	cDev->base.linkDev = NULL;
	cDev->memWrite = NULL;
	cDev->tempoWrite = NULL;

	devCfg->emuCore = _devOpts.emuCore[0];
	devCfg->srMode = _devOpts.srMode;
	devCfg->smplRate = _devOpts.smplRate ? _devOpts.smplRate : _outSmplRate;

	retVal = SndEmu_Start2(_devCfgs[0].type, devCfg, &cDev->base.defInf, _userDevList, _devStartOpts);
	if (retVal)
	{
		_devices.clear();
		return retVal;
	}
	retVal = SndEmu_GetDeviceFunc(cDev->base.defInf.devDef, RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, (void**)&cDev->memWrite);
	if (retVal == EERR_NOT_FOUND || cDev->memWrite == NULL)
	{
		FreeDeviceTree(&cDev->base, 0);
		_devices.clear();
		return 0xFF;
	}
	// the whole file is the machine image: the device keeps it for Reset
	cDev->memWrite(cDev->base.defInf.dataPtr, 0x00, _fileLen, _fileData);
	SndEmu_GetDeviceFunc(cDev->base.defInf.devDef, RWF_TEMPO | RWF_WRITE, DEVRW_VALUE, 0, (void**)&cDev->tempoWrite);
	RefreshTempo(*cDev);

	SetupLinkedDevices(&cDev->base, NULL, NULL);
	if (cDev->base.defInf.devDef->SetOptionBits != NULL)
		cDev->base.defInf.devDef->SetOptionBits(cDev->base.defInf.dataPtr, _devOpts.coreOpts);
	RefreshMuting(*cDev, _devOpts.muteOpts);

	for (clDev = &cDev->base; clDev != NULL; clDev = clDev->linkDev)
	{
		Resmpl_SetVals(&clDev->resmpl, _devOpts.resmplMode, _devCfgs[0].volume, _outSmplRate);
		Resmpl_DevConnect(&clDev->resmpl, &clDev->defInf);
		Resmpl_Init(&clDev->resmpl);
	}

	_playState |= PLAYSTATE_PLAY;
	Reset();
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_START, NULL);

	return 0x00;
}

UINT8 SPCPlayer::Stop(void)
{
	size_t curDev;

	_playState &= ~PLAYSTATE_PLAY;

	for (curDev = 0; curDev < _devices.size(); curDev ++)
		FreeDeviceTree(&_devices[curDev].base, 0);
	_devices.clear();
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_STOP, NULL);

	return 0x00;
}

// Back to the start of the song: the device returns to the machine state of the file.
UINT8 SPCPlayer::Reset(void)
{
	size_t curDev;

	_playTick = 0;
	_playSmpl = 0;
	_tickBase = 0;
	_smplBase = 0;
	_playState &= ~PLAYSTATE_END;

	RefreshTSRates();

	for (curDev = 0; curDev < _devices.size(); curDev ++)
	{
		SPC_CHIPDEV* cDev = &_devices[curDev];
		if (cDev->base.defInf.dataPtr == NULL)
			continue;

		cDev->base.defInf.devDef->Reset(cDev->base.defInf.dataPtr);
	}

	return 0x00;
}

UINT8 SPCPlayer::Seek(UINT8 unit, UINT32 pos)
{
	switch(unit)
	{
	case PLAYPOS_SAMPLE:
		// (before the current stretch of the conversion, the song restarts, which also restarts the conversion)
		if (pos < _smplBase)
			Reset();
		pos = Sample2Tick(pos);
		// fall through
	case PLAYPOS_TICK:
		_playState |= PLAYSTATE_SEEK;
		if (pos < _playTick)
			Reset();
		return SeekToTick(pos);
	case PLAYPOS_FILEOFS:
	case PLAYPOS_COMMAND:
	default:
		return 0xFF;
	}
}

// Seeking runs the audio unit up to the position and throws the frames away.
UINT8 SPCPlayer::SeekToTick(UINT32 tick)
{
	static const UINT32 SEEK_CHUNK = 4096;
	DEV_SMPL seekBufL[SEEK_CHUNK];
	DEV_SMPL seekBufR[SEEK_CHUNK];
	DEV_SMPL* seekBuf[2];

	_playState |= PLAYSTATE_SEEK;
	seekBuf[0] = seekBufL;
	seekBuf[1] = seekBufR;
	if (tick > _playTick && ! _devices.empty() && _devices[0].base.defInf.dataPtr != NULL)
	{
		DEV_INFO* devInf = &_devices[0].base.defInf;
		// the frames of the audio unit: the ticks of the song are those of speed 1
		UINT32 todo = (UINT32)((UINT64)(tick - _playTick) * 1000 / _speedNum);

		while(todo > 0)
		{
			UINT32 curCnt = (todo > SEEK_CHUNK) ? SEEK_CHUNK : todo;
			devInf->devDef->Update(devInf->dataPtr, curCnt, seekBuf);
			todo -= curCnt;
		}
	}
	_playTick = tick;
	_playSmpl = Tick2Sample(_playTick);
	_playState &= ~PLAYSTATE_SEEK;
	if (_totalTicks > 0 && _playTick >= _totalTicks)
		DoFileEnd();
	return 0x00;
}

// The tagged fade: linear from the full volume at the end of the song's length to silence at its end.
void SPCPlayer::ApplyFade(UINT32 smplCnt, WAVE_32BS* data) const
{
	UINT32 fadeStart = Tick2Sample(_playTicks);
	UINT32 fadeEnd = Tick2Sample(_totalTicks);
	UINT32 curSmpl;

	if (_fadeTicks == 0 || _playSmpl + smplCnt <= fadeStart || fadeEnd <= fadeStart)
		return;

	for (curSmpl = 0; curSmpl < smplCnt; curSmpl ++)
	{
		UINT32 pos = _playSmpl + curSmpl;
		UINT32 gain;

		if (pos < fadeStart)
			continue;
		gain = (UINT32)(((UINT64)(fadeEnd - pos) << 16) / (fadeEnd - fadeStart));
		data[curSmpl].L = (INT32)(((INT64)data[curSmpl].L * gain) >> 16);
		data[curSmpl].R = (INT32)(((INT64)data[curSmpl].R * gain) >> 16);
	}
	return;
}

UINT32 SPCPlayer::Render(UINT32 smplCnt, WAVE_32BS* data)
{
	UINT32 endSmpl;
	bool reachedEnd = false;

	if (smplCnt == 0 || data == NULL || _devices.empty())
		return 0;
	if (_playState & PLAYSTATE_END)
	{
		// past the end: the host is expected to stop, until it does the output is silent
		_playSmpl += smplCnt;
		return smplCnt;
	}

	if (_totalTicks > 0)
	{
		endSmpl = Tick2Sample(_totalTicks);
		if (_playSmpl >= endSmpl)
		{
			DoFileEnd();
			return 0;
		}
		if (smplCnt >= endSmpl - _playSmpl)
		{
			smplCnt = endSmpl - _playSmpl;
			reachedEnd = true;
		}
	}

	{
		VGM_BASEDEV* clDev;

		for (clDev = &_devices[0].base; clDev != NULL; clDev = clDev->linkDev)
		{
			if (clDev->defInf.dataPtr != NULL && ! (_devOpts.muteOpts.disable & 0x01))
				Resmpl_Execute(&clDev->resmpl, smplCnt, data);
		}
	}
	if (_totalTicks > 0)
		ApplyFade(smplCnt, data);
	_playSmpl += smplCnt;
	_playTick = Sample2Tick(_playSmpl);
	if (reachedEnd)
		DoFileEnd();

	return smplCnt;
}

void SPCPlayer::DoFileEnd(void)
{
	_playState |= PLAYSTATE_END;
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_END, NULL);

	return;
}
