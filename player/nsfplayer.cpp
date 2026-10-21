#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <vector>

#include "../common_def.h"
#include "nsfplayer.hpp"
#include "../emu/EmuStructs.h"
#include "../emu/SoundEmu.h"
#include "../emu/Resampler.h"
#include "../emu/SoundDevs.h"
#include "../utils/StrUtils.h"
#include "../emu/cores/nes_filter.h"
#include "helper.h"


// file layout
#define NSF_MAGIC			"NESM\x1A"
#define NSF_MAGIC_LEN		5
#define NSFE_MAGIC			"NSFE"
#define NSFE_MAGIC_LEN		4
#define NSF_HDR_SIZE		0x80

#define NSF_TICK_RATE		1000	// a tick is a millisecond (the unit of the NSFe 'time' and 'fade' chunks)

// The CPU returns from INIT and PLAY to this address: the player pushes it as the return address
// and watches the program counter. Nothing is mapped here ($4018-$40FF is not used by any chip).
#define NSF_HALT_ADDR		0x4100
// INIT and PLAY start with the return address on the stack at $01FE/$01FF
#define NSF_STACK_START		0xFD

// the CPU clock of the regions
static const UINT32 REGION_CLOCK[3] = {1789773, 1662607, 1773448};
// the rate of the frame (NMI) of the regions, as PLAY speed in microseconds, for a file without its own
static const UINT16 REGION_PLAY_US[3] = {16639, 19997, 19997};

// The level of each device with the volume of KIND_VOLUME, as the NSFe 'mixe' chunk measures it: the millibels
// (1/100 dB) of a single channel at its maximum volume against the APU square (harness/level_test.cpp measures
// these). A 'mixe' entry of the file scales the volume by the difference to this.
// 0 = APU squares, 1 = APU triangle / noise / DPCM, 2 = VRC6, 3 = VRC7, 4 = FDS, 5 = MMC5, 6 = N163, 7 = 5B
// The defaults of the chunk are 0, -20, 0, 1100, 700, 0, 1100 or 1900, -130. The cores of the BeesNES chips (VRC6,
// N163, 5B) keep the relative levels of BeesNES (N163: 17.9 dB, the high end of the 11-19.5 dB the boards
// of the games have; 5B: 1.5 dB above the default), the VRC7 volume is set for the default.
static const INT16 MIXE_NATIVE[8] = {0, 57, -25, 1100, 697, 0, 1790, 18};
// the 'mixe' device of each kind of device
static const UINT8 KIND_MIXE_DEV[6] = {0, 2, 3, 5, 6, 7};

// volume of each kind of device (8.8 fixed point, 0x100 = 100%) without a 'mixe' chunk. The BeesNES cores
// and the MMC5 are scaled to the APU, so they use its volume (0xC0, as in a VGM). The YM2413 of the VRC7
// is 0x368 for 11 dB over the APU square.
static const UINT16 KIND_VOLUME[6] = {0xC0, 0xC0, 0x368, 0xC0, 0xC0, 0xC0};

INLINE UINT16 ReadLE16(const UINT8* data)
{
	return	((UINT16)data[0x01] <<  8) | ((UINT16)data[0x00] <<  0);
}

INLINE UINT32 ReadLE24(const UINT8* data)
{
	return	((UINT32)data[0x02] << 16) | ((UINT32)data[0x01] <<  8) | ((UINT32)data[0x00] <<  0);
}

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

struct NSFPlayer::NSF_VRC7_FILTER
{
	NES_OUTPUT_FILTER l;
	NES_OUTPUT_FILTER r;
};

NSFPlayer::NSFPlayer() :
	_dLoad(NULL),
	_fileLen(0),
	_fileData(NULL),
	_romData(NULL),
	_romLen(0),
	_tickFreq(NSF_TICK_RATE),
	_curSong(0),
	_playTicks(0),
	_fadeTicks(0),
	_totalTicks(0),
	_tsMult(0),
	_tsDiv(NSF_TICK_RATE),
	_phase(PHASE_IDLE),
	_cpuCycle(0),
	_rBuf(NULL),
	_rBase(0),
	_rEnd(0),
	_devSmpl(0),
	_vrc7Filter(NULL),
	_playSmpl(0),
	_playTick(0),
	_playState(0x00)
{
	UINT8 retVal;

	memset(&_fileHdr, 0x00, sizeof(NSF_HEADER));
	_vrc7Filter = new NSF_VRC7_FILTER;
	memset(_dev, 0x00, sizeof(_dev));
	memset(_mem, 0x00, sizeof(_mem));
	memset(_banks, 0x00, sizeof(_banks));
	_region = NSF_REGION_NTSC;
	_cpuClock = REGION_CLOCK[NSF_REGION_NTSC];
	_playPeriodUs = REGION_PLAY_US[NSF_REGION_NTSC];
	_mulA = 0;
	_mulB = 0;
	_playBase = 0;
	_playIndex = 0;
	_nextPlay = 0;
	_hasRate = false;
	_hdrStringsUTF8 = false;
	_vrc7Variant = 0;
	memset(_mixe, 0x00, sizeof(_mixe));
	_mixeSet = 0x00;
	_playOpts.genOpts.pbSpeed = 0x10000;

	memset(&_cpu, 0x00, sizeof(_cpu));
	_cpu.read = &NSFPlayer::BusRead;
	_cpu.write = &NSFPlayer::BusWrite;
	_cpu.ctx = this;

	retVal = CPConv_Init(&_cpc1252, "CP1252", "UTF-8");
	if (retVal)
		_cpc1252 = NULL;
	_tagList.reserve(16);
	_tagList.push_back(NULL);
}

NSFPlayer::~NSFPlayer()
{
	_eventCbFunc = NULL;	// prevent any callbacks during destruction

	if (_playState & PLAYSTATE_PLAY)
		Stop();
	UnloadFile();

	if (_cpc1252 != NULL)
		CPConv_Deinit(_cpc1252);
	delete _vrc7Filter;
}

UINT32 NSFPlayer::GetPlayerType(void) const
{
	return FCC_NSF;
}

const char* NSFPlayer::GetPlayerName(void) const
{
	return "NSF";
}

/*static*/ UINT8 NSFPlayer::PlayerCanLoadFile(DATA_LOADER *dataLoader)
{
	const UINT8* data;

	DataLoader_ReadUntil(dataLoader, NSF_MAGIC_LEN);
	if (DataLoader_GetSize(dataLoader) < NSFE_MAGIC_LEN)
		return 0xF1;	// file too small
	data = DataLoader_GetData(dataLoader);
	if (! memcmp(&data[0x00], NSFE_MAGIC, NSFE_MAGIC_LEN))
		return 0x00;	// NSFe, the chunks are checked when loading
	if (DataLoader_GetSize(dataLoader) < NSF_MAGIC_LEN || memcmp(&data[0x00], NSF_MAGIC, NSF_MAGIC_LEN))
		return 0xF0;	// invalid file

	DataLoader_ReadUntil(dataLoader, NSF_HDR_SIZE);
	if (DataLoader_GetSize(dataLoader) < NSF_HDR_SIZE)
		return 0xF1;	// file too small for the header
	return 0x00;
}

UINT8 NSFPlayer::CanLoadFile(DATA_LOADER *dataLoader) const
{
	return this->PlayerCanLoadFile(dataLoader);
}

// Error codes of LoadFile(): 0xF0 invalid file, 0xF1 file too small, 0xF2 the file needs a feature the
// player does not have (a mandatory chunk it does not know, the NSF2 non-returning INIT).
UINT8 NSFPlayer::LoadFile(DATA_LOADER *dataLoader)
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

	ClearFileData();
	if (! memcmp(&_fileData[0x00], NSFE_MAGIC, NSFE_MAGIC_LEN))
		retVal = ParseNSFe();
	else
		retVal = ParseNSF();
	if (! retVal)
		retVal = FinishLoad();
	if (retVal)
	{
		ClearFileData();
		_dLoad = NULL;
		_fileData = NULL;
		_fileLen = 0;
		return retVal;
	}

	RefreshSongTags();
	CalcSongLength();
	GenerateDeviceConfig();

	return 0x00;
}

UINT8 NSFPlayer::UnloadFile(void)
{
	if (_playState & PLAYSTATE_PLAY)
		return 0xFF;

	_playState = 0x00;
	_dLoad = NULL;
	_fileData = NULL;
	_fileLen = 0;
	ClearFileData();
	_devices.clear();
	_devCfgs.clear();
	_devOpts.clear();

	return 0x00;
}

void NSFPlayer::ClearFileData(void)
{
	memset(&_fileHdr, 0x00, sizeof(NSF_HEADER));
	_hasRate = false;
	_romData = NULL;
	_romLen = 0;
	_name.clear();
	_artist.clear();
	_copyright.clear();
	_ripper.clear();
	_text.clear();
	_hdrStringsUTF8 = false;
	_vrc7Variant = 0;
	_vrc7Patches.clear();
	_playlist.clear();
	_trackTime.clear();
	_trackFade.clear();
	_trackLabel.clear();
	_trackAuthor.clear();
	memset(_mixe, 0x00, sizeof(_mixe));
	_mixeSet = 0x00;
	_curSong = 0;
	_tagData.clear();
	_tagList.clear();
	_tagList.push_back(NULL);
	return;
}

const NSF_HEADER* NSFPlayer::GetFileHeader(void) const
{
	return &_fileHdr;
}

// The region bits of the NSF header and of the NSFe 'INFO' chunk: bit 0 = PAL, bit 1 = dual.
// A dual file runs as NTSC, unless the 'regn' chunk says otherwise.
static void SetRegionBits(NSF_HEADER& hdr, UINT8 bits)
{
	if (bits & 0x02)
	{
		hdr.regionMask = 0x03;
		hdr.regionPref = NSF_REGION_NTSC;
	}
	else if (bits & 0x01)
	{
		hdr.regionMask = 0x02;
		hdr.regionPref = NSF_REGION_PAL;
	}
	else
	{
		hdr.regionMask = 0x01;
		hdr.regionPref = NSF_REGION_NTSC;
	}
	return;
}

UINT8 NSFPlayer::ParseNSF(void)
{
	const UINT8* hdr = _fileData;
	UINT8 version = hdr[0x05];
	UINT32 progLen;

	_fileHdr.fileType = (version >= 2) ? NSF_FILE_NSF2 : NSF_FILE_NSF;
	_fileHdr.songCount = hdr[0x06];
	if (_fileHdr.songCount == 0)
		return 0xF0;
	_fileHdr.startSong = hdr[0x07] ? (hdr[0x07] - 1) : 0;
	if (_fileHdr.startSong >= _fileHdr.songCount)
		_fileHdr.startSong = 0;
	_fileHdr.loadAddr = ReadLE16(&hdr[0x08]);
	_fileHdr.initAddr = ReadLE16(&hdr[0x0A]);
	_fileHdr.playAddr = ReadLE16(&hdr[0x0C]);
	_fileHdr.speedNTSC = ReadLE16(&hdr[0x6E]);
	memcpy(_fileHdr.bankInit, &hdr[0x70], 8);
	_fileHdr.bankSwitched = 0;
	for (size_t curBank = 0; curBank < 8; curBank ++)
	{
		if (_fileHdr.bankInit[curBank])
			_fileHdr.bankSwitched = 1;
	}
	_fileHdr.speedPAL = ReadLE16(&hdr[0x78]);
	SetRegionBits(_fileHdr, hdr[0x7A]);
	_fileHdr.chips = hdr[0x7B];
	_fileHdr.nsf2Flags = (version >= 2) ? hdr[0x7C] : 0x00;
	_hasRate = true;

	_hdrStringsUTF8 = (version >= 2);
	{
		const char* strFields[3] = {(const char*)&hdr[0x0E], (const char*)&hdr[0x2E], (const char*)&hdr[0x4E]};
		std::string* strDest[3] = {&_name, &_artist, &_copyright};
		size_t curStr;

		for (curStr = 0; curStr < 3; curStr ++)
		{
			const char* endPtr = (const char*)memchr(strFields[curStr], '\0', 32);

			if (endPtr == NULL)
				endPtr = strFields[curStr] + 32;
			*strDest[curStr] = GetUTF8String(strFields[curStr], endPtr, _hdrStringsUTF8);
		}
	}

	_romData = &hdr[NSF_HDR_SIZE];
	_romLen = _fileLen - NSF_HDR_SIZE;
	progLen = ReadLE24(&hdr[0x7D]);
	if (progLen != 0 && progLen < _romLen)
	{
		// the metadata chunks of NSFe follow the program data
		_romLen = progLen;
		return ParseChunks(NSF_HDR_SIZE + progLen, false);
	}
	return 0x00;
}

UINT8 NSFPlayer::ParseNSFe(void)
{
	UINT8 retVal;

	_fileHdr.fileType = NSF_FILE_NSFE;
	_hdrStringsUTF8 = true;
	retVal = ParseChunks(NSFE_MAGIC_LEN, true);
	if (retVal)
		return retVal;
	// the chunks INFO and DATA are required
	if (_fileHdr.songCount == 0 || _romData == NULL)
		return 0xF0;
	return 0x00;
}

// Reads a null-terminated string. usedLen is the number of bytes up to and including the NUL
// (or all of them when the NUL is missing).
/*static*/ std::string NSFPlayer::ReadString(const UINT8* data, size_t len, size_t* usedLen)
{
	const UINT8* endPtr = (const UINT8*)memchr(data, '\0', len);

	if (endPtr == NULL)
	{
		*usedLen = len;
		return std::string((const char*)data, len);
	}
	*usedLen = (size_t)(endPtr - data) + 1;
	return std::string((const char*)data, (size_t)(endPtr - data));
}

// The chunks of NSFe, and of the metadata after the data of an NSF2. A chunk is the size of its data,
// four characters of ID and the data. A chunk that starts with a capital letter is mandatory: the
// player must know it, or it can not play the file.
UINT8 NSFPlayer::ParseChunks(UINT32 pos, bool isNSFe)
{
	while(pos + 8 <= _fileLen)
	{
		UINT32 size = ReadLE32(&_fileData[pos + 0]);
		const UINT8* id = &_fileData[pos + 4];
		const UINT8* data = &_fileData[pos + 8];
		UINT32 avail = _fileLen - (pos + 8);
		size_t curItem;

		if (size > avail)
			size = avail;	// the chunk runs past the end of the file: use what is there
		pos += 8 + size;

		if (! memcmp(id, "NEND", 4))
		{
			break;
		}
		else if (! memcmp(id, "INFO", 4))
		{
			if (! isNSFe)
				continue;	// not allowed in the metadata of an NSF2
			if (size < 9)
				return 0xF0;
			_fileHdr.loadAddr = ReadLE16(&data[0x00]);
			_fileHdr.initAddr = ReadLE16(&data[0x02]);
			_fileHdr.playAddr = ReadLE16(&data[0x04]);
			SetRegionBits(_fileHdr, data[0x06]);
			_fileHdr.chips = data[0x07];
			_fileHdr.songCount = data[0x08];
			_fileHdr.startSong = (size >= 10) ? data[0x09] : 0;
			if (_fileHdr.songCount == 0)
				return 0xF0;
			if (_fileHdr.startSong >= _fileHdr.songCount)
				_fileHdr.startSong = 0;
		}
		else if (! memcmp(id, "DATA", 4))
		{
			if (isNSFe)
			{
				_romData = data;
				_romLen = size;
			}
		}
		else if (! memcmp(id, "BANK", 4))
		{
			if (isNSFe)
			{
				memset(_fileHdr.bankInit, 0x00, sizeof(_fileHdr.bankInit));
				for (curItem = 0; curItem < 8 && curItem < size; curItem ++)
					_fileHdr.bankInit[curItem] = data[curItem];
				_fileHdr.bankSwitched = 1;	// the chunk says to use bankswitching
			}
		}
		else if (! memcmp(id, "NSF2", 4))
		{
			if (isNSFe && size >= 1)
				_fileHdr.nsf2Flags = data[0x00];
		}
		else if (! memcmp(id, "RATE", 4))
		{
			if (size >= 2)
				_fileHdr.speedNTSC = ReadLE16(&data[0x00]);
			if (size >= 4)
				_fileHdr.speedPAL = ReadLE16(&data[0x02]);
			if (size >= 6)
				_fileHdr.speedDendy = ReadLE16(&data[0x04]);
			_hasRate = true;
		}
		else if (! memcmp(id, "VRC7", 4))
		{
			if (size >= 1)
			{
				_vrc7Variant = data[0x00] ? 1 : 0;
				if (size - 1 >= 152)
					_vrc7Patches.assign(&data[0x01], &data[0x01 + 152]);
				else if (size - 1 >= 128)
					_vrc7Patches.assign(&data[0x01], &data[0x01 + 128]);
			}
		}
		else if (! memcmp(id, "plst", 4))
		{
			_playlist.assign(data, data + size);
		}
		else if (! memcmp(id, "time", 4))
		{
			_trackTime.clear();
			for (curItem = 0; curItem + 4 <= size; curItem += 4)
				_trackTime.push_back((INT32)ReadLE32(&data[curItem]));
		}
		else if (! memcmp(id, "fade", 4))
		{
			_trackFade.clear();
			for (curItem = 0; curItem + 4 <= size; curItem += 4)
				_trackFade.push_back((INT32)ReadLE32(&data[curItem]));
		}
		else if (! memcmp(id, "tlbl", 4) || ! memcmp(id, "taut", 4))
		{
			std::vector<std::string>& dest = (id[1] == 'l') ? _trackLabel : _trackAuthor;
			size_t strPos = 0;

			dest.clear();
			while(strPos < size)
			{
				size_t used;

				dest.push_back(ReadString(&data[strPos], size - strPos, &used));
				strPos += used;
			}
		}
		else if (! memcmp(id, "auth", 4))
		{
			std::string* strDest[4] = {&_name, &_artist, &_copyright, &_ripper};
			size_t strPos = 0;

			for (curItem = 0; curItem < 4 && strPos < size; curItem ++)
			{
				size_t used;

				*strDest[curItem] = ReadString(&data[strPos], size - strPos, &used);
				strPos += used;
			}
		}
		else if (! memcmp(id, "text", 4))
		{
			size_t used;

			_text = ReadString(data, size, &used);
		}
		else if (! memcmp(id, "mixe", 4))
		{
			for (curItem = 0; curItem + 3 <= size; curItem += 3)
			{
				UINT8 devNum = data[curItem];

				if (devNum < 8)
				{
					_mixe[devNum] = (INT16)ReadLE16(&data[curItem + 1]);
					_mixeSet |= (1 << devNum);
				}
			}
		}
		else if (! memcmp(id, "regn", 4))
		{
			if (size >= 1 && (data[0x00] & 0x07))
			{
				_fileHdr.regionMask = data[0x00] & 0x07;
				if (size >= 2 && data[0x01] <= NSF_REGION_DENDY && (_fileHdr.regionMask & (1 << data[0x01])))
					_fileHdr.regionPref = data[0x01];
				else
					_fileHdr.regionPref = (_fileHdr.regionMask & 0x01) ? NSF_REGION_NTSC :
										(_fileHdr.regionMask & 0x02) ? NSF_REGION_PAL : NSF_REGION_DENDY;
			}
		}
		else if (id[0] >= 'A' && id[0] <= 'Z')
		{
			return 0xF2;	// a mandatory chunk that the player does not know
		}
		// other chunks (psfx, ...) are not used
	}
	return 0x00;
}

// Checks what the parts of the file say about the song and fixes the region and the play speed.
UINT8 NSFPlayer::FinishLoad(void)
{
	if (_fileHdr.songCount == 0 || _romData == NULL)
		return 0xF0;
	if (_fileHdr.nsf2Flags & 0x20)
		return 0xF2;	// non-returning INIT: the song runs inside INIT and PLAY is an NMI inside it
	if (! _fileHdr.bankSwitched && _fileHdr.loadAddr < 0x6000)
		return 0xF0;	// no memory below $6000 to load the program to
	if (_fileHdr.initAddr < 0x6000 || (_fileHdr.playAddr != 0 && _fileHdr.playAddr < 0x6000))
		return 0xF0;

	InitRegion();
	_curSong = _fileHdr.startSong;
	return 0x00;
}

// The region the song is played in sets the CPU clock and the speed of PLAY.
void NSFPlayer::InitRegion(void)
{
	UINT16 speed;

	_region = _fileHdr.regionPref;
	if (_region > NSF_REGION_DENDY || ! (_fileHdr.regionMask & (1 << _region)))
		_region = NSF_REGION_NTSC;
	_cpuClock = REGION_CLOCK[_region];

	switch(_region)
	{
	case NSF_REGION_PAL:
		speed = _fileHdr.speedPAL;
		break;
	case NSF_REGION_DENDY:
		speed = _fileHdr.speedDendy ? _fileHdr.speedDendy : _fileHdr.speedPAL;
		break;
	default:
		speed = _fileHdr.speedNTSC;
		break;
	}
	// a file without a speed (or with a nonsense one) plays at the rate of the frame
	if (speed < 100)
		speed = REGION_PLAY_US[_region];
	_playPeriodUs = speed;
	return;
}

std::string NSFPlayer::GetUTF8String(const char* startPtr, const char* endPtr, bool isUTF8)
{
	if (startPtr == endPtr)
		return std::string();

	if (! isUTF8 && _cpc1252 != NULL)
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

// An empty value, and the "<?>" of an unknown field, remove the tag.
void NSFPlayer::SetTagString(const char* tagName, const std::string& value)
{
	if (value.empty() || value == "<?>")
		_tagData.erase(tagName);
	else
		_tagData[tagName] = value;
	return;
}

// The tags of the file and the ones of the song.
void NSFPlayer::RefreshSongTags(void)
{
	std::map<std::string, std::string>::const_iterator mapIt;

	_tagData.clear();
	_tagList.clear();

	SetTagString("GAME", _name);
	if (_curSong < _trackAuthor.size() && ! _trackAuthor[_curSong].empty())
		SetTagString("ARTIST", _trackAuthor[_curSong]);
	else
		SetTagString("ARTIST", _artist);
	if (_curSong < _trackLabel.size())
		SetTagString("TITLE", _trackLabel[_curSong]);
	SetTagString("COPYRIGHT", _copyright);
	SetTagString("ENCODED_BY", _ripper);
	SetTagString("COMMENT", _text);
	SetTagString("SYSTEM", "Nintendo Entertainment System");

	for (mapIt = _tagData.begin(); mapIt != _tagData.end(); ++mapIt)
	{
		_tagList.push_back(mapIt->first.c_str());
		_tagList.push_back(mapIt->second.c_str());
	}
	_tagList.push_back(NULL);
	return;
}

// The length of a song is the 'time' and 'fade' of its track, in milliseconds. A song without a time
// has no length: it plays until the host stops it.
void NSFPlayer::CalcSongLength(void)
{
	_playTicks = 0;
	_fadeTicks = 0;
	_totalTicks = 0;

	if (_curSong < _trackTime.size() && _trackTime[_curSong] > 0)
		_playTicks = (UINT32)_trackTime[_curSong];
	if (_playTicks == 0)
		return;	// no length: plays forever

	if (_curSong < _trackFade.size() && _trackFade[_curSong] > 0)
		_fadeTicks = (UINT32)_trackFade[_curSong];
	_totalTicks = _playTicks + _fadeTicks;
	return;
}

const char* const* NSFPlayer::GetTags(void)
{
	return &_tagList[0];
}

UINT8 NSFPlayer::GetSongInfo(PLR_SONG_INFO& songInf)
{
	if (_dLoad == NULL)
		return 0xFF;

	songInf.format = FCC_NSF;
	songInf.fileVerMaj = (_fileHdr.fileType == NSF_FILE_NSF2) ? 2 : 1;
	songInf.fileVerMin = 0;
	songInf.tickRateMul = 1;
	songInf.tickRateDiv = _tickFreq;
	songInf.songLen = GetTotalTicks();	// 0 = no length, plays forever
	songInf.loopTick = (UINT32)-1;
	songInf.volGain = 0x10000;
	songInf.deviceCnt = (UINT32)_devCfgs.size();

	return 0x00;
}

UINT8 NSFPlayer::GetSongDeviceInfo(std::vector<PLR_DEV_INFO>& devInfList) const
{
	size_t curDev;

	if (_dLoad == NULL)
		return 0xFF;

	devInfList.clear();
	for (curDev = 0; curDev < _devCfgs.size(); curDev ++)
	{
		const DEV_GEN_CFG* devCfg = reinterpret_cast<const DEV_GEN_CFG*>(&_devCfgs[curDev].data[0]);
		PLR_DEV_INFO devInf;

		memset(&devInf, 0x00, sizeof(PLR_DEV_INFO));
		devInf.id = (UINT32)curDev;
		devInf.parentIdx = (UINT32)-1;
		devInf.type = _devCfgs[curDev].type;
		devInf.instance = 0;
		devInf.devCfg = devCfg;
		if (curDev < _devices.size())
		{
			const VGM_BASEDEV& cDev = _devices[curDev].base;
			devInf.devDecl = cDev.defInf.devDecl;
			devInf.core = (cDev.defInf.devDef != NULL) ? cDev.defInf.devDef->coreID : 0x00;
			devInf.volume = (cDev.resmpl.volumeL + cDev.resmpl.volumeR) / 2;
			devInf.smplRate = cDev.defInf.sampleRate;
		}
		else
		{
			devInf.devDecl = SndEmu_GetDevDecl(devInf.type, _userDevList, _devStartOpts);
			devInf.core = 0x00;
			devInf.volume = _devCfgs[curDev].volume;
			devInf.smplRate = 0;
		}
		devInfList.push_back(devInf);
	}
	if (! _devices.empty())
		return 0x01;	// returned "live" data
	else
		return 0x00;	// returned data based on the file
}

size_t NSFPlayer::DeviceID2OptionID(UINT32 id) const
{
	size_t curDev;

	if (id & 0x80000000)
	{
		DEV_ID type = (id >> 0) & 0xFF;
		UINT8 instance = (id >> 16) & 0xFF;

		if (instance != 0)
			return (size_t)-1;
		for (curDev = 0; curDev < _devCfgs.size(); curDev ++)
		{
			if (_devCfgs[curDev].type == type)
				return curDev;
		}
		return (size_t)-1;
	}
	if (id < _devCfgs.size())
		return id;
	return (size_t)-1;
}

void NSFPlayer::RefreshMuting(NSF_CHIPDEV& chipDev, const PLR_MUTE_OPTS& muteOpts)
{
	DEV_INFO* devInf = &chipDev.base.defInf;
	if (devInf->dataPtr != NULL && devInf->devDef->SetMuteMask != NULL)
		devInf->devDef->SetMuteMask(devInf->dataPtr, muteOpts.chnMute[0]);

	return;
}

UINT8 NSFPlayer::SetDeviceOptions(UINT32 id, const PLR_DEV_OPTS& devOpts)
{
	size_t optID = DeviceID2OptionID(id);

	if (optID == (size_t)-1)
		return 0x80;	// bad device ID

	_devOpts[optID] = devOpts;
	if (optID < _devices.size())
		RefreshMuting(_devices[optID], _devOpts[optID].muteOpts);
	return 0x00;
}

UINT8 NSFPlayer::GetDeviceOptions(UINT32 id, PLR_DEV_OPTS& devOpts) const
{
	size_t optID = DeviceID2OptionID(id);

	if (optID == (size_t)-1)
		return 0x80;	// bad device ID

	devOpts = _devOpts[optID];
	return 0x00;
}

UINT8 NSFPlayer::SetDeviceMuting(UINT32 id, const PLR_MUTE_OPTS& muteOpts)
{
	size_t optID = DeviceID2OptionID(id);

	if (optID == (size_t)-1)
		return 0x80;	// bad device ID

	_devOpts[optID].muteOpts = muteOpts;
	if (optID < _devices.size())
		RefreshMuting(_devices[optID], _devOpts[optID].muteOpts);
	return 0x00;
}

UINT8 NSFPlayer::GetDeviceMuting(UINT32 id, PLR_MUTE_OPTS& muteOpts) const
{
	size_t optID = DeviceID2OptionID(id);

	if (optID == (size_t)-1)
		return 0x80;	// bad device ID

	muteOpts = _devOpts[optID].muteOpts;
	return 0x00;
}

UINT8 NSFPlayer::SetPlayerOptions(const NSF_PLAY_OPTIONS& playOpts)
{
	_playOpts = playOpts;
	return 0x00;
}

UINT8 NSFPlayer::GetPlayerOptions(NSF_PLAY_OPTIONS& playOpts) const
{
	playOpts = _playOpts;
	return 0x00;
}

UINT16 NSFPlayer::GetSongCount(void) const
{
	return _fileHdr.songCount;
}

UINT16 NSFPlayer::GetCurSong(void) const
{
	return _curSong;
}

const std::vector<UINT8>& NSFPlayer::GetPlaylist(void) const
{
	return _playlist;
}

UINT8 NSFPlayer::SelectSong(UINT16 song)
{
	if (_dLoad == NULL)
		return 0xFF;
	if (song >= _fileHdr.songCount)
		return 0x80;

	_curSong = song;
	RefreshSongTags();
	CalcSongLength();
	if (_playState & PLAYSTATE_PLAY)
		Reset();
	return 0x00;
}

UINT8 NSFPlayer::SetSampleRate(UINT32 sampleRate)
{
	if (_playState & PLAYSTATE_PLAY)
		return 0x01;	// can't set during playback

	_outSmplRate = sampleRate;
	RefreshTSRates();
	return 0x00;
}

// The machine is the song: its speed is fixed by the console, so the playback speed is always 100%.
double NSFPlayer::GetPlaybackSpeed(void) const
{
	return 1.0;
}

UINT8 NSFPlayer::SetPlaybackSpeed(double speed)
{
	return (speed == 1.0) ? 0x00 : 0xFF;
}

void NSFPlayer::RefreshTSRates(void)
{
	_tsMult = _outSmplRate;
	_tsDiv = _tickFreq;
	return;
}

UINT32 NSFPlayer::Tick2Sample(UINT32 ticks) const
{
	if (ticks == (UINT32)-1)
		return -1;
	return (UINT32)(ticks * _tsMult / _tsDiv);
}

UINT32 NSFPlayer::Sample2Tick(UINT32 samples) const
{
	if (samples == (UINT32)-1)
		return -1;
	if (_tsMult == 0)
		return 0;
	return (UINT32)(samples * _tsDiv / _tsMult);
}

double NSFPlayer::Tick2Second(UINT32 ticks) const
{
	if (ticks == (UINT32)-1)
		return -1.0;
	return ticks / (double)_tickFreq;
}

UINT8 NSFPlayer::GetState(void) const
{
	return _playState;
}

UINT32 NSFPlayer::GetCurPos(UINT8 unit) const
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

UINT32 NSFPlayer::GetCurLoop(void) const
{
	return 0;
}

UINT32 NSFPlayer::GetTotalTicks(void) const
{
	return _totalTicks;
}

UINT32 NSFPlayer::GetLoopTicks(void) const
{
	return 0;
}

// The volume of a device: the one of its kind, changed by the 'mixe' chunk of the file. The mix of a device
// is the millibels (1/100 dB) of its loudest output against the APU square.
UINT16 NSFPlayer::GetMixVolume(UINT8 kind, UINT16 baseVol) const
{
	UINT8 mixDev = KIND_MIXE_DEV[kind];
	double gain;
	double volume;

	if (! (_mixeSet & (1 << mixDev)))
		return baseVol;
	gain = pow(10.0, (_mixe[mixDev] - MIXE_NATIVE[mixDev]) / 2000.0);
	volume = baseVol * gain;
	if (volume < 1.0)
		volume = 1.0;
	if (volume > 0x7FFF)
		volume = 0x7FFF;
	return (UINT16)(volume + 0.5);
}

// The devices of the song: the APU (with the FDS), then the chips of the header in the order of
// the DEVKIND_ constants. Their clocks are the CPU clock, only the VRC7 runs at twice of it.
void NSFPlayer::GenerateDeviceConfig(void)
{
	static const DEV_ID KIND_TYPE[DEVKIND_COUNT] = {DEVID_NES_APU, DEVID_VRC6, DEVID_YM2413, DEVID_MMC5, DEVID_N163, DEVID_SUNSOFT5B};
	static const UINT8 KIND_CHIP[DEVKIND_COUNT] = {0x00, NSF_CHIP_VRC6, NSF_CHIP_VRC7, NSF_CHIP_MMC5, NSF_CHIP_N163, NSF_CHIP_5B};
	UINT8 kind;

	_devCfgs.clear();
	_devOpts.clear();
	for (kind = 0; kind < DEVKIND_COUNT; kind ++)
	{
		DevCfg devCfgItem;
		DEV_GEN_CFG devCfg;
		PLR_DEV_OPTS devOpts;

		if (KIND_CHIP[kind] != 0x00 && ! (_fileHdr.chips & KIND_CHIP[kind]))
			continue;

		memset(&devCfg, 0x00, sizeof(DEV_GEN_CFG));
		devCfg.clock = _cpuClock;
		devCfg.flags = 0x00;
		if (kind == DEVKIND_APU)
			devCfg.flags = (_fileHdr.chips & NSF_CHIP_FDS) ? 0x01 : 0x00;	// the FDS is part of the APU device
		else if (kind == DEVKIND_VRC7)
		{
			devCfg.clock = _cpuClock * 2;
			devCfg.flags = _vrc7Variant ? 0x00 : 0x01;	// 1 = VRC7, 0 = YM2413
		}

		devCfgItem.type = KIND_TYPE[kind];
		devCfgItem.kind = kind;
		devCfgItem.volume = GetMixVolume(kind, KIND_VOLUME[kind]);
		SaveDeviceConfig(devCfgItem.data, &devCfg, sizeof(DEV_GEN_CFG));
		_devCfgs.push_back(devCfgItem);

		InitDeviceOptions(devOpts);
		if (kind == DEVKIND_APU)
			devOpts.coreOpts = 0x01B7;	// the default options of the NES core
		_devOpts.push_back(devOpts);
	}
	return;
}

UINT8 NSFPlayer::Start(void)
{
	size_t curDev;
	UINT8 retVal;

	if (_dLoad == NULL)
		return 0xFF;

	_devices.clear();
	_devices.resize(_devCfgs.size());
	memset(_dev, 0x00, sizeof(_dev));
	for (curDev = 0; curDev < _devCfgs.size(); curDev ++)
	{
		NSF_CHIPDEV* cDev = &_devices[curDev];
		DEV_GEN_CFG* devCfg = reinterpret_cast<DEV_GEN_CFG*>(&_devCfgs[curDev].data[0]);
		const PLR_DEV_OPTS& devOpts = _devOpts[curDev];
		const DEV_DEF* devDef;
		VGM_BASEDEV* clDev;

		cDev->base.defInf.dataPtr = NULL;
		cDev->base.linkDev = NULL;
		cDev->write8 = NULL;
		cDev->read8 = NULL;
		cDev->write16 = NULL;
		cDev->read16 = NULL;
		cDev->memWrite = NULL;

		devCfg->emuCore = devOpts.emuCore[0];
		devCfg->srMode = devOpts.srMode;
		devCfg->smplRate = devOpts.smplRate ? devOpts.smplRate : _outSmplRate;

		retVal = SndEmu_Start2(_devCfgs[curDev].type, devCfg, &cDev->base.defInf, _userDevList, _devStartOpts);
		if (retVal)
		{
			for (curDev = 0; curDev < _devices.size(); curDev ++)
				FreeDeviceTree(&_devices[curDev].base, 0);
			_devices.clear();
			memset(_dev, 0x00, sizeof(_dev));
			return retVal;
		}
		devDef = cDev->base.defInf.devDef;
		SndEmu_GetDeviceFunc(devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, (void**)&cDev->write8);
		SndEmu_GetDeviceFunc(devDef, RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, (void**)&cDev->read8);
		SndEmu_GetDeviceFunc(devDef, RWF_REGISTER | RWF_WRITE, DEVRW_A16D8, 0, (void**)&cDev->write16);
		SndEmu_GetDeviceFunc(devDef, RWF_REGISTER | RWF_READ, DEVRW_A16D8, 0, (void**)&cDev->read16);
		SndEmu_GetDeviceFunc(devDef, RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, (void**)&cDev->memWrite);

		if (_devCfgs[curDev].kind == DEVKIND_VRC7 && ! _vrc7Patches.empty() && cDev->memWrite != NULL)
			cDev->memWrite(cDev->base.defInf.dataPtr, 0x00, (UINT32)_vrc7Patches.size(), &_vrc7Patches[0]);

		SetupLinkedDevices(&cDev->base, NULL, NULL);
		if (devDef->SetOptionBits != NULL)
			devDef->SetOptionBits(cDev->base.defInf.dataPtr, devOpts.coreOpts);
		RefreshMuting(*cDev, devOpts.muteOpts);

		for (clDev = &cDev->base; clDev != NULL; clDev = clDev->linkDev)
		{
			Resmpl_SetVals(&clDev->resmpl, devOpts.resmplMode, _devCfgs[curDev].volume, _outSmplRate);
			Resmpl_DevConnect(&clDev->resmpl, &clDev->defInf);
			Resmpl_Init(&clDev->resmpl);
		}
		_dev[_devCfgs[curDev].kind] = cDev;
	}

	// the registers each kind of device must have
	if (_dev[DEVKIND_APU]->write8 == NULL || _dev[DEVKIND_APU]->read8 == NULL ||
		(_dev[DEVKIND_VRC6] != NULL && _dev[DEVKIND_VRC6]->write16 == NULL) ||
		(_dev[DEVKIND_VRC7] != NULL && _dev[DEVKIND_VRC7]->write8 == NULL) ||
		(_dev[DEVKIND_MMC5] != NULL && (_dev[DEVKIND_MMC5]->write16 == NULL || _dev[DEVKIND_MMC5]->read16 == NULL)) ||
		(_dev[DEVKIND_N163] != NULL && (_dev[DEVKIND_N163]->write16 == NULL || _dev[DEVKIND_N163]->read16 == NULL)) ||
		(_dev[DEVKIND_5B] != NULL && _dev[DEVKIND_5B]->write16 == NULL))
	{
		for (curDev = 0; curDev < _devices.size(); curDev ++)
			FreeDeviceTree(&_devices[curDev].base, 0);
		_devices.clear();
		memset(_dev, 0x00, sizeof(_dev));
		return 0xFF;
	}

	_playState |= PLAYSTATE_PLAY;
	Reset();
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_START, NULL);

	return 0x00;
}

UINT8 NSFPlayer::Stop(void)
{
	size_t curDev;

	_playState &= ~PLAYSTATE_PLAY;

	for (curDev = 0; curDev < _devices.size(); curDev ++)
		FreeDeviceTree(&_devices[curDev].base, 0);
	_devices.clear();
	memset(_dev, 0x00, sizeof(_dev));
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_STOP, NULL);

	return 0x00;
}

// Back to the start of the song: the devices reset, the program is loaded again and the CPU
// calls INIT for the song.
UINT8 NSFPlayer::Reset(void)
{
	size_t curDev;

	_playTick = 0;
	_playSmpl = 0;
	_devSmpl = 0;
	nes_output_filter_init(&_vrc7Filter->l, (float)_outSmplRate);
	nes_output_filter_init(&_vrc7Filter->r, (float)_outSmplRate);
	_playState &= ~PLAYSTATE_END;

	RefreshTSRates();

	for (curDev = 0; curDev < _devices.size(); curDev ++)
	{
		NSF_CHIPDEV* cDev = &_devices[curDev];
		if (cDev->base.defInf.dataPtr == NULL)
			continue;

		// the resampler keeps the last samples and its position, which would leak into the new song
		Resmpl_Deinit(&cDev->base.resmpl);
		cDev->base.defInf.devDef->Reset(cDev->base.defInf.dataPtr);
		Resmpl_Init(&cDev->base.resmpl);
	}
	if (! _devices.empty())
		StartSong();

	return 0x00;
}

UINT8 NSFPlayer::Seek(UINT8 unit, UINT32 pos)
{
	switch(unit)
	{
	case PLAYPOS_SAMPLE:
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

// Seeking runs the song up to the position and throws the samples away.
UINT8 NSFPlayer::SeekToTick(UINT32 tick)
{
	static const UINT32 SEEK_CHUNK = 4096;
	std::vector<WAVE_32BS> seekBuf(SEEK_CHUNK);
	UINT32 target;

	_playState |= PLAYSTATE_SEEK;
	target = Tick2Sample(tick);
	while(! _devices.empty() && _playSmpl < target)
	{
		UINT32 curCnt = target - _playSmpl;

		if (curCnt > SEEK_CHUNK)
			curCnt = SEEK_CHUNK;
		memset(&seekBuf[0], 0x00, curCnt * sizeof(WAVE_32BS));
		RenderSamples(curCnt, &seekBuf[0]);
		_playSmpl += curCnt;
	}
	_playTick = tick;
	_playState &= ~PLAYSTATE_SEEK;
	if (_totalTicks > 0 && _playTick >= _totalTicks)
		DoFileEnd();
	return 0x00;
}

// The tagged fade: linear from the full volume at the end of the song's length to silence at its end.
void NSFPlayer::ApplyFade(UINT32 smplCnt, WAVE_32BS* data) const
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

UINT32 NSFPlayer::Render(UINT32 smplCnt, WAVE_32BS* data)
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

	RenderSamples(smplCnt, data);
	if (_totalTicks > 0)
		ApplyFade(smplCnt, data);
	_playSmpl += smplCnt;
	_playTick = Sample2Tick(_playSmpl);
	if (reachedEnd)
		DoFileEnd();

	return smplCnt;
}

void NSFPlayer::DoFileEnd(void)
{
	_playState |= PLAYSTATE_END;
	if (_eventCbFunc != NULL)
		_eventCbFunc(this, _eventCbParam, PLREVT_END, NULL);

	return;
}

// Runs the CPU up to the end of the samples and renders the devices: the bus functions bring the
// devices to the sample of every write to a register before they apply it.
// The output is added to data, the buffer for the samples from the position _playSmpl.
UINT32 NSFPlayer::RenderSamples(UINT32 smplCnt, WAVE_32BS* data)
{
	_rBuf = data;
	_rBase = _playSmpl;
	_rEnd = _playSmpl + smplCnt;

	RunCpu(SampleToCycle(_rEnd));
	SyncDevices(_rEnd);

	_rBuf = NULL;
	return smplCnt;
}

// Renders the devices up to the sample position. Nothing is rendered when it is not in the
// samples of the Render() in progress.
void NSFPlayer::SyncDevices(UINT32 smpl)
{
	size_t curDev;
	UINT32 smplCnt;

	if (_rBuf == NULL)
		return;
	if (smpl > _rEnd)
		smpl = _rEnd;
	if (smpl <= _devSmpl)
		return;

	smplCnt = smpl - _devSmpl;
	for (curDev = 0; curDev < _devices.size(); curDev ++)
	{
		NSF_CHIPDEV* cDev = &_devices[curDev];
		UINT8 disable = _devOpts[curDev].muteOpts.disable;
		VGM_BASEDEV* clDev;

		for (clDev = &cDev->base; clDev != NULL; clDev = clDev->linkDev, disable >>= 1)
		{
			if (clDev->defInf.dataPtr == NULL || (disable & 0x01))
				continue;
			if (cDev == _dev[DEVKIND_VRC7])
				RenderVrc7(clDev, smplCnt, &_rBuf[_devSmpl - _rBase]);
			else
				Resmpl_Execute(&clDev->resmpl, smplCnt, &_rBuf[_devSmpl - _rBase]);
		}
	}
	_devSmpl = smpl;
	return;
}

// The filter is a state that runs over the samples in order, so the result does not depend on how the
// rendering is cut into parts.
void NSFPlayer::RenderVrc7(VGM_BASEDEV* dev, UINT32 smplCnt, WAVE_32BS* out)
{
	UINT32 curSmpl;

	_vrc7Buf.assign(smplCnt, WAVE_32BS());
	memset(&_vrc7Buf[0], 0x00, smplCnt * sizeof(WAVE_32BS));
	Resmpl_Execute(&dev->resmpl, smplCnt, &_vrc7Buf[0]);
	for (curSmpl = 0; curSmpl < smplCnt; curSmpl ++)
	{
		out[curSmpl].L += (INT32)nes_output_filter(&_vrc7Filter->l, (float)_vrc7Buf[curSmpl].L);
		out[curSmpl].R += (INT32)nes_output_filter(&_vrc7Filter->r, (float)_vrc7Buf[curSmpl].R);
	}
}

// the sample a CPU cycle is in
UINT32 NSFPlayer::CycleToSample(UINT64 cycle) const
{
	return (UINT32)(cycle * _outSmplRate / _cpuClock);
}

// the first cycle that is in the sample or after it
UINT64 NSFPlayer::SampleToCycle(UINT32 smpl) const
{
	return ((UINT64)smpl * _cpuClock + _outSmplRate - 1) / _outSmplRate;
}

// The cycle of the call of PLAY with the number `index`, counted from the end of INIT.
UINT64 NSFPlayer::PlayTime(UINT32 index) const
{
	return _playBase + (UINT64)index * _playPeriodUs * _cpuClock / 1000000;
}

// PLAY returned: the next call is the first one of the schedule that is not before the cycle now. A PLAY
// that took longer than its period misses the calls it overran, as it would on the console.
void NSFPlayer::ScheduleNextPlay(void)
{
	do
	{
		_playIndex ++;
		_nextPlay = PlayTime(_playIndex);
	} while(_nextPlay < _cpuCycle);
	return;
}

// The CPU is to run the routine at the address and return to NSF_HALT_ADDR with an RTS.
void NSFPlayer::CallRoutine(UINT16 addr)
{
	_mem[0x01FF] = (UINT8)((NSF_HALT_ADDR - 1) >> 8);
	_mem[0x01FE] = (UINT8)((NSF_HALT_ADDR - 1) & 0xFF);
	_cpu.s = NSF_STACK_START;
	_cpu.pc = addr;
	return;
}

// The CPU runs until the cycle. It calls INIT first, then PLAY at its period. A routine that does not
// return (PLAY of the PCM drivers) just keeps the CPU.
void NSFPlayer::RunCpu(UINT64 cycleLimit)
{
	while(_cpuCycle < cycleLimit)
	{
		if (_phase == PHASE_IDLE)
		{
			if (_cpuCycle < _nextPlay)
			{
				// nothing to run until the next call of PLAY
				if (_nextPlay >= cycleLimit)
				{
					_cpuCycle = cycleLimit;
					break;
				}
				_cpuCycle = _nextPlay;
			}
			CallRoutine(_fileHdr.playAddr);
			_phase = PHASE_PLAY;
		}

		nsf_cpu6502_instruction(&_cpu);
		if (_cpu.pc == NSF_HALT_ADDR)
		{
			if (_phase == PHASE_INIT)
			{
				_playBase = _cpuCycle;
				_playIndex = 0;
				_nextPlay = _cpuCycle;
			}
			else
			{
				ScheduleNextPlay();
			}
			_phase = PHASE_IDLE;
			if (_fileHdr.playAddr == 0)
				_nextPlay = (UINT64)-1;	// a file without PLAY
		}
	}
	return;
}

// Loads the program to the memory the way the file says, for the start of a song: the RAM is cleared and
// the banks have their initial values.
void NSFPlayer::BuildMemory(void)
{
	memset(_mem, 0x00, sizeof(_mem));
	memset(_banks, 0x00, sizeof(_banks));

	if (! _fileHdr.bankSwitched)
	{
		// the data goes to the load address, as much as there is room for
		UINT32 maxLen = 0x10000 - _fileHdr.loadAddr;
		UINT32 copyLen = (_romLen < maxLen) ? _romLen : maxLen;

		memcpy(&_mem[_fileHdr.loadAddr], _romData, copyLen);
		return;
	}

	{
		// 4K banks, after as many bytes as the load address has in its bank
		UINT32 padLen = _fileHdr.loadAddr & 0x0FFF;
		UINT32 bankCnt = (padLen + _romLen + 0x0FFF) / 0x1000;
		UINT8 curBank;

		_bankRom.assign((size_t)bankCnt * 0x1000, 0x00);
		if (_romLen > 0)
			memcpy(&_bankRom[padLen], _romData, _romLen);

		for (curBank = 0; curBank < 8; curBank ++)
			SetBank(0x5FF8 + curBank, _fileHdr.bankInit[curBank]);
		if (_fileHdr.chips & NSF_CHIP_FDS)
		{
			// $6000-$7FFF has the banks that $E000-$FFFF has
			SetBank(0x5FF6, _fileHdr.bankInit[6]);
			SetBank(0x5FF7, _fileHdr.bankInit[7]);
		}
	}
	return;
}

// Switches the bank of a window of 4K ($5FF6/$5FF7: $6000 and $7000 (FDS), $5FF8-$5FFF: $8000-$FFFF).
// The window gets a copy of the bank. The APU keeps its own copy of $8000-$FFFF for the DPCM.
void NSFPlayer::SetBank(UINT16 reg, UINT8 bank)
{
	UINT32 dest;
	UINT32 srcOfs = (UINT32)bank * 0x1000;

	dest = (reg >= 0x5FF8) ? (0x8000 + (UINT32)(reg - 0x5FF8) * 0x1000) : (0x6000 + (UINT32)(reg - 0x5FF6) * 0x1000);
	_banks[(reg >= 0x5FF8) ? (reg - 0x5FF8) : (8 + reg - 0x5FF6)] = bank;
	if (srcOfs + 0x1000 <= _bankRom.size())
		memcpy(&_mem[dest], &_bankRom[srcOfs], 0x1000);
	else
		memset(&_mem[dest], 0x00, 0x1000);	// a bank the file does not have
	if (dest >= 0x8000)
		CopyDpcmMemory(dest, 0x1000);
	return;
}

// the APU reads the samples of the DPCM from its copy of $8000-$FFFF
void NSFPlayer::CopyDpcmMemory(UINT32 addr, UINT32 length)
{
	NSF_CHIPDEV* apu = _dev[DEVKIND_APU];

	if (apu != NULL && apu->memWrite != NULL)
		apu->memWrite(apu->base.defInf.dataPtr, addr, length, &_mem[addr]);
	return;
}

// What the NSF specification has an NSF player do before INIT: the sound registers are cleared and the
// frame counter is in the 4-step mode. The channels are disabled while the registers are cleared (as
// they are after a power-up of the console): with the device's reset state, where they are enabled, the
// write to $400F would start the noise.
void NSFPlayer::InitSoundRegs(void)
{
	NSF_CHIPDEV* apu = _dev[DEVKIND_APU];
	UINT8 reg;

	apu->write8(apu->base.defInf.dataPtr, 0x15, 0x00);
	for (reg = 0x00; reg <= 0x13; reg ++)
		apu->write8(apu->base.defInf.dataPtr, reg, 0x00);
	apu->write8(apu->base.defInf.dataPtr, 0x15, 0x00);
	apu->write8(apu->base.defInf.dataPtr, 0x15, 0x0F);
	apu->write8(apu->base.defInf.dataPtr, 0x17, 0x40);
	return;
}

// The start of the song, after the devices were reset: the memory, the sound registers, then INIT
// with the song number in A and the region in X.
void NSFPlayer::StartSong(void)
{
	BuildMemory();
	CopyDpcmMemory(0x8000, 0x8000);
	InitSoundRegs();
	_mulA = 0;
	_mulB = 0;

	nsf_cpu6502_power(&_cpu);
	_cpu.a = (UINT8)_curSong;
	_cpu.x = _region;
	_cpu.y = 0;
	_cpuCycle = 0;
	_playBase = 0;
	_playIndex = 0;
	_nextPlay = 0;
	CallRoutine(_fileHdr.initAddr);
	_phase = PHASE_INIT;
	return;
}

/*static*/ UINT8 NSFPlayer::BusRead(void* ctx, UINT16 addr)
{
	return static_cast<NSFPlayer*>(ctx)->CpuRead(addr);
}

/*static*/ void NSFPlayer::BusWrite(void* ctx, UINT16 addr, UINT8 value)
{
	static_cast<NSFPlayer*>(ctx)->CpuWrite(addr, value);
	return;
}

// Every access of the CPU is one cycle: the counter is the clock of the song.
UINT8 NSFPlayer::CpuRead(UINT16 addr)
{
	UINT64 cycle = _cpuCycle ++;

	if (addr >= 0x6000)
		return _mem[addr];
	if (addr < 0x2000)
		return _mem[addr & 0x07FF];	// the RAM, mirrored
	return ReadIO(addr, cycle);
}

void NSFPlayer::CpuWrite(UINT16 addr, UINT8 value)
{
	UINT64 cycle = _cpuCycle ++;

	if (addr < 0x2000)
		_mem[addr & 0x07FF] = value;
	else if (addr >= 0x6000)
		WriteHigh(addr, value, cycle);
	else
		WriteIO(addr, value, cycle);
	return;
}

// The registers the song reads: they come from the devices as they are in the sample of the read.
UINT8 NSFPlayer::ReadIO(UINT16 addr, UINT64 cycle)
{
	if (addr >= 0x4000 && addr < 0x4018)
	{
		if (addr == 0x4015)
		{
			NSF_CHIPDEV* apu = _dev[DEVKIND_APU];

			SyncDevices(CycleToSample(cycle));
			return apu->read8(apu->base.defInf.dataPtr, 0x15);
		}
		return 0x40;	// the controllers are not there
	}
	if ((_fileHdr.chips & NSF_CHIP_FDS) && addr >= 0x4040 && addr <= 0x4097)
	{
		NSF_CHIPDEV* apu = _dev[DEVKIND_APU];

		SyncDevices(CycleToSample(cycle));
		return apu->read8(apu->base.defInf.dataPtr, (UINT8)(addr & 0xFF));
	}
	if (_dev[DEVKIND_N163] != NULL && addr >= 0x4800 && addr <= 0x4FFF)
	{
		NSF_CHIPDEV* n163 = _dev[DEVKIND_N163];

		SyncDevices(CycleToSample(cycle));
		return n163->read16(n163->base.defInf.dataPtr, addr);
	}
	if (_dev[DEVKIND_MMC5] != NULL)
	{
		if (addr == 0x5205)
			return (UINT8)(_mulA * _mulB);
		if (addr == 0x5206)
			return (UINT8)((_mulA * _mulB) >> 8);
		if (addr == 0x5015)
		{
			NSF_CHIPDEV* mmc5 = _dev[DEVKIND_MMC5];

			SyncDevices(CycleToSample(cycle));
			return mmc5->read16(mmc5->base.defInf.dataPtr, addr);
		}
		if (addr >= 0x5C00 && addr <= 0x5FF5)
			return _mem[addr];	// ExRAM
	}
	return (UINT8)(addr >> 8);	// nothing there: the data bus holds what the address put on it
}

// The registers below the program, $2000-$5FFF.
void NSFPlayer::WriteIO(UINT16 addr, UINT8 value, UINT64 cycle)
{
	if (addr >= 0x4000 && addr < 0x4018)
	{
		// $4014 (sprite DMA) and $4016 (controllers) are not the APU's
		if (addr != 0x4014 && addr != 0x4016)
		{
			NSF_CHIPDEV* apu = _dev[DEVKIND_APU];

			SyncDevices(CycleToSample(cycle));
			apu->write8(apu->base.defInf.dataPtr, (UINT8)(addr & 0xFF), value);
		}
		return;
	}
	if ((_fileHdr.chips & NSF_CHIP_FDS) && addr >= 0x4040 && addr <= 0x4097)
	{
		NSF_CHIPDEV* apu = _dev[DEVKIND_APU];

		SyncDevices(CycleToSample(cycle));
		apu->write8(apu->base.defInf.dataPtr, (UINT8)(addr & 0xFF), value);
		return;
	}
	if (_dev[DEVKIND_N163] != NULL && addr >= 0x4800 && addr <= 0x4FFF)
	{
		NSF_CHIPDEV* n163 = _dev[DEVKIND_N163];

		SyncDevices(CycleToSample(cycle));
		n163->write16(n163->base.defInf.dataPtr, addr, value);
		return;
	}
	if (_dev[DEVKIND_MMC5] != NULL)
	{
		if ((addr >= 0x5000 && addr <= 0x5007) || addr == 0x5010 || addr == 0x5011 || addr == 0x5015)
		{
			NSF_CHIPDEV* mmc5 = _dev[DEVKIND_MMC5];

			SyncDevices(CycleToSample(cycle));
			mmc5->write16(mmc5->base.defInf.dataPtr, addr, value);
			return;
		}
		if (addr == 0x5205)
		{
			_mulA = value;
			return;
		}
		if (addr == 0x5206)
		{
			_mulB = value;
			return;
		}
		if (addr >= 0x5C00 && addr <= 0x5FF5)
		{
			_mem[addr] = value;	// ExRAM
			return;
		}
	}
	if (_fileHdr.bankSwitched && addr >= 0x5FF6 && addr <= 0x5FFF)
	{
		// $5FF6/$5FF7 exist when the FDS has RAM at $6000-$7FFF
		if (addr >= 0x5FF8 || (_fileHdr.chips & NSF_CHIP_FDS))
			SetBank(addr, value);
	}
	return;
}

// $6000-$FFFF: the RAM at $6000-$7FFF, the registers of the mapper chips and, with the FDS, the RAM
// at $8000-$DFFF. The rest is read-only memory.
void NSFPlayer::WriteHigh(UINT16 addr, UINT8 value, UINT64 cycle)
{
	bool isReg = false;

	if (addr >= 0x8000)
	{
		if (_dev[DEVKIND_VRC6] != NULL &&
			(addr & 0xFFFC) >= 0x9000 && (addr & 0xFFFC) <= 0xB000 && (addr & 0x0FFC) == 0x0000)
		{
			// $9000-$9003, $A000-$A002, $B000-$B002; the mirrors of the VRC6 are not used
			NSF_CHIPDEV* vrc6 = _dev[DEVKIND_VRC6];

			SyncDevices(CycleToSample(cycle));
			vrc6->write16(vrc6->base.defInf.dataPtr, addr, value);
			isReg = true;
		}
		if (_dev[DEVKIND_VRC7] != NULL && (addr == 0x9010 || addr == 0x9030))
		{
			NSF_CHIPDEV* vrc7 = _dev[DEVKIND_VRC7];

			SyncDevices(CycleToSample(cycle));
			vrc7->write8(vrc7->base.defInf.dataPtr, (addr == 0x9030) ? 0x01 : 0x00, value);
			isReg = true;
		}
		if (_dev[DEVKIND_5B] != NULL && addr >= 0xC000)
		{
			NSF_CHIPDEV* s5b = _dev[DEVKIND_5B];

			SyncDevices(CycleToSample(cycle));
			s5b->write16(s5b->base.defInf.dataPtr, addr, value);
			isReg = true;
		}
		if (_dev[DEVKIND_N163] != NULL && (addr >= 0xF800 || (addr >= 0xE000 && addr <= 0xE7FF && _dev[DEVKIND_5B] == NULL)))
		{
			// $F800 is the address port. $E000 disables the chip, which the data port of the
			// 5B would do all the time when both are there.
			NSF_CHIPDEV* n163 = _dev[DEVKIND_N163];

			SyncDevices(CycleToSample(cycle));
			n163->write16(n163->base.defInf.dataPtr, addr, value);
			isReg = true;
		}
	}

	if (addr < 0x8000)
		_mem[addr] = value;
	else if ((_fileHdr.chips & NSF_CHIP_FDS) && addr < 0xE000 && ! isReg)
		_mem[addr] = value;
	return;
}
