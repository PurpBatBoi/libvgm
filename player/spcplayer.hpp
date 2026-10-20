#ifndef __SPCPLAYER_HPP__
#define __SPCPLAYER_HPP__

#include "../stdtype.h"
#include "../emu/EmuStructs.h"
#include "../emu/Resampler.h"
#include "../utils/StrUtils.h"
#include "helper.h"
#include "playerbase.hpp"
#include "../utils/DataLoader.h"
#include <vector>
#include <map>
#include <string>


#define FCC_SPC 	0x53504300

// SNES SPC700 Sound File (.spc): a snapshot of the SNES audio unit (SPC700 CPU registers, the
// 64 KB RAM and the 128 S-DSP registers), played by running the audio unit itself.
//
// Header (offsets of the fields the player reads, see the format description):
//	Ofs	Len	Description
//	000	21	"SNES-SPC700 Sound File Data v0.30" (the part up to the 'v' is checked)
//	021	02	26, 26
//	023	01	26 = header has ID666 tag, 27 = no ID666 tag
//	02E	D2	ID666 tag, text or binary form (see SPCPlayer::LoadTags)
//	100	10000	RAM
//	10100	80	DSP registers
//	101C0	40	extra RAM
//	10200	..	optional extended ID666 ("xid6") chunk
//
// The song length comes from the tags. A file without a length plays until the host stops it:
// GetTotalTicks() is 0 and the player never reaches PLAYSTATE_END.

struct SPC_HEADER
{
	UINT8 hasID666;			// header byte 0x23 == 26
	UINT8 hasXID6;			// an "xid6" chunk follows the machine image
	UINT8 id666Binary;		// the ID666 tag is in binary form (otherwise text form)
};

struct SPC_PLAY_OPTIONS
{
	PLR_GEN_OPTS genOpts;
};


class SPCPlayer : public PlayerBase
{
private:
	struct DevCfg
	{
		DEV_ID type;
		UINT16 volume;
		std::vector<UINT8> data;
	};
	struct SPC_CHIPDEV
	{
		VGM_BASEDEV base;
		DEVFUNC_WRITE_BLOCK memWrite;
	};

public:
	SPCPlayer();
	~SPCPlayer();

	UINT32 GetPlayerType(void) const;
	const char* GetPlayerName(void) const;
	static UINT8 PlayerCanLoadFile(DATA_LOADER *dataLoader);
	UINT8 CanLoadFile(DATA_LOADER *dataLoader) const;
	UINT8 LoadFile(DATA_LOADER *dataLoader);
	UINT8 UnloadFile(void);
	const SPC_HEADER* GetFileHeader(void) const;

	const char* const* GetTags(void);
	UINT8 GetSongInfo(PLR_SONG_INFO& songInf);
	UINT8 GetSongDeviceInfo(std::vector<PLR_DEV_INFO>& devInfList) const;
	UINT8 SetDeviceOptions(UINT32 id, const PLR_DEV_OPTS& devOpts);
	UINT8 GetDeviceOptions(UINT32 id, PLR_DEV_OPTS& devOpts) const;
	UINT8 SetDeviceMuting(UINT32 id, const PLR_MUTE_OPTS& muteOpts);
	UINT8 GetDeviceMuting(UINT32 id, PLR_MUTE_OPTS& muteOpts) const;
	UINT8 SetPlayerOptions(const SPC_PLAY_OPTIONS& playOpts);
	UINT8 GetPlayerOptions(SPC_PLAY_OPTIONS& playOpts) const;

	UINT8 SetSampleRate(UINT32 sampleRate);
	double GetPlaybackSpeed(void) const;
	UINT8 SetPlaybackSpeed(double speed);
	UINT32 Tick2Sample(UINT32 ticks) const;
	UINT32 Sample2Tick(UINT32 samples) const;
	double Tick2Second(UINT32 ticks) const;

	UINT8 GetState(void) const;
	UINT32 GetCurPos(UINT8 unit) const;
	UINT32 GetCurLoop(void) const;
	UINT32 GetTotalTicks(void) const;	// get time for playing once in ticks (0 = plays forever)
	UINT32 GetLoopTicks(void) const;	// get time for one loop in ticks (always 0: no loops)

	UINT8 Start(void);
	UINT8 Stop(void);
	UINT8 Reset(void);
	UINT8 Seek(UINT8 unit, UINT32 pos);
	UINT32 Render(UINT32 smplCnt, WAVE_32BS* data);

private:
	size_t DeviceID2OptionID(UINT32 id) const;
	void RefreshMuting(SPC_CHIPDEV& chipDev, const PLR_MUTE_OPTS& muteOpts);

	static bool IsTextID666(const UINT8* hdr);
	void CalcSongLength(void);
	UINT8 LoadTags(void);
	void LoadID666Tags(const UINT8* hdr);
	void LoadXID6Tags(void);
	void LoadTag(const char* tagName, const char* startPtr, size_t maxlen);
	void SetTagString(const char* tagName, const std::string& value);
	std::string GetUTF8String(const char* startPtr, const char* endPtr);

	void RefreshTSRates(void);

	static void PlayerLogCB(void* userParam, void* source, UINT8 level, const char* message);
	static void SndEmuLogCB(void* userParam, void* source, UINT8 level, const char* message);

	void GenerateDeviceConfig(void);
	UINT8 SeekToTick(UINT32 tick);
	void ApplyFade(UINT32 smplCnt, WAVE_32BS* data) const;
	void DoFileEnd(void);

	CPCONV* _cpc1252;	// CP1252 -> UTF-8 codepage conversion
	DATA_LOADER* _dLoad;
	UINT32 _fileLen;
	const UINT8* _fileData;	// data pointer for quick access, equals DataLoader_GetData(_dLoad)

	SPC_HEADER _fileHdr;
	std::vector<DevCfg> _devCfgs;
	UINT32 _tickFreq;
	UINT32 _playTicks;	// length of the song without the fade (0 = no length, plays forever)
	UINT32 _fadeTicks;	// length of the fade-out after _playTicks
	UINT32 _totalTicks;	// _playTicks + _fadeTicks (0 = plays forever)
	// the length fields of the tags
	UINT32 _id666Secs;		// ID666: seconds to play
	UINT32 _id666FadeMs;	// ID666: fade length in milliseconds
	UINT8 _xHasLen;			// xid6: at least one of introduction / loop / end length
	INT32 _xIntro;			// xid6 30h: introduction length (1/64000 s)
	INT32 _xLoop;			// xid6 31h: loop length (1/64000 s)
	INT32 _xEnd;			// xid6 32h: end length (1/64000 s, can be negative)
	UINT8 _xLoopCount;		// xid6 35h: number of times to play the loop (default 1)
	UINT8 _xHasFade;		// xid6 33h present
	UINT32 _xFade;			// xid6 33h: fade length (1/64000 s)
	std::map<std::string, std::string> _tagData;
	std::vector<const char*> _tagList;

	// tick/sample conversion rates
	UINT64 _tsMult;
	UINT64 _tsDiv;

	SPC_PLAY_OPTIONS _playOpts;
	PLR_DEV_OPTS _devOpts;
	std::vector<SPC_CHIPDEV> _devices;
	std::vector<std::string> _devNames;
	size_t _optDevMap;	// index of the SNES APU in _devices (-1 = not started)

	UINT32 _devTick;	// device position in ticks (frames the device has rendered, up to its resampler buffer)
	UINT32 _playSmpl;	// output sample position
	UINT32 _playTick;	// position in ticks (derived from _playSmpl, or the target of a seek)

	UINT8 _playState;
};

#endif	// __SPCPLAYER_HPP__
